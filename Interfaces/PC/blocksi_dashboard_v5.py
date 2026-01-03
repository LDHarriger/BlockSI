#!/usr/bin/env python3
"""
BlockSI Dashboard v5 - Fixed server architecture with power control features

Key fixes from broken v5:
- No st.rerun() that destroys connections
- Correct DATA format parsing (esp_ts,o3,temp,press,sample_v,ref_v,day,month,year,hour,minute,second)
- Stable TCP server that persists across Streamlit reruns
- Uses streamlit-autorefresh for updates

Run with: streamlit run blocksi_dashboard_v5.py -- --port 5000
"""

import streamlit as st
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import socket
import threading
import time
import os
import csv
import sys
from datetime import datetime, timedelta
from dataclasses import dataclass, field
from typing import Optional, List
from collections import deque

# Try to import streamlit-autorefresh
try:
    from streamlit_autorefresh import st_autorefresh
    HAS_AUTOREFRESH = True
except ImportError:
    HAS_AUTOREFRESH = False

# =============================================================================
# Configuration
# =============================================================================

DEFAULT_PORT = 5000
MAX_SAMPLES = 7200

O3_WARNING_PPM = 0.1
O3_DANGER_PPM = 0.3  
O3_CRITICAL_PPM = 1.0

O3_FLOW_COEFF_A = 1.78
O3_FLOW_COEFF_B = 1.40
POWER_THRESHOLD_PCT = 20
POWER_SATURATION_PCT = 75

# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class Sample:
    timestamp: datetime
    o3_pct: float
    temp_c: float
    pressure_mbar: float
    sample_v: float = 0.0
    ref_v: float = 0.0
    room_o3_ppm: Optional[float] = None
    vessel_temp_c: Optional[float] = None
    power_pct: int = 0
    flow_lpm: float = 5.0

@dataclass 
class DeviceState:
    connected: bool = False
    client_ip: str = ""
    device_info: str = ""
    ozone_gen: bool = False
    o2_conc: bool = False
    heap_free: int = 0
    error_count: int = 0
    last_data_time: Optional[datetime] = None
    backup_recording: bool = False
    storage_total: int = 0
    storage_used: int = 0
    storage_free: int = 0
    m106h_avg_time: str = "unknown"
    m106h_avg_option: int = 0
    m106h_logging: bool = False
    power_pct: int = 0
    flow_lpm: float = 5.0
    dac_available: bool = False
    room_o3_ppm: float = 0.0
    room_o3_alarm: bool = False
    vessel_temp_c: Optional[float] = None
    lab_o3_available: bool = False
    thermocouple_available: bool = False

# =============================================================================
# O3 Prediction Model
# =============================================================================

def predict_o3_output(flow_lpm: float, power_pct: int) -> float:
    if flow_lpm <= 0:
        return 0.0
    o3_max = O3_FLOW_COEFF_A / flow_lpm + O3_FLOW_COEFF_B
    if power_pct < POWER_THRESHOLD_PCT:
        power_factor = 0.0
    elif power_pct > POWER_SATURATION_PCT:
        base = (POWER_SATURATION_PCT - POWER_THRESHOLD_PCT) / (100 - POWER_THRESHOLD_PCT)
        extra = (power_pct - POWER_SATURATION_PCT) / (100 - POWER_SATURATION_PCT)
        power_factor = base + extra * 0.2
    else:
        power_factor = (power_pct - POWER_THRESHOLD_PCT) / (100 - POWER_THRESHOLD_PCT)
    return o3_max * power_factor

def get_flow_o3_curve(power_pct: int, flow_range=(1.0, 10.0)) -> tuple:
    flows = np.linspace(flow_range[0], flow_range[1], 50)
    o3_values = [predict_o3_output(f, power_pct) for f in flows]
    return flows, o3_values

def get_power_o3_curve(flow_lpm: float, power_range=(0, 100)) -> tuple:
    powers = np.arange(power_range[0], power_range[1] + 1)
    o3_values = [predict_o3_output(flow_lpm, p) for p in powers]
    return powers, o3_values

# =============================================================================
# TCP Server
# =============================================================================

class BlockSIServer:
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls, port=DEFAULT_PORT):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
            return cls._instance
    
    def __init__(self, port=DEFAULT_PORT):
        if self._initialized:
            return
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = False
        self.thread = None
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.state = DeviceState()
        self.data_lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.pc_recording = False
        self.pc_record_file = None
        self.pc_record_writer = None
        self.pc_record_filename = ""
        self.pc_record_count = 0
        self.stream_file = None
        self.stream_writer = None
        self._initialized = True
    
    def start(self):
        if self.running:
            return True
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)
            self.running = True
            self.thread = threading.Thread(target=self._server_loop, daemon=True)
            self.thread.start()
            self._start_stream_log()
            print(f"[SERVER] Started on port {self.port}")
            return True
        except Exception as e:
            print(f"[SERVER] Failed: {e}")
            return False
    
    def _start_stream_log(self):
        filename = f"{datetime.now().strftime('%Y-%m-%d')}_Stream.csv"
        try:
            file_exists = os.path.exists(filename)
            self.stream_file = open(filename, 'a', newline='')
            self.stream_writer = csv.writer(self.stream_file)
            if not file_exists:
                self.stream_writer.writerow(['timestamp', 'o3_pct', 'temp_c', 'pressure_mbar', 'sample_v', 'ref_v'])
        except Exception as e:
            print(f"[STREAM] Failed: {e}")
    
    def _server_loop(self):
        buffer = ""
        while self.running:
            if not self.state.connected:
                try:
                    self.client_socket, addr = self.server_socket.accept()
                    self.client_socket.settimeout(0.1)
                    self.state.connected = True
                    self.state.client_ip = f"{addr[0]}:{addr[1]}"
                    print(f"[SERVER] Connected: {self.state.client_ip}")
                    buffer = ""
                    self._send_cmd("status")
                    self._send_cmd("relay_get")
                    self._send_cmd("sensors_get")
                except socket.timeout:
                    continue
                except:
                    continue
            try:
                data = self.client_socket.recv(4096)
                if not data:
                    self._handle_disconnect()
                    continue
                buffer += data.decode('utf-8', errors='replace')
                self.state.last_data_time = datetime.now()
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self._process_line(line)
            except socket.timeout:
                continue
            except:
                self._handle_disconnect()
    
    def _handle_disconnect(self):
        if self.state.connected:
            print("[SERVER] Disconnected")
        self.state.connected = False
        self.state.client_ip = ""
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
    
    def _process_line(self, line):
        if line.startswith('DATA,'):
            self._parse_data(line)
        elif line.startswith('RSP,'):
            self._parse_response(line)
        elif line.startswith('HELLO,'):
            self.state.device_info = line[6:]
        elif line.startswith('FILE,'):
            self._parse_file(line)
    
    def _parse_data(self, line):
        """Parse: DATA,esp_ts,o3,temp,press,sample_v,ref_v,day,month,year,hour,minute,second"""
        parts = line.split(',')
        if len(parts) < 7:
            return
        try:
            sample = Sample(
                timestamp=datetime.now(),
                o3_pct=float(parts[2]),
                temp_c=float(parts[3]),
                pressure_mbar=float(parts[4]),
                sample_v=float(parts[5]),
                ref_v=float(parts[6])
            )
            sample.room_o3_ppm = self.state.room_o3_ppm
            sample.vessel_temp_c = self.state.vessel_temp_c
            sample.power_pct = self.state.power_pct
            sample.flow_lpm = self.state.flow_lpm
            with self.data_lock:
                self.samples.append(sample)
            if self.stream_writer:
                ts = sample.timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                self.stream_writer.writerow([ts, sample.o3_pct, sample.temp_c, sample.pressure_mbar, sample.sample_v, sample.ref_v])
                self.stream_file.flush()
            if self.pc_recording and self.pc_record_writer:
                ts = sample.timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                self.pc_record_writer.writerow([ts, sample.o3_pct, sample.temp_c, sample.pressure_mbar, sample.sample_v, sample.ref_v])
                self.pc_record_file.flush()
                self.pc_record_count += 1
        except (ValueError, IndexError) as e:
            self.state.error_count += 1
    
    def _parse_response(self, line):
        parts = line.split(',', 3)
        if len(parts) < 3:
            return
        status, cmd = parts[1], parts[2]
        details = parts[3] if len(parts) > 3 else ""
        if cmd == 'relay_get' and status == 'OK':
            for p in details.split(','):
                if '=' in p:
                    k, v = p.split('=', 1)
                    if k == 'ozone_gen': self.state.ozone_gen = v == '1'
                    elif k == 'o2_conc': self.state.o2_conc = v == '1'
        elif cmd == 'status' and status == 'OK':
            for p in details.split(','):
                if '=' in p:
                    k, v = p.split('=', 1)
                    if k == 'heap': self.state.heap_free = int(v)
        elif cmd == 'sensors_get' and status == 'OK':
            for p in details.split(','):
                if '=' in p:
                    k, v = p.split('=', 1)
                    if k == 'dac': self.state.dac_available = v.lower() == 'ok'
                    elif k == 'lab_o3': self.state.lab_o3_available = v.lower() == 'ok'
                    elif k == 'thermo': self.state.thermocouple_available = v.lower() == 'ok'
                    elif k == 'room_o3':
                        try: self.state.room_o3_ppm = float(v)
                        except: pass
                    elif k == 'vessel_temp':
                        try: self.state.vessel_temp_c = float(v)
                        except: pass
        elif cmd == 'power_get' and status == 'OK':
            for p in details.split(','):
                if '=' in p:
                    k, v = p.split('=', 1)
                    if k == 'pct': self.state.power_pct = int(v)
                    elif k == 'flow': self.state.flow_lpm = float(v)
    
    def _parse_file(self, line):
        parts = line.split(',')
        if len(parts) >= 2:
            if parts[1].upper() == 'START': self.state.backup_recording = True
            elif parts[1].upper() == 'STOP': self.state.backup_recording = False
    
    def _send_cmd(self, cmd, args=""):
        if not self.state.connected or not self.client_socket:
            return False
        full = f"CMD,{cmd}" + (f",{args}" if args else "")
        try:
            with self.send_lock:
                self.client_socket.sendall((full + '\n').encode())
            return True
        except:
            return False
    
    def send_command(self, cmd, args=""):
        return self._send_cmd(cmd, args)
    
    def start_pc_recording(self, filename=None):
        if self.pc_recording:
            return self.pc_record_filename
        if not filename:
            filename = f"blocksi_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            self.pc_record_file = open(filename, 'w', newline='')
            self.pc_record_writer = csv.writer(self.pc_record_file)
            self.pc_record_writer.writerow(['timestamp', 'o3_pct', 'temp_c', 'pressure_mbar', 'sample_v', 'ref_v'])
            self.pc_record_filename = filename
            self.pc_record_count = 0
            self.pc_recording = True
            return filename
        except:
            return ""
    
    def stop_pc_recording(self):
        if not self.pc_recording:
            return ("", 0)
        fn, cnt = self.pc_record_filename, self.pc_record_count
        if self.pc_record_file:
            self.pc_record_file.close()
        self.pc_record_file = None
        self.pc_record_writer = None
        self.pc_recording = False
        self.pc_record_filename = ""
        self.pc_record_count = 0
        return (fn, cnt)
    
    def get_dataframe(self, minutes=10):
        cutoff = datetime.now() - timedelta(minutes=minutes)
        with self.data_lock:
            recent = [s for s in self.samples if s.timestamp > cutoff]
        if not recent:
            return pd.DataFrame()
        return pd.DataFrame([{'timestamp': s.timestamp, 'o3_pct': s.o3_pct, 'temp_c': s.temp_c,
            'pressure_mbar': s.pressure_mbar, 'sample_v': s.sample_v, 'ref_v': s.ref_v,
            'room_o3_ppm': s.room_o3_ppm, 'vessel_temp_c': s.vessel_temp_c,
            'power_pct': s.power_pct, 'flow_lpm': s.flow_lpm} for s in recent])
    
    def get_sample_count(self):
        return len(self.samples)
    
    def is_connected(self):
        return self.state.connected and self.client_socket is not None

def get_server(port=DEFAULT_PORT):
    if 'server' not in st.session_state:
        server = BlockSIServer(port)
        server.start()
        st.session_state.server = server
    return st.session_state.server

# =============================================================================
# UI
# =============================================================================

def render_sidebar():
    server = get_server()
    with st.sidebar:
        st.title("🧪 BlockSI Control")
        st.subheader("Connection")
        if server.is_connected():
            st.success("🟢 ESP32 Connected")
            st.caption(f"From: {server.state.client_ip}")
            if server.state.last_data_time:
                age = (datetime.now() - server.state.last_data_time).seconds
                st.caption(f"Last: {age}s ago")
        else:
            st.error("🔴 Disconnected")
            st.caption(f"Port {server.port}")
        
        if server.state.room_o3_ppm >= O3_DANGER_PPM:
            st.warning(f"⚠️ Room O3: {server.state.room_o3_ppm:.3f} ppm")
        
        st.divider()
        col1, col2 = st.columns(2)
        with col1: st.metric("Samples", server.get_sample_count())
        with col2: st.metric("Errors", server.state.error_count)
        
        sensors = []
        sensors.append("✅DAC" if server.state.dac_available else "❌DAC")
        sensors.append("✅O3" if server.state.lab_o3_available else "❌O3")
        sensors.append("✅TC" if server.state.thermocouple_available else "❌TC")
        st.caption(" ".join(sensors))
        
        st.divider()
        st.subheader("Recording")
        if server.pc_recording:
            st.success(f"📝 {server.pc_record_count} samples")
            if st.button("⏹️ Stop", use_container_width=True): server.stop_pc_recording()
        else:
            if st.button("▶️ Start", use_container_width=True, disabled=not server.is_connected()):
                server.start_pc_recording()
        
        st.divider()
        time_range = st.selectbox("Time", [1,5,10,30,60], index=2, format_func=lambda x: f"{x}m")
        st.session_state.time_range = time_range
        if st.button("🔄 Refresh", use_container_width=True, disabled=not server.is_connected()):
            server.send_command("status")
            server.send_command("relay_get")
            server.send_command("sensors_get")
            server.send_command("power_get")

def render_live_data():
    server = get_server()
    time_range = st.session_state.get('time_range', 10)
    if not server.is_connected():
        st.info("Waiting for ESP32...")
        return
    df = server.get_dataframe(minutes=time_range)
    if df.empty:
        st.info("Waiting for data...")
        return
    
    latest = df.iloc[-1]
    cols = st.columns(5)
    with cols[0]: st.metric("O3 (%)", f"{latest['o3_pct']:.4f}")
    with cols[1]: st.metric("Temp (°C)", f"{latest['temp_c']:.1f}")
    with cols[2]: st.metric("Press (mbar)", f"{latest['pressure_mbar']:.1f}")
    with cols[3]: st.metric("Room O3", f"{server.state.room_o3_ppm:.4f}" if server.state.room_o3_ppm else "-")
    with cols[4]: st.metric("Vessel", f"{server.state.vessel_temp_c:.1f}" if server.state.vessel_temp_c else "-")
    
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.1,
        subplot_titles=("O3", "Temperature"), row_heights=[0.6, 0.4])
    fig.add_trace(go.Scatter(x=df['timestamp'], y=df['o3_pct'], name='O3', line=dict(color='blue')), row=1, col=1)
    fig.add_trace(go.Scatter(x=df['timestamp'], y=df['temp_c'], name='Temp', line=dict(color='green')), row=2, col=1)
    fig.update_layout(height=450, showlegend=True, margin=dict(l=50,r=50,t=60,b=40))
    st.plotly_chart(fig, use_container_width=True)
    with st.expander("Data Table"):
        st.dataframe(df.tail(30), use_container_width=True)

def render_power_control():
    server = get_server()
    st.header("🎛️ Power Control")
    if not server.is_connected():
        st.warning("Connect ESP32 first")
        return
    if not server.state.dac_available:
        st.error("DAC not available")
        if st.button("Refresh Sensors"): server.send_command("sensors_get")
        return
    
    col1, col2 = st.columns(2)
    with col1:
        power = st.slider("Power (%)", 0, 100, server.state.power_pct)
        if power < POWER_THRESHOLD_PCT: st.info("Below threshold")
        elif power > POWER_SATURATION_PCT: st.warning("Saturation zone")
        else: st.success("Active range")
        if st.button("Apply", type="primary", use_container_width=True):
            server.send_command("power_set", str(power))
            server.state.power_pct = power
        if st.button("🛑 STOP (0%)", use_container_width=True):
            server.send_command("power_set", "0")
            server.state.power_pct = 0
        st.divider()
        flow = st.number_input("Flow (LPM)", 0.5, 10.0, server.state.flow_lpm, 0.5)
        if st.button("Set Flow"):
            server.send_command("flow_set", f"{flow:.1f}")
            server.state.flow_lpm = flow
        st.metric("Predicted O3", f"{predict_o3_output(flow, power):.2f}%")
    
    with col2:
        fig = make_subplots(rows=2, cols=1, subplot_titles=("O3 vs Flow", "O3 vs Power"), vertical_spacing=0.15)
        flows, o3f = get_flow_o3_curve(power)
        pred = predict_o3_output(flow, power)
        fig.add_trace(go.Scatter(x=flows, y=o3f, line=dict(color='blue')), row=1, col=1)
        fig.add_trace(go.Scatter(x=[flow], y=[pred], mode='markers', marker=dict(color='red', size=10)), row=1, col=1)
        powers, o3p = get_power_o3_curve(flow)
        fig.add_trace(go.Scatter(x=powers, y=o3p, line=dict(color='green')), row=2, col=1)
        fig.add_trace(go.Scatter(x=[power], y=[pred], mode='markers', marker=dict(color='red', size=10)), row=2, col=1)
        fig.add_vline(x=POWER_THRESHOLD_PCT, line_dash="dash", line_color="orange", row=2, col=1)
        fig.add_vline(x=POWER_SATURATION_PCT, line_dash="dash", line_color="red", row=2, col=1)
        fig.update_layout(height=450, showlegend=False)
        st.plotly_chart(fig, use_container_width=True)

def render_relay_control():
    server = get_server()
    st.header("⚡ Relays")
    if not server.is_connected():
        st.warning("Connect first")
        return
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("O3 Generator")
        st.markdown(f"**{'🟢 ON' if server.state.ozone_gen else '🔴 OFF'}**")
        c1, c2 = st.columns(2)
        with c1:
            if st.button("ON", key="o3on", disabled=server.state.ozone_gen):
                server.send_command("relay_set", "ozone_gen,1")
                server.state.ozone_gen = True
        with c2:
            if st.button("OFF", key="o3off", disabled=not server.state.ozone_gen):
                server.send_command("relay_set", "ozone_gen,0")
                server.state.ozone_gen = False
    with col2:
        st.subheader("O2 Concentrator")
        st.markdown(f"**{'🟢 ON' if server.state.o2_conc else '🔴 OFF'}**")
        c1, c2 = st.columns(2)
        with c1:
            if st.button("ON", key="o2on", disabled=server.state.o2_conc):
                server.send_command("relay_set", "o2_conc,1")
                server.state.o2_conc = True
        with c2:
            if st.button("OFF", key="o2off", disabled=not server.state.o2_conc):
                server.send_command("relay_set", "o2_conc,0")
                server.state.o2_conc = False
    if st.button("🛑 ALL OFF", use_container_width=True):
        server.send_command("relay_all_off")
        server.state.ozone_gen = False
        server.state.o2_conc = False

def render_sensors():
    server = get_server()
    st.header("🌡️ Sensors")
    if not server.is_connected():
        st.warning("Connect first")
        return
    if st.button("🔄 Refresh"):
        server.send_command("sensors_get")
    cols = st.columns(3)
    with cols[0]:
        st.subheader("DAC")
        if server.state.dac_available:
            st.success("✅ OK")
            st.metric("Power", f"{server.state.power_pct}%")
        else:
            st.error("❌ Missing")
    with cols[1]:
        st.subheader("Lab O3")
        if server.state.lab_o3_available:
            st.success("✅ OK")
            st.metric("Room O3", f"{server.state.room_o3_ppm:.4f} ppm")
        else:
            st.error("❌ Missing")
    with cols[2]:
        st.subheader("Thermocouple")
        if server.state.thermocouple_available:
            st.success("✅ OK")
            if server.state.vessel_temp_c and server.state.vessel_temp_c > -900:
                st.metric("Vessel", f"{server.state.vessel_temp_c:.1f}°C")
            else:
                st.warning("Fault")
        else:
            st.error("❌ Missing")

def render_106h():
    server = get_server()
    st.header("📊 106-H")
    if not server.is_connected():
        st.warning("Connect first")
        return
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("Averaging")
        opts = ["2s", "10s", "60s", "300s"]
        sel = st.selectbox("Time", opts, index=server.state.m106h_avg_option)
        if st.button("Set"):
            server.send_command("106h_avg_set", str(opts.index(sel)))
    with col2:
        st.subheader("Logging")
        st.markdown(f"**{'🟢 Active' if server.state.m106h_logging else '🔴 Off'}**")
        c1, c2 = st.columns(2)
        with c1:
            if st.button("Start"): server.send_command("106h_log_start")
        with c2:
            if st.button("Stop"): server.send_command("106h_log_stop")

def render_backup():
    server = get_server()
    st.header("📁 Backup")
    if not server.is_connected():
        st.warning("Connect first")
        return
    if st.button("Refresh"):
        server.send_command("backup_status")

def main():
    st.set_page_config(page_title="BlockSI Dashboard", page_icon="🔬", layout="wide")
    port = DEFAULT_PORT
    if '--port' in sys.argv:
        try: port = int(sys.argv[sys.argv.index('--port') + 1])
        except: pass
    get_server(port)
    if HAS_AUTOREFRESH:
        st_autorefresh(interval=2000, key="refresh")
    st.title("🔬 BlockSI Dashboard v5.0")
    render_sidebar()
    tabs = st.tabs(["📈 Live", "🎛️ Power", "⚡ Relays", "🌡️ Sensors", "📊 106-H", "📁 Backup"])
    with tabs[0]: render_live_data()
    with tabs[1]: render_power_control()
    with tabs[2]: render_relay_control()
    with tabs[3]: render_sensors()
    with tabs[4]: render_106h()
    with tabs[5]: render_backup()
    if not HAS_AUTOREFRESH:
        st.sidebar.caption("pip install streamlit-autorefresh")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
BlockSI Dashboard v3 - Streamlit-based control and monitoring interface

Features:
- Connection state properly tracked with timeout detection
- PC-side recording to CSV (simplified format: PC timestamp only)
- 106-H control: averaging time, internal logging, log download
- ESP32 backup storage management
- Relay controls

Run with: streamlit run blocksi_dashboard.py -- --port 5000
"""

import streamlit as st
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import socket
import threading
import time
import os
import csv
import sys
from datetime import datetime, timedelta
from queue import Queue, Empty
from dataclasses import dataclass, field
from typing import Optional, List
import io

# -----------------------------------------------------------------------------
# Data Classes
# -----------------------------------------------------------------------------

@dataclass
class Sample:
    timestamp: datetime
    o3: float
    temp: float
    press: float
    sample_v: float
    ref_v: float

@dataclass 
class DeviceState:
    connected: bool = False
    client_ip: str = ""
    device_info: str = ""
    ozone_gen: bool = False
    o2_conc: bool = False
    heap_free: int = 0
    error_count: int = 0
    backup_recording: bool = False
    backup_files: List = field(default_factory=list)
    storage_total: int = 0
    storage_used: int = 0
    storage_free: int = 0
    last_data_time: datetime = None
    # 106-H state
    m106h_avg_time: str = "unknown"
    m106h_avg_option: int = 0
    m106h_logging: bool = False

# -----------------------------------------------------------------------------
# TCP Server (singleton, runs in background)
# -----------------------------------------------------------------------------

class BlockSIServer:
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls, port=5000):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
            return cls._instance
    
    def __init__(self, port=5000):
        if self._initialized:
            return
            
        self.port = port
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # Data storage
        self.samples: List[Sample] = []
        self.max_samples = 3600
        self.state = DeviceState()
        
        # Thread safety
        self.data_lock = threading.Lock()
        self.send_lock = threading.Lock()
        
        # PC-side recording
        self.pc_recording = False
        self.pc_record_file = None
        self.pc_record_writer = None
        self.pc_record_filename = ""
        self.pc_record_count = 0
        
        # Stream logging (always on)
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
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            
            self._setup_stream_csv()
            print(f"[SERVER] Listening on port {self.port}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to start server: {e}")
            return False
    
    def _setup_stream_csv(self):
        """Setup continuous stream CSV file"""
        filename = f"{datetime.now().strftime('%Y-%m-%d')}_Stream.csv"
        file_exists = os.path.exists(filename)
        self.stream_file = open(filename, 'a', newline='')
        self.stream_writer = csv.writer(self.stream_file)
        if not file_exists:
            self.stream_writer.writerow([
                'timestamp', 'ozone_pct', 'temperature_c', 'pressure_mbar',
                'sample_v', 'ref_v'
            ])
            self.stream_file.flush()
        print(f"[CSV] Stream file: {filename}")
    
    def start_pc_recording(self, name: str) -> str:
        """Start recording to a named sequence file on PC"""
        if self.pc_recording:
            self.stop_pc_recording()
        
        self.pc_record_filename = f"{datetime.now().strftime('%Y-%m-%d')}_{name}.csv"
        self.pc_record_file = open(self.pc_record_filename, 'w', newline='')
        self.pc_record_writer = csv.writer(self.pc_record_file)
        self.pc_record_writer.writerow([
            'timestamp', 'ozone_pct', 'temperature_c', 'pressure_mbar',
            'sample_v', 'ref_v'
        ])
        self.pc_record_file.flush()
        self.pc_recording = True
        self.pc_record_count = 0
        print(f"[RECORD] Started: {self.pc_record_filename}")
        return self.pc_record_filename
    
    def stop_pc_recording(self) -> tuple:
        """Stop PC recording, return (filename, sample_count)"""
        if not self.pc_recording:
            return ("", 0)
        
        filename = self.pc_record_filename
        count = self.pc_record_count
        
        if self.pc_record_file:
            self.pc_record_file.close()
            self.pc_record_file = None
            self.pc_record_writer = None
        
        self.pc_recording = False
        self.pc_record_filename = ""
        self.pc_record_count = 0
        print(f"[RECORD] Stopped: {filename} ({count} samples)")
        return (filename, count)
    
    def send_command(self, cmd: str) -> bool:
        """Send command to ESP32"""
        if not self.state.connected or not self.client_socket:
            return False
        try:
            with self.send_lock:
                self.client_socket.sendall((cmd + '\n').encode())
            return True
        except Exception as e:
            print(f"[ERROR] Send failed: {e}")
            self._handle_disconnect()
            return False
    
    def _handle_disconnect(self):
        """Handle client disconnection"""
        print("[DISCONNECTED] ESP32")
        self.state.connected = False
        self.state.client_ip = ""
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
    
    def _process_line(self, line: str):
        line = line.strip()
        if not line:
            return
        
        if line.startswith('DATA,'):
            self._process_data(line)
        elif line.startswith('RSP,'):
            self._process_response(line)
        elif line.startswith('HELLO,'):
            self.state.device_info = line[6:]
            print(f"[DEVICE] {self.state.device_info}")
        elif line.startswith('FILE,'):
            self._process_file_notification(line)
        elif line.startswith('LOG,'):
            self._process_log_data(line)
    
    def _process_data(self, line: str):
        parts = line.split(',')
        if len(parts) < 7:
            return
        try:
            now = datetime.now()
            sample = Sample(
                timestamp=now,
                o3=float(parts[2]),
                temp=float(parts[3]),
                press=float(parts[4]),
                sample_v=float(parts[5]),
                ref_v=float(parts[6])
            )
            
            with self.data_lock:
                self.samples.append(sample)
                if len(self.samples) > self.max_samples:
                    self.samples.pop(0)
                self.state.last_data_time = now
            
            # Format timestamp for CSV (pandas-compatible ISO format)
            ts_str = now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # millisecond precision
            
            # Log to stream CSV
            if self.stream_writer:
                self.stream_writer.writerow([
                    ts_str, sample.o3, sample.temp, sample.press,
                    sample.sample_v, sample.ref_v
                ])
                self.stream_file.flush()
            
            # Log to recording CSV if active
            if self.pc_recording and self.pc_record_writer:
                self.pc_record_writer.writerow([
                    ts_str, sample.o3, sample.temp, sample.press,
                    sample.sample_v, sample.ref_v
                ])
                self.pc_record_file.flush()
                self.pc_record_count += 1
                
        except (ValueError, IndexError) as e:
            print(f"[ERROR] Parse: {e}")
    
    def _process_response(self, line: str):
        parts = line.split(',', 3)
        if len(parts) >= 3:
            status = parts[1]
            cmd = parts[2]
            details = parts[3] if len(parts) > 3 else ""
            
            if cmd == 'relay_get' and status == 'OK':
                for pair in details.split(','):
                    if '=' in pair:
                        key, val = pair.split('=', 1)
                        if key == 'ozone_gen':
                            self.state.ozone_gen = val == '1'
                        elif key == 'o2_conc':
                            self.state.o2_conc = val == '1'
            
            elif cmd == 'status' and status == 'OK':
                for pair in details.split(','):
                    if '=' in pair:
                        key, val = pair.split('=', 1)
                        if key == 'heap':
                            self.state.heap_free = int(val)
                        elif key == 'errors':
                            self.state.error_count = int(val)
            
            elif cmd == 'backup_list' and status == 'OK':
                self.state.backup_files = []
                if details and details != 'no_files':
                    for entry in details.split(';'):
                        file_parts = entry.split(':')
                        if len(file_parts) >= 1:
                            self.state.backup_files.append({
                                'filename': file_parts[0],
                                'size': int(file_parts[1]) if len(file_parts) > 1 else 0,
                                'samples': int(file_parts[2]) if len(file_parts) > 2 else 0
                            })
            
            elif cmd == 'backup_status' and status == 'OK':
                for pair in details.split(','):
                    if '=' in pair:
                        key, val = pair.split('=', 1)
                        if key == 'total':
                            self.state.storage_total = int(val)
                        elif key == 'used':
                            self.state.storage_used = int(val)
                        elif key == 'free':
                            self.state.storage_free = int(val)
                        elif key == 'recording':
                            self.state.backup_recording = val == '1'
            
            # 106-H responses
            elif cmd == '106h_avg_get' and status == 'OK':
                for pair in details.split(','):
                    if '=' in pair:
                        key, val = pair.split('=', 1)
                        if key == 'avg':
                            self.state.m106h_avg_time = val
                        elif key == 'option':
                            self.state.m106h_avg_option = int(val)
            
            elif cmd == '106h_avg' and status == 'OK':
                if 'avg=' in details:
                    self.state.m106h_avg_time = details.split('=')[1]
            
            elif cmd == '106h_log_status' and status == 'OK':
                if 'logging=' in details:
                    self.state.m106h_logging = details.split('=')[1] == 'active'
            
            elif cmd in ('106h_log_start', '106h_log_stop') and status == 'OK':
                self.state.m106h_logging = cmd == '106h_log_start'
    
    def _process_file_notification(self, line: str):
        parts = line.split(',')
        if len(parts) >= 2:
            action = parts[1].upper()
            if action == 'START':
                self.state.backup_recording = True
            elif action == 'STOP':
                self.state.backup_recording = False
    
    def _process_log_data(self, line: str):
        """Process 106-H log data transmission"""
        parts = line.split(',', 2)
        if len(parts) >= 2:
            action = parts[1].upper()
            if action == 'START':
                self.m106h_log_receiving = True
                self.m106h_log_data = []
                self.m106h_log_complete = False
                print("[106H LOG] Receiving log data...")
            elif action == 'DATA' and len(parts) > 2:
                self.m106h_log_data.append(parts[2])
            elif action == 'END':
                self.m106h_log_receiving = False
                self.m106h_log_complete = True
                print(f"[106H LOG] Received {len(self.m106h_log_data)} lines")
    
    def save_106h_log(self, filename: str) -> bool:
        """Save received 106-H log data to file"""
        if not self.m106h_log_data:
            return False
        
        try:
            with open(filename, 'w') as f:
                # Write header
                f.write("log_num,ozone_pct,temperature_c,pressure_mbar,ref_v,sample_v,date,time\n")
                # Write data
                for line in self.m106h_log_data:
                    f.write(line + '\n')
            print(f"[106H LOG] Saved to {filename}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to save log: {e}")
            return False
    
    def get_106h_log_csv(self) -> str:
        """Get 106-H log data as CSV string"""
        if not self.m106h_log_data:
            return ""
        
        lines = ["log_num,ozone_pct,temperature_c,pressure_mbar,ref_v,sample_v,date,time"]
        lines.extend(self.m106h_log_data)
        return '\n'.join(lines)
    
    def _run(self):
        buffer = ""
        
        while self.running:
            # Accept new connection if needed
            if not self.state.connected:
                try:
                    self.client_socket, addr = self.server_socket.accept()
                    self.client_socket.settimeout(0.5)
                    self.state.connected = True
                    self.state.client_ip = f"{addr[0]}:{addr[1]}"
                    self.state.last_data_time = datetime.now()
                    buffer = ""  # Clear buffer on new connection
                    print(f"[CONNECTED] ESP32 at {self.state.client_ip}")
                    time.sleep(0.3)
                    self.send_command("CMD,relay_get")
                    self.send_command("CMD,status")
                    self.send_command("CMD,106h_avg_get")
                    self.send_command("CMD,106h_log_status")
                except socket.timeout:
                    continue
                except Exception as e:
                    continue
            
            # Check for stale connection (no data for 30 seconds)
            if self.state.connected and self.state.last_data_time:
                if (datetime.now() - self.state.last_data_time).seconds > 30:
                    print("[TIMEOUT] No data for 30s, assuming disconnected")
                    self._handle_disconnect()
                    continue
            
            # Receive data
            try:
                data = self.client_socket.recv(4096)
                if not data:
                    self._handle_disconnect()
                    continue
                
                buffer += data.decode('utf-8', errors='replace')
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self._process_line(line)
                    
            except socket.timeout:
                continue
            except Exception as e:
                self._handle_disconnect()
    
    def get_samples_df(self, minutes: int = 10) -> pd.DataFrame:
        cutoff = datetime.now() - timedelta(minutes=minutes)
        with self.data_lock:
            recent = [s for s in self.samples if s.timestamp > cutoff]
        
        if not recent:
            return pd.DataFrame()
        
        return pd.DataFrame([{
            'timestamp': s.timestamp,
            'O3 (%)': s.o3,
            'Temperature (°C)': s.temp,
            'Pressure (mbar)': s.press
        } for s in recent])
    
    def get_sample_count(self) -> int:
        with self.data_lock:
            return len(self.samples)
    
    def is_connected(self) -> bool:
        """Check actual connection state"""
        return self.state.connected and self.client_socket is not None
    
    def stop(self):
        self.running = False
        self._handle_disconnect()
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        if self.stream_file:
            self.stream_file.close()
        self.stop_pc_recording()

# -----------------------------------------------------------------------------
# Get/Create Server Instance
# -----------------------------------------------------------------------------

def get_server(port: int = 5000) -> BlockSIServer:
    if 'server' not in st.session_state:
        server = BlockSIServer(port)
        server.start()
        st.session_state.server = server
    return st.session_state.server

# -----------------------------------------------------------------------------
# UI Components  
# -----------------------------------------------------------------------------

def render_sidebar():
    with st.sidebar:
        st.title("🧪 BlockSI Control")
        
        server = get_server()
        connected = server.is_connected()
        
        # Connection status
        st.subheader("Connection")
        if connected:
            st.success("● ESP32 Connected")
            st.caption(f"From: {server.state.client_ip}")
            if server.state.device_info:
                st.caption(f"Device: {server.state.device_info}")
            if server.state.last_data_time:
                age = (datetime.now() - server.state.last_data_time).seconds
                st.caption(f"Last data: {age}s ago")
        else:
            st.error("● Disconnected")
            st.caption(f"Listening on port {server.port}")
            st.info("Waiting for ESP32 to connect...")
        
        st.divider()
        
        # Device stats
        st.subheader("Device Status")
        col1, col2 = st.columns(2)
        with col1:
            st.metric("Samples", server.get_sample_count())
        with col2:
            st.metric("Errors", server.state.error_count)
        
        if server.state.heap_free > 0:
            st.metric("Free Heap", f"{server.state.heap_free:,} B")
        
        # Recording status
        if server.pc_recording:
            st.success(f"📝 Recording: {server.pc_record_filename}")
            st.caption(f"{server.pc_record_count} samples")
        
        if server.state.backup_recording:
            st.info("💾 ESP32 backup active")
        
        if server.state.m106h_logging:
            st.info("📊 106-H logging active")
        
        st.divider()
        
        # Display settings
        st.subheader("Display")
        time_range = st.selectbox("Time Range", [1, 5, 10, 30, 60], index=2, 
                                   format_func=lambda x: f"{x} min")
        st.session_state.time_range = time_range
        
        refresh_rate = st.selectbox("Refresh Rate", [1, 2, 5, 10], index=1,
                                     format_func=lambda x: f"{x} sec")
        st.session_state.refresh_rate = refresh_rate
        
        if st.button("🔄 Refresh Status", use_container_width=True, disabled=not connected):
            server.send_command("CMD,status")
            server.send_command("CMD,relay_get")
            server.send_command("CMD,106h_avg_get")
            server.send_command("CMD,106h_log_status")


def render_relay_controls():
    st.subheader("⚡ Relay Controls")
    
    server = get_server()
    connected = server.is_connected()
    
    if not connected:
        st.warning("ESP32 not connected - relay controls disabled")
        return
    
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.write("**Ozone Generator**")
        ozone_state = "🟢 ON" if server.state.ozone_gen else "🔴 OFF"
        st.write(f"Status: {ozone_state}")
        
        c1, c2 = st.columns(2)
        with c1:
            if st.button("ON", key="ozone_on", use_container_width=True):
                server.send_command("CMD,relay_set,ozone_gen,1")
                time.sleep(0.2)
                server.send_command("CMD,relay_get")
        with c2:
            if st.button("OFF", key="ozone_off", use_container_width=True):
                server.send_command("CMD,relay_set,ozone_gen,0")
                time.sleep(0.2)
                server.send_command("CMD,relay_get")
    
    with col2:
        st.write("**O₂ Concentrator**")
        o2_state = "🟢 ON" if server.state.o2_conc else "🔴 OFF"
        st.write(f"Status: {o2_state}")
        
        c1, c2 = st.columns(2)
        with c1:
            if st.button("ON", key="o2_on", use_container_width=True):
                server.send_command("CMD,relay_set,o2_conc,1")
                time.sleep(0.2)
                server.send_command("CMD,relay_get")
        with c2:
            if st.button("OFF", key="o2_off", use_container_width=True):
                server.send_command("CMD,relay_set,o2_conc,0")
                time.sleep(0.2)
                server.send_command("CMD,relay_get")
    
    with col3:
        st.write("**Emergency Stop**")
        st.write("")
        if st.button("🛑 ALL OFF", key="all_off", use_container_width=True, type="primary"):
            server.send_command("CMD,relay_all_off")
            time.sleep(0.2)
            server.send_command("CMD,relay_get")


def render_live_data():
    st.subheader("📊 Live Data")
    
    server = get_server()
    time_range = st.session_state.get('time_range', 10)
    
    df = server.get_samples_df(minutes=time_range)
    
    if df.empty:
        st.info("Waiting for data from ESP32...")
        return
    
    # Current values
    latest = df.iloc[-1]
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.metric("Ozone", f"{latest['O3 (%)']:.3f} %")
    with col2:
        st.metric("Temperature", f"{latest['Temperature (°C)']:.1f} °C")
    with col3:
        st.metric("Pressure", f"{latest['Pressure (mbar)']:.1f} mbar")
    
    # Charts
    fig = make_subplots(
        rows=3, cols=1,
        shared_xaxes=True,
        vertical_spacing=0.05,
        subplot_titles=('Ozone Concentration', 'Temperature', 'Pressure')
    )
    
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['O3 (%)'], 
                   mode='lines', name='O3', line=dict(color='#1f77b4', width=2)),
        row=1, col=1
    )
    
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['Temperature (°C)'], 
                   mode='lines', name='Temp', line=dict(color='#ff7f0e', width=2)),
        row=2, col=1
    )
    
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['Pressure (mbar)'], 
                   mode='lines', name='Press', line=dict(color='#2ca02c', width=2)),
        row=3, col=1
    )
    
    fig.update_layout(
        height=500,
        showlegend=False,
        margin=dict(l=60, r=20, t=30, b=20)
    )
    
    fig.update_yaxes(title_text="O3 (%)", row=1, col=1)
    fig.update_yaxes(title_text="°C", row=2, col=1)
    fig.update_yaxes(title_text="mbar", row=3, col=1)
    
    st.plotly_chart(fig, use_container_width=True)


def render_recording_tab():
    st.subheader("📝 PC Recording")
    
    server = get_server()
    
    # PC Recording controls
    st.write("**Record to PC**")
    st.caption("Saves data to CSV file on this computer")
    
    col1, col2, col3 = st.columns([2, 1, 1])
    with col1:
        seq_name = st.text_input("Recording Name", value="Sterilization", 
                                  key="pc_seq_name", label_visibility="collapsed",
                                  placeholder="Enter recording name...")
    with col2:
        start_disabled = server.pc_recording
        if st.button("▶️ Start", key="pc_start", use_container_width=True, disabled=start_disabled):
            filename = server.start_pc_recording(seq_name)
            st.session_state.last_record_action = f"Started: {filename}"
    with col3:
        stop_disabled = not server.pc_recording
        if st.button("⏹️ Stop", key="pc_stop", use_container_width=True, disabled=stop_disabled):
            filename, count = server.stop_pc_recording()
            st.session_state.last_record_action = f"Saved: {filename} ({count} samples)"
    
    # Show current recording status
    if server.pc_recording:
        st.success(f"🔴 Recording to: {server.pc_record_filename} ({server.pc_record_count} samples)")
    elif 'last_record_action' in st.session_state:
        st.info(st.session_state.last_record_action)
    
    st.divider()
    
    # List local CSV files
    st.write("**Local CSV Files**")
    csv_files = [f for f in os.listdir('.') if f.endswith('.csv')]
    csv_files.sort(reverse=True)
    
    if not csv_files:
        st.info("No CSV files in current directory")
    else:
        for f in csv_files[:10]:  # Show last 10
            size = os.path.getsize(f)
            col1, col2 = st.columns([4, 1])
            with col1:
                st.write(f"📄 {f}")
                st.caption(f"{size:,} bytes")
            with col2:
                with open(f, 'rb') as file:
                    st.download_button(
                        "📥",
                        file.read(),
                        file_name=f,
                        mime="text/csv",
                        key=f"dl_local_{f}"
                    )


def render_106h_tab():
    """Render 106-H Ozone Monitor control tab"""
    st.subheader("📊 106-H Ozone Monitor Control")
    
    server = get_server()
    connected = server.is_connected()
    
    if not connected:
        st.warning("ESP32 not connected - 106-H controls disabled")
        return
    
    # Averaging Time Control
    st.write("**Averaging Time**")
    st.caption("Change how often the 106-H outputs measurements. Shorter times give faster feedback during sequences.")
    
    avg_options = {
        1: "2 seconds",
        2: "10 seconds (default)",
        3: "1 minute",
        4: "5 minutes",
        5: "1 hour"
    }
    
    current_avg = server.state.m106h_avg_time
    st.write(f"Current setting: **{current_avg}**")
    
    col1, col2 = st.columns([3, 1])
    with col1:
        selected_avg = st.selectbox(
            "Select averaging time",
            options=list(avg_options.keys()),
            format_func=lambda x: avg_options[x],
            index=1,  # Default to 10 seconds
            key="avg_select",
            label_visibility="collapsed"
        )
    with col2:
        if st.button("Set", key="set_avg", use_container_width=True):
            server.send_command(f"CMD,106h_avg,{selected_avg}")
            time.sleep(0.5)
            server.send_command("CMD,106h_avg_get")
            st.success(f"Set to {avg_options[selected_avg]}")
    
    st.divider()
    
    # Internal Data Logging
    st.write("**106-H Internal Data Logging**")
    st.caption("The 106-H can log data to its internal memory. This is a backup independent of PC recording.")
    
    logging_status = "🟢 Active" if server.state.m106h_logging else "🔴 Inactive"
    st.write(f"Logging Status: {logging_status}")
    
    col1, col2, col3 = st.columns(3)
    with col1:
        if st.button("▶️ Start Logging", key="106h_log_start", use_container_width=True,
                     disabled=server.state.m106h_logging):
            server.send_command("CMD,106h_log_start")
            time.sleep(0.3)
            server.send_command("CMD,106h_log_status")
    with col2:
        if st.button("⏹️ Stop Logging", key="106h_log_stop", use_container_width=True,
                     disabled=not server.state.m106h_logging):
            server.send_command("CMD,106h_log_stop")
            time.sleep(0.3)
            server.send_command("CMD,106h_log_status")
    with col3:
        if st.button("🔄 Refresh Status", key="106h_refresh", use_container_width=True):
            server.send_command("CMD,106h_avg_get")
            server.send_command("CMD,106h_log_status")
    
    st.divider()
    
    # Info about log download
    st.info("💡 **To download logged data from 106-H:** Use the 2B Technologies Display & Graphing Software "
            "or a terminal emulator. The log transmit feature will be added in a future update.")


def render_backup_tab():
    st.subheader("💾 ESP32 Backup Storage")
    
    server = get_server()
    connected = server.is_connected()
    
    if not connected:
        st.warning("ESP32 not connected")
        return
    
    # Refresh button
    col1, col2 = st.columns([4, 1])
    with col2:
        if st.button("🔄 Refresh", key="refresh_backup"):
            server.send_command("CMD,backup_status")
            time.sleep(0.3)
            server.send_command("CMD,backup_list")
            time.sleep(0.3)
    
    # Storage metrics
    if server.state.storage_total > 0:
        col1, col2, col3 = st.columns(3)
        with col1:
            st.metric("Total", f"{server.state.storage_total // 1024} KB")
        with col2:
            st.metric("Used", f"{server.state.storage_used // 1024} KB")
        with col3:
            pct = (server.state.storage_free * 100) // server.state.storage_total if server.state.storage_total else 0
            st.metric("Free", f"{server.state.storage_free // 1024} KB ({pct}%)")
    
    st.divider()
    
    # ESP32 recording controls  
    st.write("**ESP32 Backup Recording**")
    st.caption("Records to ESP32 flash memory (backup in case PC disconnects)")
    
    col1, col2, col3 = st.columns([2, 1, 1])
    with col1:
        esp_seq_name = st.text_input("Backup Name", value="Sterilization",
                                      key="esp_seq_name", label_visibility="collapsed")
    with col2:
        if st.button("▶️ Start", key="esp_start", use_container_width=True):
            server.send_command(f"CMD,backup_start,{esp_seq_name}")
    with col3:
        if st.button("⏹️ Stop", key="esp_stop", use_container_width=True):
            server.send_command("CMD,backup_stop")
    
    if server.state.backup_recording:
        st.success("🔴 ESP32 backup recording active")
    
    st.divider()
    
    # File list
    st.write("**Backup Files on ESP32**")
    if not server.state.backup_files:
        st.info("No backup files found. Click 'Refresh' to check.")
    else:
        for f in server.state.backup_files:
            col1, col2 = st.columns([4, 1])
            with col1:
                st.write(f"📄 {f['filename']}")
                st.caption(f"{f['size']:,} bytes, ~{f['samples']} samples")
            with col2:
                if st.button("🗑️", key=f"del_esp_{f['filename']}", help="Delete from ESP32"):
                    server.send_command(f"CMD,backup_delete,{f['filename']}")
                    time.sleep(0.3)
                    server.send_command("CMD,backup_list")


def render_data_table():
    st.subheader("📋 Recent Data")
    
    server = get_server()
    df = server.get_samples_df(minutes=st.session_state.get('time_range', 10))
    
    if df.empty:
        st.info("No data yet")
        return
    
    st.dataframe(
        df.tail(100).iloc[::-1],
        use_container_width=True,
        hide_index=True
    )

# -----------------------------------------------------------------------------
# Main App
# -----------------------------------------------------------------------------

def main():
    st.set_page_config(
        page_title="BlockSI Control",
        page_icon="🧪",
        layout="wide",
        initial_sidebar_state="expanded"
    )
    
    # Parse port from command line
    port = 5000
    if '--port' in sys.argv:
        try:
            idx = sys.argv.index('--port')
            port = int(sys.argv[idx + 1])
        except:
            pass
    
    # Initialize server
    server = get_server(port)
    
    # Render UI
    render_sidebar()
    
    # Main content tabs
    tab1, tab2, tab3, tab4, tab5 = st.tabs([
        "📊 Monitor", 
        "📝 Recording", 
        "📊 106-H Control",
        "💾 ESP32 Backup", 
        "📋 Data"
    ])
    
    with tab1:
        render_relay_controls()
        st.divider()
        render_live_data()
    
    with tab2:
        render_recording_tab()
    
    with tab3:
        render_106h_tab()
    
    with tab4:
        render_backup_tab()
    
    with tab5:
        render_data_table()
    
    # Auto-refresh
    refresh_rate = st.session_state.get('refresh_rate', 2)
    time.sleep(refresh_rate)
    st.rerun()


if __name__ == '__main__':
    main()

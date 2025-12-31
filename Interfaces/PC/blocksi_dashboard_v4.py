#!/usr/bin/env python3
"""
BlockSI Control Dashboard v2.0

Extended features:
- O3 generator power control with predicted output
- Room O3 safety monitoring
- Vessel temperature display
- Flow rate setting
- Enhanced data visualization

Author: BlockSI Development Team
"""

import streamlit as st
import socket
import threading
import queue
import time
import pandas as pd
import numpy as np
from datetime import datetime
from collections import deque
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os

# =============================================================================
# Configuration
# =============================================================================

DEFAULT_ESP32_IP = "192.168.1.100"
DEFAULT_ESP32_PORT = 5000
MAX_DATA_POINTS = 500
RECONNECT_INTERVAL = 5.0

# Safety thresholds for room O3 (ppm)
O3_WARNING_PPM = 0.1
O3_DANGER_PPM = 0.3
O3_CRITICAL_PPM = 1.0

# =============================================================================
# O3 Prediction Model (from empirical calibration data)
# =============================================================================

def predict_o3_output(flow_lpm: float, power_pct: int) -> float:
    """
    Predict O3 output based on flow rate and power level.
    
    Empirical model:
    - At max power: O3 = 1.78/F + 1.40 (hyperbolic flow relationship)
    - Power scaling: three-region model (onset, linear, saturation)
    
    Args:
        flow_lpm: Flow rate in LPM
        power_pct: Power level 0-100%
    
    Returns:
        Predicted O3 concentration in %vol
    """
    if flow_lpm <= 0 or power_pct == 0:
        return 0.0
    
    # Max power O3 at this flow rate (hyperbolic model from calibration)
    o3_max = 1.78 / flow_lpm + 1.40
    
    # Power scaling (empirical three-region model)
    if power_pct < 20:
        # Below corona onset - minimal output
        power_factor = power_pct / 100.0
    elif power_pct <= 75:
        # Active range - approximately linear
        # Maps 20-75% power to 30-100% of max output
        power_factor = 0.30 + (power_pct - 20.0) / 55.0 * 0.70
    else:
        # Saturation above 75%
        power_factor = 1.0
    
    return o3_max * power_factor

def get_flow_o3_curve(power_pct: int, flow_range=(0.5, 5.0)) -> tuple:
    """Generate O3 vs flow rate curve for given power level."""
    flows = np.linspace(flow_range[0], flow_range[1], 50)
    o3_values = [predict_o3_output(f, power_pct) for f in flows]
    return flows, o3_values

def get_power_o3_curve(flow_lpm: float, power_range=(0, 100)) -> tuple:
    """Generate O3 vs power curve for given flow rate."""
    powers = np.arange(power_range[0], power_range[1] + 1)
    o3_values = [predict_o3_output(flow_lpm, p) for p in powers]
    return powers, o3_values

# =============================================================================
# Network Communication
# =============================================================================

class ESP32Client:
    """TCP client for ESP32 communication."""
    
    def __init__(self):
        self.socket = None
        self.connected = False
        self.receive_queue = queue.Queue()
        self.send_queue = queue.Queue()
        self.data_buffer = deque(maxlen=MAX_DATA_POINTS)
        self.receive_thread = None
        self.send_thread = None
        self.running = False
        self.lock = threading.Lock()
        
        # Extended data columns
        self.columns = [
            'timestamp', 'o3_pct', 'temp_c', 'pressure_mbar', 
            'sample_v', 'ref_v', 'room_o3_ppm', 'vessel_temp_c',
            'power_pct', 'flow_lpm'
        ]
        
        # Current state
        self.current_power = 0
        self.current_flow = 5.0
        self.room_o3_alarm = False
        
    def connect(self, ip: str, port: int) -> bool:
        """Connect to ESP32."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((ip, port))
            self.socket.settimeout(0.5)
            self.connected = True
            self.running = True
            
            # Start threads
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
            self.receive_thread.start()
            self.send_thread.start()
            
            return True
        except Exception as e:
            st.error(f"Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from ESP32."""
        self.running = False
        self.connected = False
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
    
    def _receive_loop(self):
        """Background thread for receiving data."""
        buffer = ""
        while self.running:
            try:
                data = self.socket.recv(4096).decode('utf-8', errors='ignore')
                if not data:
                    self.connected = False
                    break
                
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self._process_line(line)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.connected = False
                break
    
    def _send_loop(self):
        """Background thread for sending commands."""
        while self.running:
            try:
                cmd = self.send_queue.get(timeout=0.5)
                if self.connected and self.socket:
                    self.socket.sendall((cmd + '\n').encode('utf-8'))
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    self.connected = False
                break
    
    def _process_line(self, line: str):
        """Process received line."""
        if line.startswith('DATA,'):
            self._parse_data(line)
        elif line.startswith('RESP,'):
            self.receive_queue.put(('response', line[5:]))
        elif line.startswith('FILE,'):
            self.receive_queue.put(('file_header', line))
        elif line == 'END_FILE':
            self.receive_queue.put(('file_end', None))
        else:
            self.receive_queue.put(('message', line))
    
    def _parse_data(self, line: str):
        """Parse extended DATA message."""
        try:
            parts = line.split(',')
            if len(parts) >= 11:
                # DATA,timestamp,o3_pct,temp,press,sample_v,ref_v,room_o3,vessel_temp,power,flow
                timestamp = parts[1]
                o3_pct = float(parts[2])
                temp_c = float(parts[3])
                pressure = float(parts[4])
                sample_v = float(parts[5])
                ref_v = float(parts[6])
                
                # Handle "-" for disconnected sensors
                room_o3 = None if parts[7] == '-' else float(parts[7])
                vessel_temp = None if parts[8] == '-' else float(parts[8])
                
                power = int(parts[9])
                flow = float(parts[10])
                
                with self.lock:
                    self.data_buffer.append({
                        'timestamp': timestamp,
                        'o3_pct': o3_pct,
                        'temp_c': temp_c,
                        'pressure_mbar': pressure,
                        'sample_v': sample_v,
                        'ref_v': ref_v,
                        'room_o3_ppm': room_o3,
                        'vessel_temp_c': vessel_temp,
                        'power_pct': power,
                        'flow_lpm': flow
                    })
                    self.current_power = power
                    self.current_flow = flow
                    
                    # Check room O3 alarm
                    if room_o3 is not None and room_o3 >= O3_WARNING_PPM:
                        self.room_o3_alarm = True
                    else:
                        self.room_o3_alarm = False
                        
        except Exception as e:
            pass  # Ignore parse errors
    
    def send_command(self, cmd: str, args: str = None) -> str:
        """Send command and wait for response."""
        if not self.connected:
            return None
        
        full_cmd = f"CMD,{cmd}"
        if args:
            full_cmd += f",{args}"
        
        # Clear old responses
        while not self.receive_queue.empty():
            try:
                self.receive_queue.get_nowait()
            except:
                break
        
        self.send_queue.put(full_cmd)
        
        # Wait for response
        try:
            msg_type, response = self.receive_queue.get(timeout=5.0)
            if msg_type == 'response':
                return response
        except queue.Empty:
            pass
        
        return None
    
    def get_dataframe(self) -> pd.DataFrame:
        """Get data as pandas DataFrame."""
        with self.lock:
            if not self.data_buffer:
                return pd.DataFrame(columns=self.columns)
            return pd.DataFrame(list(self.data_buffer))

# =============================================================================
# Streamlit UI
# =============================================================================

def main():
    st.set_page_config(
        page_title="BlockSI Control Dashboard",
        page_icon="🔬",
        layout="wide",
        initial_sidebar_state="expanded"
    )
    
    st.title("🔬 BlockSI Control Dashboard v2.0")
    
    # Initialize session state
    if 'client' not in st.session_state:
        st.session_state.client = ESP32Client()
    if 'recording' not in st.session_state:
        st.session_state.recording = False
    if 'record_data' not in st.session_state:
        st.session_state.record_data = []
    
    client = st.session_state.client
    
    # Sidebar - Connection and Settings
    with st.sidebar:
        st.header("⚙️ Connection")
        
        col1, col2 = st.columns([3, 1])
        with col1:
            esp_ip = st.text_input("ESP32 IP", value=DEFAULT_ESP32_IP)
        with col2:
            esp_port = st.number_input("Port", value=DEFAULT_ESP32_PORT, min_value=1, max_value=65535)
        
        col1, col2 = st.columns(2)
        with col1:
            if st.button("Connect", disabled=client.connected, use_container_width=True):
                if client.connect(esp_ip, int(esp_port)):
                    st.success("Connected!")
                    st.rerun()
        with col2:
            if st.button("Disconnect", disabled=not client.connected, use_container_width=True):
                client.disconnect()
                st.rerun()
        
        # Connection status
        if client.connected:
            st.success("🟢 Connected")
            
            # Room O3 alarm indicator
            if client.room_o3_alarm:
                st.error("⚠️ ROOM O3 ALARM!")
        else:
            st.error("🔴 Disconnected")
        
        st.divider()
        
        # System Status
        st.header("📊 System Status")
        if client.connected:
            if st.button("Refresh Status", use_container_width=True):
                resp = client.send_command("status")
                if resp:
                    st.code(resp)
        
        st.divider()
        
        # Recording controls
        st.header("💾 Recording")
        col1, col2 = st.columns(2)
        with col1:
            if st.button("▶️ Start", disabled=st.session_state.recording, use_container_width=True):
                if client.connected:
                    resp = client.send_command("recording_start")
                    if resp and "started" in resp:
                        st.session_state.recording = True
                        st.session_state.record_data = []
        with col2:
            if st.button("⏹️ Stop", disabled=not st.session_state.recording, use_container_width=True):
                if client.connected:
                    resp = client.send_command("recording_stop")
                    st.session_state.recording = False
        
        if st.session_state.recording:
            st.info("🔴 Recording...")
        
        # Save local CSV
        if st.button("📥 Save CSV", use_container_width=True):
            df = client.get_dataframe()
            if not df.empty:
                filename = f"blocksi_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                df.to_csv(filename, index=False)
                st.success(f"Saved: {filename}")
    
    # Main content - Tabs
    tab1, tab2, tab3, tab4, tab5 = st.tabs([
        "📈 Live Data", "🎛️ Power Control", "⚡ Relay Control", 
        "🌡️ Sensors", "📁 Backup Files"
    ])
    
    # ==========================================================================
    # Tab 1: Live Data
    # ==========================================================================
    with tab1:
        if client.connected:
            df = client.get_dataframe()
            
            if not df.empty:
                # Create multi-panel plot
                fig = make_subplots(
                    rows=3, cols=1,
                    shared_xaxes=True,
                    vertical_spacing=0.08,
                    subplot_titles=("O3 Concentration (%vol)", "Room O3 (ppm)", "Vessel Temperature (°C)"),
                    row_heights=[0.4, 0.3, 0.3]
                )
                
                # O3 concentration from 106-H
                fig.add_trace(
                    go.Scatter(
                        x=list(range(len(df))),
                        y=df['o3_pct'],
                        mode='lines',
                        name='O3 %vol',
                        line=dict(color='blue', width=2)
                    ),
                    row=1, col=1
                )
                
                # Room O3 (with safety thresholds)
                room_o3 = df['room_o3_ppm'].dropna()
                if not room_o3.empty:
                    fig.add_trace(
                        go.Scatter(
                            x=room_o3.index.tolist(),
                            y=room_o3,
                            mode='lines',
                            name='Room O3',
                            line=dict(color='orange', width=2)
                        ),
                        row=2, col=1
                    )
                    # Warning threshold line
                    fig.add_hline(y=O3_WARNING_PPM, line_dash="dash", line_color="yellow",
                                  annotation_text="Warning", row=2, col=1)
                    fig.add_hline(y=O3_DANGER_PPM, line_dash="dash", line_color="red",
                                  annotation_text="Danger", row=2, col=1)
                
                # Vessel temperature
                vessel_temp = df['vessel_temp_c'].dropna()
                if not vessel_temp.empty:
                    fig.add_trace(
                        go.Scatter(
                            x=vessel_temp.index.tolist(),
                            y=vessel_temp,
                            mode='lines',
                            name='Vessel Temp',
                            line=dict(color='red', width=2)
                        ),
                        row=3, col=1
                    )
                
                fig.update_layout(
                    height=700,
                    showlegend=True,
                    legend=dict(orientation="h", yanchor="bottom", y=1.02),
                    margin=dict(l=50, r=20, t=80, b=50)
                )
                
                st.plotly_chart(fig, use_container_width=True)
                
                # Latest values
                st.subheader("Latest Readings")
                latest = df.iloc[-1]
                
                col1, col2, col3, col4, col5 = st.columns(5)
                
                with col1:
                    st.metric("O3 (%vol)", f"{latest['o3_pct']:.4f}")
                with col2:
                    room_o3_val = latest['room_o3_ppm']
                    if pd.notna(room_o3_val):
                        color = "normal" if room_o3_val < O3_WARNING_PPM else "inverse"
                        st.metric("Room O3 (ppm)", f"{room_o3_val:.4f}")
                    else:
                        st.metric("Room O3 (ppm)", "-")
                with col3:
                    vessel_t = latest['vessel_temp_c']
                    if pd.notna(vessel_t):
                        st.metric("Vessel (°C)", f"{vessel_t:.1f}")
                    else:
                        st.metric("Vessel (°C)", "-")
                with col4:
                    st.metric("Power (%)", f"{latest['power_pct']}")
                with col5:
                    st.metric("Flow (LPM)", f"{latest['flow_lpm']:.1f}")
                
                # Data table
                with st.expander("📋 Raw Data Table"):
                    st.dataframe(df.tail(50), use_container_width=True)
            else:
                st.info("Waiting for data...")
        else:
            st.warning("Connect to ESP32 to view live data")
    
    # ==========================================================================
    # Tab 2: Power Control
    # ==========================================================================
    with tab2:
        st.header("🎛️ O3 Generator Power Control")
        
        if client.connected:
            col1, col2 = st.columns([1, 1])
            
            with col1:
                st.subheader("Power Setting")
                
                # Current values
                current_power = client.current_power
                current_flow = client.current_flow
                
                # Flow rate setting
                new_flow = st.slider("O2 Flow Rate (LPM)", 0.5, 10.0, current_flow, 0.5,
                                     help="Set the O2 flow rate from the concentrator")
                
                if st.button("Set Flow Rate"):
                    resp = client.send_command("flow_set", str(new_flow))
                    if resp:
                        st.success(f"Flow rate set: {resp}")
                
                st.divider()
                
                # Power level setting
                new_power = st.slider("Power Level (%)", 0, 100, current_power, 1,
                                       help="O3 generator power (0-100%)")
                
                # Show predicted O3
                predicted_o3 = predict_o3_output(new_flow, new_power)
                st.info(f"**Predicted O3:** {predicted_o3:.2f} %vol at {new_flow:.1f} LPM")
                
                col_a, col_b = st.columns(2)
                with col_a:
                    if st.button("⚡ Set Power", use_container_width=True, type="primary"):
                        resp = client.send_command("power_set", str(new_power))
                        if resp:
                            st.success(f"Power set: {resp}")
                            st.rerun()
                with col_b:
                    if st.button("🛑 Power OFF", use_container_width=True):
                        resp = client.send_command("power_set", "0")
                        if resp:
                            st.warning("Power set to 0%")
                            st.rerun()
                
                # Quick presets
                st.subheader("Quick Presets")
                preset_cols = st.columns(4)
                presets = [25, 50, 75, 100]
                for i, pct in enumerate(presets):
                    with preset_cols[i]:
                        pred = predict_o3_output(new_flow, pct)
                        if st.button(f"{pct}%\n({pred:.1f}%)", use_container_width=True):
                            resp = client.send_command("power_set", str(pct))
                            if resp:
                                st.rerun()
            
            with col2:
                st.subheader("O3 Output Prediction")
                
                # Power vs O3 curve
                powers, o3_values = get_power_o3_curve(new_flow)
                
                fig = go.Figure()
                fig.add_trace(go.Scatter(
                    x=powers, y=o3_values,
                    mode='lines',
                    name=f'Predicted @ {new_flow:.1f} LPM',
                    line=dict(color='blue', width=2)
                ))
                
                # Mark current setting
                fig.add_trace(go.Scatter(
                    x=[new_power], y=[predicted_o3],
                    mode='markers',
                    name='Current Setting',
                    marker=dict(color='red', size=15, symbol='star')
                ))
                
                # Add regions
                fig.add_vrect(x0=0, x1=20, fillcolor="gray", opacity=0.2,
                              annotation_text="Sub-threshold", annotation_position="top left")
                fig.add_vrect(x0=75, x1=100, fillcolor="orange", opacity=0.1,
                              annotation_text="Saturation", annotation_position="top right")
                
                fig.update_layout(
                    title="Power vs O3 Output",
                    xaxis_title="Power (%)",
                    yaxis_title="O3 (%vol)",
                    height=400
                )
                
                st.plotly_chart(fig, use_container_width=True)
                
                # Flow vs O3 curve
                flows, o3_flow_values = get_flow_o3_curve(new_power)
                
                fig2 = go.Figure()
                fig2.add_trace(go.Scatter(
                    x=flows, y=o3_flow_values,
                    mode='lines',
                    name=f'Predicted @ {new_power}% power',
                    line=dict(color='green', width=2)
                ))
                
                # Mark current setting
                fig2.add_trace(go.Scatter(
                    x=[new_flow], y=[predicted_o3],
                    mode='markers',
                    name='Current Setting',
                    marker=dict(color='red', size=15, symbol='star')
                ))
                
                fig2.update_layout(
                    title="Flow Rate vs O3 Output",
                    xaxis_title="Flow Rate (LPM)",
                    yaxis_title="O3 (%vol)",
                    height=400
                )
                
                st.plotly_chart(fig2, use_container_width=True)
        else:
            st.warning("Connect to ESP32 to control power")
    
    # ==========================================================================
    # Tab 3: Relay Control
    # ==========================================================================
    with tab3:
        st.header("⚡ Relay Control")
        
        if client.connected:
            col1, col2 = st.columns(2)
            
            with col1:
                st.subheader("O3 Generator Relay")
                col_a, col_b = st.columns(2)
                with col_a:
                    if st.button("🟢 ON", key="o3_on", use_container_width=True):
                        resp = client.send_command("relay_set", "ozone_gen,1")
                        if resp:
                            st.success(resp)
                with col_b:
                    if st.button("🔴 OFF", key="o3_off", use_container_width=True):
                        resp = client.send_command("relay_set", "ozone_gen,0")
                        if resp:
                            st.info(resp)
            
            with col2:
                st.subheader("O2 Concentrator Relay")
                col_a, col_b = st.columns(2)
                with col_a:
                    if st.button("🟢 ON", key="o2_on", use_container_width=True):
                        resp = client.send_command("relay_set", "o2_conc,1")
                        if resp:
                            st.success(resp)
                with col_b:
                    if st.button("🔴 OFF", key="o2_off", use_container_width=True):
                        resp = client.send_command("relay_set", "o2_conc,0")
                        if resp:
                            st.info(resp)
            
            st.divider()
            
            if st.button("🔄 Get Relay Status"):
                resp = client.send_command("relay_get")
                if resp:
                    st.code(resp)
        else:
            st.warning("Connect to ESP32 to control relays")
    
    # ==========================================================================
    # Tab 4: Sensors
    # ==========================================================================
    with tab4:
        st.header("🌡️ Sensor Status")
        
        if client.connected:
            col1, col2 = st.columns(2)
            
            with col1:
                st.subheader("106-H Ozone Monitor")
                if st.button("Get 106-H Status"):
                    resp = client.send_command("106h_status")
                    if resp:
                        st.code(resp)
                
                st.divider()
                
                st.subheader("Averaging Time")
                avg_options = {
                    "2 seconds": 1,
                    "10 seconds": 2,
                    "1 minute": 3,
                    "5 minutes": 4,
                    "1 hour": 5
                }
                selected_avg = st.selectbox("Select averaging", list(avg_options.keys()))
                
                if st.button("Set Averaging"):
                    resp = client.send_command("106h_avg_set", str(avg_options[selected_avg]))
                    if resp:
                        st.success(resp)
            
            with col2:
                st.subheader("Secondary Sensors")
                
                if st.button("Get Sensor Status"):
                    resp = client.send_command("sensors_get")
                    if resp:
                        parts = resp.split(',')
                        for part in parts:
                            st.code(part)
                
                st.divider()
                
                st.subheader("Room O3 Alarm")
                if st.button("Check Alarm Status"):
                    resp = client.send_command("room_o3_alarm")
                    if resp:
                        if "active" in resp:
                            st.error(f"⚠️ ALARM: {resp}")
                        else:
                            st.success(f"✅ {resp}")
                
                st.divider()
                
                st.subheader("I2C Bus")
                if st.button("Scan I2C"):
                    resp = client.send_command("i2c_scan")
                    st.info("Check ESP32 serial output for scan results")
        else:
            st.warning("Connect to ESP32 to view sensor status")
    
    # ==========================================================================
    # Tab 5: Backup Files
    # ==========================================================================
    with tab5:
        st.header("📁 Backup Files (ESP32 SPIFFS)")
        
        if client.connected:
            if st.button("🔄 Refresh File List"):
                resp = client.send_command("backup_list")
                if resp:
                    st.session_state.backup_files = resp
            
            if 'backup_files' in st.session_state:
                st.code(st.session_state.backup_files)
                
                # Parse file list
                if "files=" in st.session_state.backup_files:
                    parts = st.session_state.backup_files.split(',')
                    if len(parts) > 1:
                        for part in parts[1:]:
                            if ':' in part:
                                filename, size = part.split(':')
                                col1, col2, col3 = st.columns([3, 1, 1])
                                with col1:
                                    st.text(f"{filename} ({size} bytes)")
                                with col2:
                                    if st.button("📥", key=f"dl_{filename}"):
                                        resp = client.send_command("backup_get", filename)
                                        st.info("File transfer initiated")
                                with col3:
                                    if st.button("🗑️", key=f"del_{filename}"):
                                        resp = client.send_command("backup_delete", filename)
                                        if resp:
                                            st.success(f"Deleted: {resp}")
        else:
            st.warning("Connect to ESP32 to manage backup files")
    
    # Auto-refresh
    if client.connected:
        time.sleep(0.5)
        st.rerun()

if __name__ == "__main__":
    main()

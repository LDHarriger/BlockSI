#!/usr/bin/env python3
"""
BlockSI Dashboard v9 - Unified State & Improved UI

Key improvements from v8:
- Uses ESP32 timestamp (properly synced on connect)
- Unified system state model
- Slider syncs with presets
- Toggle buttons for relays with color feedback
- Power curve visualization
- Calibration interface

DATA format from ESP32 (v2):
  DATA,esp_timestamp_ms,vessel_o3_pct,temp_c,pressure_mbar,sample_v,ref_v,
       day,month,year,hour,minute,second,room_o3_ppm,vessel_temp_c,
       power_target_pct,power_actual_pct,wiper_voltage

Usage:
    streamlit run blocksi_dashboard_v9.py -- --port 5000
"""

import streamlit as st
import socket
import threading
import queue
import time
import pandas as pd
import numpy as np
from datetime import datetime, timedelta
from collections import deque
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import argparse
import os

# =============================================================================
# Page Config
# =============================================================================
st.set_page_config(
    page_title="BlockSI Control",
    page_icon="🍄",
    layout="wide",
    initial_sidebar_state="expanded"
)

# =============================================================================
# Configuration
# =============================================================================
DEFAULT_PORT = 5000
MAX_DATA_POINTS = 500
# CSV file path - save to Data folder with timestamp
DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Data')
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)
CSV_FILE = os.path.join(DATA_DIR, f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_Stream.csv")
MOTOR_POT_OHMS = 5000
POWER_MISMATCH_THRESHOLD = 5.0  # % error threshold

# Power model coefficients (from characterization)
# O3_max = 1.78/F + 1.40 where F = flow rate in LPM (default 5 LPM)
POWER_MODEL_A = 1.78
POWER_MODEL_B = 1.40
DEFAULT_FLOW_LPM = 5.0

# =============================================================================
# Unified System State
# =============================================================================
class SystemState:
    """
    Unified system state - single source of truth.
    Updated from telemetry and commands.
    """
    def __init__(self):
        # Power
        self.power_target_pct = 0
        self.power_actual_pct = 0.0
        self.wiper_voltage = 0.0
        self.power_error = False
        
        # Relays
        self.relay_o3_gen = False
        self.relay_o2_conc = False
        self.relay_air_comp = False
        
        # Sensors
        self.vessel_o3_pct = 0.0
        self.room_o3_ppm = 0.0
        self.vessel_temp_c = -999.0
        self.cell_temp_c = 0.0
        self.pressure_mbar = 0.0
        
        # Timing
        self.esp_time_offset_ms = 0  # PC_time - ESP_time
        self.time_synced = False
        self.last_update = None
        
        # Calibration
        self.calibration_active = False
        self.calibration_data = []  # List of (power_pct, o3_pct) tuples
        
        # Connection
        self.connected = False


@st.cache_resource
def get_system_state():
    """Get singleton system state."""
    return SystemState()


# =============================================================================
# Network State (thread-safe)
# =============================================================================
class NetworkState:
    """Thread-safe network state."""
    def __init__(self):
        self.thread = None
        self.running = False
        self.server_socket = None
        self.client_socket = None
        self.lock = threading.Lock()
        self.data_queue = queue.Queue(maxsize=1000)
        self.response_queue = queue.Queue(maxsize=50)
        self.port = None


@st.cache_resource
def get_network_state():
    return NetworkState()


_net = get_network_state()
_sys = get_system_state()


# =============================================================================
# Session State
# =============================================================================
if 'data_buffer' not in st.session_state:
    st.session_state.data_buffer = deque(maxlen=MAX_DATA_POINTS)

if 'debug_logs' not in st.session_state:
    st.session_state.debug_logs = deque(maxlen=100)

if 'last_response' not in st.session_state:
    st.session_state.last_response = ""


def log_debug(msg: str):
    ts = datetime.now().strftime('%H:%M:%S')
    st.session_state.debug_logs.appendleft(f"{ts} {msg}")


# =============================================================================
# Power Model
# =============================================================================
def predict_o3_from_power(power_pct: float, flow_lpm: float = DEFAULT_FLOW_LPM) -> float:
    """
    Predict O3 output based on power level.
    Uses piecewise model:
    - 0-20%: minimal output (threshold zone)
    - 20-75%: linear ramp
    - 75-100%: saturation (diminishing returns)
    """
    if power_pct <= 0 or flow_lpm <= 0:
        return 0.0
    
    # Max O3 at this flow rate
    o3_max = POWER_MODEL_A / flow_lpm + POWER_MODEL_B
    
    # Power zone scaling
    if power_pct < 20:
        scaling = (power_pct / 20) * 0.1
    elif power_pct <= 75:
        scaling = 0.1 + (power_pct - 20) / 55 * 0.9
    else:
        scaling = 1.0  # Saturated
    
    return o3_max * scaling


def generate_power_curve(flow_lpm: float = DEFAULT_FLOW_LPM):
    """Generate power vs O3 curve data."""
    power_values = np.linspace(0, 100, 101)
    o3_values = [predict_o3_from_power(p, flow_lpm) for p in power_values]
    return power_values, o3_values


# =============================================================================
# Data Parsing
# =============================================================================
def parse_data_line(line: str) -> dict:
    """
    Parse DATA line from ESP32 using ESP32 timestamp.
    
    Format: DATA,esp_ts_ms,vessel_o3_pct,temp_c,press,sample_v,ref_v,d,m,y,h,min,sec,
                 room_o3_ppm,vessel_temp_c,power_target,power_actual,wiper_v
    """
    parts = line.split(',')
    if len(parts) < 5:
        return None
    
    def safe_float(idx, default=0.0):
        try:
            return float(parts[idx]) if idx < len(parts) and parts[idx] else default
        except (ValueError, IndexError):
            return default
    
    def safe_int(idx, default=0):
        try:
            return int(parts[idx]) if idx < len(parts) and parts[idx] else default
        except (ValueError, IndexError):
            return default
    
    esp_ts_ms = safe_int(1)
    
    # Convert ESP32 timestamp to PC time
    if _sys.time_synced:
        pc_ts_ms = esp_ts_ms + _sys.esp_time_offset_ms
        timestamp = datetime.fromtimestamp(pc_ts_ms / 1000.0)
    else:
        # Fallback to PC time if not synced
        timestamp = datetime.now()
    
    return {
        'timestamp': timestamp,
        'esp_ts_ms': esp_ts_ms,
        'vessel_o3_pct': safe_float(2),
        'cell_temp_c': safe_float(3),
        'pressure_mbar': safe_float(4),
        'sample_v': safe_float(5),
        'ref_v': safe_float(6),
        'day': safe_int(7),
        'month': safe_int(8),
        'year': safe_int(9),
        'hour': safe_int(10),
        'minute': safe_int(11),
        'second': safe_int(12),
        'room_o3_ppm': safe_float(13),
        'vessel_temp_c': safe_float(14, -999.0),
        'power_target_pct': safe_int(15),
        'power_actual_pct': safe_float(16),
        'wiper_voltage': safe_float(17),
    }


def update_system_state(sample: dict):
    """Update unified system state from telemetry."""
    _sys.power_target_pct = sample.get('power_target_pct', 0)
    _sys.power_actual_pct = sample.get('power_actual_pct', 0.0)
    _sys.wiper_voltage = sample.get('wiper_voltage', 0.0)
    _sys.vessel_o3_pct = sample.get('vessel_o3_pct', 0.0)
    _sys.room_o3_ppm = sample.get('room_o3_ppm', 0.0)
    _sys.vessel_temp_c = sample.get('vessel_temp_c', -999.0)
    _sys.cell_temp_c = sample.get('cell_temp_c', 0.0)
    _sys.pressure_mbar = sample.get('pressure_mbar', 0.0)
    _sys.last_update = sample.get('timestamp')
    
    # Check for power mismatch
    error = abs(_sys.power_target_pct - _sys.power_actual_pct)
    _sys.power_error = error > POWER_MISMATCH_THRESHOLD


# =============================================================================
# Network Functions
# =============================================================================
def get_client_socket():
    with _net.lock:
        return _net.client_socket


def set_client_socket(sock):
    with _net.lock:
        _net.client_socket = sock
        _sys.connected = sock is not None


def is_connected():
    return _sys.connected


def receiver_thread_func(port: int):
    """Background receiver thread."""
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', port))
        server.listen(1)
        server.settimeout(1.0)
        _net.server_socket = server
        print(f"[Receiver] Server listening on port {port}")
    except Exception as e:
        print(f"[Receiver] Failed to start: {e}")
        _net.running = False
        return
    
    rx_buffer = ""
    
    while _net.running:
        if get_client_socket() is None:
            try:
                client, addr = server.accept()
                client.settimeout(0.5)
                set_client_socket(client)
                print(f"[Receiver] Connected from {addr}")
                
                # Send time sync on connect
                # Offset will be calculated when we receive the response
                pc_time_ms = int(time.time() * 1000)
                sync_cmd = f"CMD,time_sync,{pc_time_ms}\n"
                try:
                    client.sendall(sync_cmd.encode())
                    print(f"[Receiver] Sent time sync request: PC={pc_time_ms}ms")
                except:
                    pass
                    
            except socket.timeout:
                continue
            except Exception as e:
                continue
        
        client = get_client_socket()
        if client is None:
            continue
        
        try:
            data = client.recv(2048)
            if not data:
                print("[Receiver] Connection closed")
                try:
                    client.close()
                except:
                    pass
                set_client_socket(None)
                rx_buffer = ""
                continue
            
            rx_buffer += data.decode('utf-8', errors='ignore')
            
            while '\n' in rx_buffer:
                line, rx_buffer = rx_buffer.split('\n', 1)
                line = line.strip()
                if not line:
                    continue
                
                if line.startswith('DATA,'):
                    sample = parse_data_line(line)
                    if sample:
                        try:
                            _net.data_queue.put_nowait(sample)
                        except queue.Full:
                            pass
                elif line.startswith('RSP,'):
                    # Check for time_sync response to properly calculate offset
                    # Format: RSP,OK,time_sync,synced,esp=XXX,pc=XXX
                    if 'time_sync' in line and 'esp=' in line:
                        try:
                            # Parse esp uptime from response
                            import re
                            esp_match = re.search(r'esp=(\d+)', line)
                            pc_match = re.search(r'pc=(\d+)', line)
                            if esp_match and pc_match:
                                esp_uptime = int(esp_match.group(1))
                                pc_time = int(pc_match.group(1))
                                _sys.esp_time_offset_ms = pc_time - esp_uptime
                                _sys.time_synced = True
                                print(f"[Receiver] Time sync complete: offset={_sys.esp_time_offset_ms}ms")
                        except Exception as e:
                            print(f"[Receiver] Time sync parse error: {e}")
                    try:
                        _net.response_queue.put_nowait(line)
                    except queue.Full:
                        pass
        
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Receiver] Error: {e}")
            try:
                client.close()
            except:
                pass
            set_client_socket(None)
            rx_buffer = ""
    
    # Cleanup
    if get_client_socket():
        try:
            get_client_socket().close()
        except:
            pass
        set_client_socket(None)
    
    if _net.server_socket:
        try:
            _net.server_socket.close()
        except:
            pass
        _net.server_socket = None


def start_receiver(port: int):
    if _net.thread is not None and _net.thread.is_alive():
        if _net.port == port:
            return
    
    _net.running = True
    _net.port = port
    _net.thread = threading.Thread(target=receiver_thread_func, args=(port,), daemon=True)
    _net.thread.start()


def send_command(cmd: str, timeout: float = 2.0) -> str:
    """Send command and wait for response."""
    client = get_client_socket()
    if client is None:
        log_debug(f"send_command({cmd}): not connected")
        return None
    
    # Clear old responses
    while not _net.response_queue.empty():
        try:
            _net.response_queue.get_nowait()
        except queue.Empty:
            break
    
    try:
        full_cmd = f"CMD,{cmd}\n"
        with _net.lock:
            client.sendall(full_cmd.encode())
        log_debug(f"Sent: {cmd}")
        
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                response = _net.response_queue.get(timeout=0.1)
                st.session_state.last_response = response
                log_debug(f"Response: {response}")
                return response
            except queue.Empty:
                continue
        
        log_debug(f"Timeout: {cmd}")
        return None
    except Exception as e:
        log_debug(f"Error: {e}")
        return None


# =============================================================================
# Command Helpers
# =============================================================================
def set_power(power_pct: int):
    """Set power and update system state.
    
    Note: We don't modify session_state['power_slider'] here because
    Streamlit doesn't allow modifying widget state after instantiation.
    The slider will sync via the value= parameter on next rerun.
    """
    _sys.power_target_pct = power_pct
    resp = send_command(f"power_set,{power_pct}")
    return resp and 'OK' in resp


def set_relay(relay_name: str, state: bool):
    """Set relay and update system state."""
    state_int = 1 if state else 0
    resp = send_command(f"relay_set,{relay_name},{state_int}")
    if resp and 'OK' in resp:
        if relay_name == 'ozone_gen':
            _sys.relay_o3_gen = state
        elif relay_name == 'o2_conc':
            _sys.relay_o2_conc = state
        elif relay_name == 'air_comp':
            _sys.relay_air_comp = state
        return True
    return False


# =============================================================================
# Data Processing
# =============================================================================
def process_queued_data():
    """Pull data from queue into buffer."""
    count = 0
    while not _net.data_queue.empty() and count < 50:
        try:
            sample = _net.data_queue.get_nowait()
            st.session_state.data_buffer.append(sample)
            update_system_state(sample)
            save_sample_to_csv(sample)
            count += 1
        except queue.Empty:
            break


def save_sample_to_csv(sample: dict):
    try:
        file_exists = os.path.exists(CSV_FILE)
        with open(CSV_FILE, 'a') as f:
            if not file_exists:
                f.write("timestamp,esp_ts_ms,vessel_o3_pct,cell_temp_c,pressure_mbar,"
                        "room_o3_ppm,vessel_temp_c,power_target,power_actual,wiper_v\n")
            f.write(f"{sample['timestamp']},{sample['esp_ts_ms']},{sample['vessel_o3_pct']},"
                    f"{sample['cell_temp_c']},{sample['pressure_mbar']},{sample['room_o3_ppm']},"
                    f"{sample['vessel_temp_c']},{sample.get('power_target_pct',0)},"
                    f"{sample.get('power_actual_pct',0)},{sample.get('wiper_voltage',0)}\n")
    except Exception:
        pass


# =============================================================================
# UI Components
# =============================================================================
def render_sidebar():
    """Render sidebar with connection and quick controls."""
    st.sidebar.title("🍄 BlockSI")
    
    # Connection status
    if is_connected():
        st.sidebar.success("🟢 Connected")
    else:
        st.sidebar.error("🔴 Disconnected")
    
    st.sidebar.divider()
    
    # Quick relay toggles with color
    st.sidebar.subheader("Quick Controls")
    
    col1, col2, col3 = st.sidebar.columns(3)
    with col1:
        o3_label = "🟢 O₃" if _sys.relay_o3_gen else "🔴 O₃"
        if st.button(o3_label, key="sb_o3_toggle", help="Ozone Generator"):
            set_relay('ozone_gen', not _sys.relay_o3_gen)
            st.rerun()
    
    with col2:
        o2_label = "🟢 O₂" if _sys.relay_o2_conc else "🔴 O₂"
        if st.button(o2_label, key="sb_o2_toggle", help="O2 Concentrator"):
            set_relay('o2_conc', not _sys.relay_o2_conc)
            st.rerun()
    
    with col3:
        air_label = "🟢 Air" if _sys.relay_air_comp else "🔴 Air"
        if st.button(air_label, key="sb_air_toggle", help="Air Compressor"):
            set_relay('air_comp', not _sys.relay_air_comp)
            st.rerun()
    
    st.sidebar.divider()
    
    # Live metrics
    st.sidebar.metric("Vessel O₃", f"{_sys.vessel_o3_pct:.3f} %vol")
    st.sidebar.metric("Room O₃", f"{_sys.room_o3_ppm:.3f} ppm")
    if _sys.vessel_temp_c > -900:
        st.sidebar.metric("Vessel Temp", f"{_sys.vessel_temp_c:.1f} °C")
    st.sidebar.metric("Power", f"{_sys.power_actual_pct:.1f}%")


def render_power_tab():
    """Render power control tab with new UI design."""
    st.header("⚡ Power Control")
    
    # Top row: Target, Actual, O3
    col1, col2, col3 = st.columns(3)
    with col1:
        st.metric("Target Power", f"{_sys.power_target_pct}%")
    with col2:
        delta = _sys.power_actual_pct - _sys.power_target_pct
        st.metric("Actual Power", f"{_sys.power_actual_pct:.1f}%",
                  delta=f"{delta:+.1f}%" if abs(delta) > 0.5 else None,
                  delta_color="inverse" if abs(delta) > POWER_MISMATCH_THRESHOLD else "normal")
    with col3:
        st.metric("Vessel O₃", f"{_sys.vessel_o3_pct:.4f} %vol")
    
    # Power mismatch warning
    if _sys.power_error:
        st.error(f"⚠️ Power mismatch! Target: {_sys.power_target_pct}%, "
                f"Actual: {_sys.power_actual_pct:.1f}%")
    
    st.divider()
    
    # Slider with synced value
    st.subheader("Power Setting")
    
    # Use system state for slider default
    power_pct = st.slider(
        "Power (%)",
        min_value=0,
        max_value=100,
        value=_sys.power_target_pct,
        step=1,
        key="power_slider",
        help="Drag to set power level"
    )
    
    col1, col2 = st.columns([3, 1])
    with col1:
        st.write(f"**Selected: {power_pct}%** ({power_pct/100*MOTOR_POT_OHMS:.0f}Ω)")
    with col2:
        if st.button("Apply", type="primary", key="apply_power"):
            if set_power(power_pct):
                st.success(f"Power set to {power_pct}%")
            else:
                st.error("Failed to set power")
    
    # Quick presets (10% intervals)
    st.write("**Quick Set:**")
    cols = st.columns(11)
    for i, preset in enumerate(range(0, 110, 10)):
        with cols[i]:
            # Highlight current target
            btn_type = "primary" if preset == _sys.power_target_pct else "secondary"
            if st.button(f"{preset}", key=f"preset_{preset}"):
                if set_power(preset):
                    st.rerun()
    
    # Emergency stop
    st.divider()
    if st.button("🛑 EMERGENCY STOP", type="primary"):
        set_power(0)
        st.rerun()
    
    # Power curve visualization
    st.divider()
    st.subheader("Power vs O₃ Curve")
    
    power_vals, o3_vals = generate_power_curve()
    
    fig = go.Figure()
    
    # Model curve
    fig.add_trace(go.Scatter(
        x=power_vals, y=o3_vals,
        mode='lines',
        name='Predicted O₃',
        line=dict(color='blue', width=2)
    ))
    
    # Target point (black circle)
    target_o3 = predict_o3_from_power(_sys.power_target_pct)
    fig.add_trace(go.Scatter(
        x=[_sys.power_target_pct], y=[target_o3],
        mode='markers',
        name=f'Target ({_sys.power_target_pct}%)',
        marker=dict(color='black', size=15, symbol='circle-open', line=dict(width=3))
    ))
    
    # Actual point (green dot)
    actual_o3 = _sys.vessel_o3_pct
    fig.add_trace(go.Scatter(
        x=[_sys.power_actual_pct], y=[actual_o3],
        mode='markers',
        name=f'Actual ({_sys.power_actual_pct:.1f}%)',
        marker=dict(color='green', size=12, symbol='circle')
    ))
    
    fig.update_layout(
        xaxis_title='Power (%)',
        yaxis_title='O₃ (%vol)',
        height=300,
        margin=dict(l=50, r=20, t=30, b=50),
        legend=dict(orientation='h', yanchor='bottom', y=1.02)
    )
    
    st.plotly_chart(fig)


def render_relay_tab():
    """Render relay control tab."""
    st.header("🔌 Relay Control")
    
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.subheader("O₃ Generator")
        if _sys.relay_o3_gen:
            st.success("🟢 **ON**")
            if st.button("Turn OFF", key="relay_o3_off"):
                set_relay('ozone_gen', False)
                st.rerun()
        else:
            st.error("🔴 **OFF**")
            if st.button("Turn ON", key="relay_o3_on"):
                set_relay('ozone_gen', True)
                st.rerun()
    
    with col2:
        st.subheader("O₂ Concentrator")
        if _sys.relay_o2_conc:
            st.success("🟢 **ON**")
            if st.button("Turn OFF", key="relay_o2_off"):
                set_relay('o2_conc', False)
                st.rerun()
        else:
            st.error("🔴 **OFF**")
            if st.button("Turn ON", key="relay_o2_on"):
                set_relay('o2_conc', True)
                st.rerun()
    
    with col3:
        st.subheader("Air Compressor")
        if _sys.relay_air_comp:
            st.success("🟢 **ON**")
            if st.button("Turn OFF", key="relay_air_off"):
                set_relay('air_comp', False)
                st.rerun()
        else:
            st.error("🔴 **OFF**")
            if st.button("Turn ON", key="relay_air_on"):
                set_relay('air_comp', True)
                st.rerun()


def render_telemetry_tab():
    """Render telemetry graphs."""
    st.header("📊 Telemetry")
    
    if len(st.session_state.data_buffer) == 0:
        st.info("Waiting for data...")
        return
    
    df = pd.DataFrame(list(st.session_state.data_buffer))
    
    # Metrics
    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.metric("Vessel O₃", f"{_sys.vessel_o3_pct:.4f} %vol")
    with col2:
        st.metric("Room O₃", f"{_sys.room_o3_ppm:.3f} ppm")
    with col3:
        if _sys.vessel_temp_c > -900:
            st.metric("Vessel Temp", f"{_sys.vessel_temp_c:.1f} °C")
        else:
            st.metric("Vessel Temp", "N/A")
    with col4:
        st.metric("Cell Temp", f"{_sys.cell_temp_c:.1f} °C")
    
    # Plots
    fig = make_subplots(
        rows=2, cols=1,
        subplot_titles=("Ozone Concentration", "Power & Temperature"),
        vertical_spacing=0.15,
        specs=[[{"secondary_y": True}], [{"secondary_y": True}]]
    )
    
    # Ozone
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['vessel_o3_pct'], 
                   name="Vessel O₃ (%vol)", line=dict(color='blue', width=2)),
        row=1, col=1, secondary_y=False
    )
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['room_o3_ppm'],
                   name="Room O₃ (ppm)", line=dict(color='green', width=2)),
        row=1, col=1, secondary_y=True
    )
    
    # Power & Temp
    if 'power_actual_pct' in df.columns:
        fig.add_trace(
            go.Scatter(x=df['timestamp'], y=df['power_actual_pct'],
                       name="Power (%)", line=dict(color='orange', width=2)),
            row=2, col=1, secondary_y=False
        )
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['cell_temp_c'],
                   name="Cell Temp (°C)", line=dict(color='red', width=2)),
        row=2, col=1, secondary_y=True
    )
    
    fig.update_yaxes(title_text="Vessel O₃ (%vol)", row=1, col=1, secondary_y=False)
    fig.update_yaxes(title_text="Room O₃ (ppm)", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="Power (%)", row=2, col=1, secondary_y=False)
    fig.update_yaxes(title_text="Temperature (°C)", row=2, col=1, secondary_y=True)
    
    fig.update_layout(height=500, showlegend=True)
    st.plotly_chart(fig)
    
    with st.expander("Raw Data"):
        st.dataframe(df.tail(30))


def render_calibration_tab():
    """Render calibration interface."""
    st.header("🔧 Calibration")
    
    st.markdown("""
    **Power Calibration Sweep**
    
    This routine sweeps power from 0-100% and back while recording O₃ output.
    Each step is 1% and waits for one 106-H sample (~2s) before advancing.
    
    **Requirements:**
    - O₃ generator connected directly to 106-H (minimal path volume)
    - O₂ concentrator running (for oxygen supply)
    - Stable flow rate
    """)
    
    if _sys.calibration_active:
        st.warning("Calibration in progress...")
        if st.button("Stop Calibration"):
            send_command("calibrate_stop")
            _sys.calibration_active = False
            st.rerun()
    else:
        col1, col2 = st.columns(2)
        with col1:
            if st.button("Start Calibration Sweep", type="primary"):
                resp = send_command("calibrate_start")
                if resp and 'OK' in resp:
                    _sys.calibration_active = True
                    st.success("Calibration started")
                else:
                    st.error("Failed to start calibration")
    
    # Show calibration data if available
    if len(_sys.calibration_data) > 0:
        st.subheader("Calibration Results")
        cal_df = pd.DataFrame(_sys.calibration_data, columns=['Power (%)', 'O₃ (%vol)'])
        
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=cal_df['Power (%)'], y=cal_df['O₃ (%vol)'],
            mode='markers+lines',
            name='Measured'
        ))
        fig.update_layout(
            xaxis_title='Power (%)',
            yaxis_title='O₃ (%vol)',
            height=400
        )
        st.plotly_chart(fig)


def render_debug_tab():
    """Render debug tab."""
    st.header("🐛 Debug")
    
    col1, col2 = st.columns([3, 1])
    with col1:
        cmd = st.text_input("Command", placeholder="status", key="manual_cmd")
    with col2:
        st.write("")
        st.write("")
        if st.button("Send", key="send_cmd"):
            if cmd:
                resp = send_command(cmd)
                if resp:
                    st.code(resp)
                else:
                    st.error("No response")
    
    st.divider()
    
    st.subheader("System State")
    col1, col2 = st.columns(2)
    with col1:
        st.write(f"Connected: {_sys.connected}")
        st.write(f"Time synced: {_sys.time_synced}")
        st.write(f"Power target: {_sys.power_target_pct}%")
        st.write(f"Power actual: {_sys.power_actual_pct:.1f}%")
        st.write(f"Power error: {_sys.power_error}")
    with col2:
        st.write(f"O₃ gen relay: {_sys.relay_o3_gen}")
        st.write(f"O₂ conc relay: {_sys.relay_o2_conc}")
        st.write(f"Queue size: {_net.data_queue.qsize()}")
        st.write(f"Buffer size: {len(st.session_state.data_buffer)}")
    
    st.subheader("Debug Log")
    for log in list(st.session_state.debug_logs)[:20]:
        st.text(log)


# =============================================================================
# Main
# =============================================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    args, _ = parser.parse_known_args()
    
    start_receiver(args.port)
    process_queued_data()
    
    # Auto-refresh
    try:
        from streamlit_autorefresh import st_autorefresh
        st_autorefresh(interval=2000, limit=None, key="refresh")
    except ImportError:
        st.warning("Install streamlit-autorefresh for auto-updates")
    
    render_sidebar()
    
    tab1, tab2, tab3, tab4, tab5 = st.tabs([
        "⚡ Power", "🔌 Relays", "📊 Telemetry", "🔧 Calibration", "🐛 Debug"
    ])
    
    with tab1:
        render_power_tab()
    with tab2:
        render_relay_tab()
    with tab3:
        render_telemetry_tab()
    with tab4:
        render_calibration_tab()
    with tab5:
        render_debug_tab()


if __name__ == "__main__":
    main()

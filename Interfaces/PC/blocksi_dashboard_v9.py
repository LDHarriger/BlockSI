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

# Directory structure
DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Data')
CALIBRATION_DIR = os.path.join(DATA_DIR, 'O3PowerCalibration')
MODEL_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Model', 'O3Power')

# Ensure directories exist
for _dir in [DATA_DIR, CALIBRATION_DIR, MODEL_DIR]:
    if not os.path.exists(_dir):
        os.makedirs(_dir)

MOTOR_POT_OHMS = 5000
POWER_MISMATCH_THRESHOLD = 5.0  # % error threshold

# Power model coefficients (from characterization)
# O3_max = 1.78/F + 1.40 where F = flow rate in LPM
POWER_MODEL_A = 1.78
POWER_MODEL_B = 1.40
DEFAULT_FLOW_LPM = 4.0  # Default O2 flow rate

# O3 mass flow conversion: mg/s = %vol * LPM * K
# At STP: K = (48 g/mol) / (22.4 L/mol) / 60 s/min * 10 = 0.357
O3_MASS_FLOW_K = 0.357  # mg/s per (%vol * LPM)

# Calibration sequence settings
CAL_STEP_DURATION_S = 2.0   # Time at each power level (wait for 106-H sample)
CAL_BASELINE_DURATION_S = 30.0  # Zero-power baseline duration
CAL_AIR_DWELL_TIME_S = 20.0  # Time for each Air ON/OFF measurement
CAL_NUM_RANDOM_POINTS = 15  # Number of random power levels in phase 2
AIR_COMP_LPM = 10.0  # Additional LPM when air compressor is on

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
        
        # Flow settings (manually entered from analog meter)
        self.flow_lpm = DEFAULT_FLOW_LPM
        
        # Relays
        self.relay_o3_gen = False
        self.relay_o2_conc = False
        self.relay_air_comp = False  # Air Flow boost (+~10 LPM)
        
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
        
        # Calibration sequence state
        self.cal_active = False
        self.cal_phase = None  # 'baseline', 'sweep_up', 'sweep_down', 'random_pairs'
        self.cal_phase_progress = 0.0  # 0-100% within current phase
        self.cal_current_power = 0
        self.cal_air_state = False  # Current air compressor state during cal
        self.cal_start_time = None
        self.cal_data = []  # List of dicts for each sample
        self.cal_random_powers = []  # List of random power levels for phase 2
        self.cal_random_idx = 0  # Current index in random_powers
        self.cal_o2_lpm = DEFAULT_FLOW_LPM  # O2 LPM setting for this calibration
        
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

# CSV file for this session - created once when session starts
if 'csv_file_path' not in st.session_state:
    st.session_state.csv_file_path = os.path.join(
        DATA_DIR, f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_Stream.csv"
    )
    st.session_state.csv_header_written = False


def log_debug(msg: str):
    ts = datetime.now().strftime('%H:%M:%S')
    st.session_state.debug_logs.appendleft(f"{ts} {msg}")


# =============================================================================
# Power Model & Conversions
# =============================================================================
def predict_o3_from_power(power_pct: float, flow_lpm: float = None) -> float:
    """
    Predict O3 output based on power level.
    Uses piecewise model:
    - 0-20%: minimal output (threshold zone)
    - 20-75%: linear ramp
    - 75-100%: saturation (diminishing returns)
    """
    if flow_lpm is None:
        flow_lpm = _sys.flow_lpm
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


def predict_power_from_o3(o3_pct: float, flow_lpm: float = None) -> float:
    """Inverse of predict_o3_from_power - find power for target O3."""
    if flow_lpm is None:
        flow_lpm = _sys.flow_lpm
    if o3_pct <= 0 or flow_lpm <= 0:
        return 0.0
    
    o3_max = POWER_MODEL_A / flow_lpm + POWER_MODEL_B
    scaling = o3_pct / o3_max
    
    # Clamp to valid range
    if scaling >= 1.0:
        return 100.0
    elif scaling <= 0.1:
        # In threshold zone (0-20%)
        return (scaling / 0.1) * 20
    else:
        # In linear ramp zone (20-75%)
        return 20 + (scaling - 0.1) / 0.9 * 55


def o3_pct_to_mg_per_s(o3_pct: float, flow_lpm: float = None) -> float:
    """Convert O3 %vol to mg/s mass flow rate."""
    if flow_lpm is None:
        flow_lpm = _sys.flow_lpm
    return o3_pct * flow_lpm * O3_MASS_FLOW_K


def mg_per_s_to_o3_pct(mg_per_s: float, flow_lpm: float = None) -> float:
    """Convert mg/s mass flow to O3 %vol."""
    if flow_lpm is None:
        flow_lpm = _sys.flow_lpm
    if flow_lpm <= 0:
        return 0.0
    return mg_per_s / (flow_lpm * O3_MASS_FLOW_K)


def mg_per_s_to_g_at_time(mg_per_s: float, minutes: float = 30.0) -> float:
    """Convert mg/s to grams produced over given time."""
    return mg_per_s * minutes * 60 / 1000


def g_at_time_to_mg_per_s(grams: float, minutes: float = 30.0) -> float:
    """Convert grams at time to mg/s rate."""
    if minutes <= 0:
        return 0.0
    return grams * 1000 / (minutes * 60)


def generate_power_curve(flow_lpm: float = None):
    """Generate power vs O3 curve data."""
    if flow_lpm is None:
        flow_lpm = _sys.flow_lpm
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
    """Append sample to the session's CSV file."""
    try:
        csv_path = st.session_state.csv_file_path
        
        # Write header on first write
        if not st.session_state.csv_header_written:
            with open(csv_path, 'w') as f:
                f.write("timestamp,esp_ts_ms,vessel_o3_pct,cell_temp_c,pressure_mbar,"
                        "room_o3_ppm,vessel_temp_c,power_target,power_actual,wiper_v\n")
            st.session_state.csv_header_written = True
        
        # Append data
        with open(csv_path, 'a') as f:
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
    """Render sidebar matching BlockSI v2 mockup."""
    st.sidebar.title("BlockSI v2")
    
    # Connection status
    if is_connected():
        st.sidebar.markdown("🟢 **Connected**")
    else:
        st.sidebar.markdown("🔴 **Disconnected**")
    
    st.sidebar.divider()
    
    # Relay toggles: Air, O2, O3 (clustered)
    col1, col2, col3 = st.sidebar.columns(3)
    with col1:
        air_icon = "🟢" if _sys.relay_air_comp else "⚪"
        if st.button(f"{air_icon} Air", key="sb_air", help="Air Compressor (+10 LPM)"):
            set_relay('air_comp', not _sys.relay_air_comp)
            st.rerun()
    with col2:
        o2_icon = "🟢" if _sys.relay_o2_conc else "⚪"
        if st.button(f"{o2_icon} O₂", key="sb_o2", help="O2 Concentrator"):
            set_relay('o2_conc', not _sys.relay_o2_conc)
            st.rerun()
    with col3:
        o3_icon = "🟢" if _sys.relay_o3_gen else "⚪"
        if st.button(f"{o3_icon} O₃", key="sb_o3", help="Ozone Generator"):
            set_relay('ozone_gen', not _sys.relay_o3_gen)
            st.rerun()
    
    # O2 LPM manual entry (user reads from analog meter)
    new_lpm = st.sidebar.number_input(
        "O₂ LPM",
        min_value=1.0,
        max_value=15.0,
        value=float(_sys.flow_lpm),
        step=0.5,
        key="sb_lpm",
        help="Manually enter O2 flow rate from analog meter"
    )
    if new_lpm != _sys.flow_lpm:
        _sys.flow_lpm = new_lpm
    
    st.sidebar.divider()
    
    # Sensor readings (display only - underlined style)
    st.sidebar.markdown("**Readings**")
    st.sidebar.markdown(f"<u>O₂</u>: {_sys.flow_lpm:.1f} LPM", unsafe_allow_html=True)
    st.sidebar.markdown(f"<u>O₃</u>: {_sys.vessel_o3_pct:.3f} %vol", unsafe_allow_html=True)
    
    # Target O3 (calculated from current power setting)
    target_o3 = predict_o3_from_power(_sys.power_target_pct)
    st.sidebar.markdown(f"<u>Target %vol O₃</u>: {target_o3:.2f}", unsafe_allow_html=True)


def render_power_tab():
    """Render power control tab matching BlockSI v2 mockup."""
    st.header("BlockSI v2 Settings")
    
    # =========================================================================
    # Main layout: Graph (left) + Settings boxes (right)
    # =========================================================================
    graph_col, settings_col = st.columns([2, 1])
    
    with graph_col:
        # Power slider
        power_pct = st.slider(
            "Power (%)",
            min_value=0,
            max_value=100,
            value=_sys.power_target_pct,
            step=1,
            key="power_slider",
            help="Drag to set power level",
            label_visibility="collapsed"
        )
        
        # Graduated marks (clickable presets)
        cols = st.columns(11)
        for i, preset in enumerate(range(0, 110, 10)):
            with cols[i]:
                # Highlight current target with shading effect
                if preset == _sys.power_target_pct:
                    if st.button(f"▼{preset}", key=f"preset_{preset}"):
                        pass  # Already at this value
                else:
                    if st.button(f"{preset}", key=f"preset_{preset}"):
                        if set_power(preset):
                            st.rerun()
        
        # Power vs O3 curve
        power_vals, o3_vals = generate_power_curve()
        
        fig = go.Figure()
        
        # Model curve (blue line)
        fig.add_trace(go.Scatter(
            x=power_vals, y=o3_vals,
            mode='lines',
            name='Model',
            line=dict(color='blue', width=2),
            showlegend=False
        ))
        
        # Target point (black hollow circle on curve)
        target_o3 = predict_o3_from_power(_sys.power_target_pct)
        fig.add_trace(go.Scatter(
            x=[_sys.power_target_pct], y=[target_o3],
            mode='markers',
            name='Target',
            marker=dict(
                color='rgba(0,0,0,0)',  # Hollow
                size=18,
                line=dict(color='black', width=3)
            ),
            showlegend=False
        ))
        
        # Actual point (solid green circle)
        actual_o3 = _sys.vessel_o3_pct
        fig.add_trace(go.Scatter(
            x=[_sys.power_actual_pct], y=[actual_o3],
            mode='markers',
            name='Actual',
            marker=dict(color='green', size=14),
            showlegend=False
        ))
        
        fig.update_layout(
            xaxis_title='Power',
            yaxis_title='O₃ %vol',
            height=350,
            margin=dict(l=50, r=20, t=20, b=50),
            xaxis=dict(range=[0, 105], dtick=10),
            yaxis=dict(range=[0, max(o3_vals) * 1.1])
        )
        
        st.plotly_chart(fig, key="power_curve")
    
    with settings_col:
        st.markdown("**Settings**")
        
        # LPM setting
        new_lpm = st.number_input(
            "LPM",
            min_value=1.0,
            max_value=15.0,
            value=float(_sys.flow_lpm),
            step=0.5,
            key="settings_lpm",
            help="O2 flow rate"
        )
        if new_lpm != _sys.flow_lpm:
            _sys.flow_lpm = new_lpm
            st.rerun()
        
        st.divider()
        
        # Current values for conversion
        target_o3_pct = predict_o3_from_power(_sys.power_target_pct)
        target_mg_per_s = o3_pct_to_mg_per_s(target_o3_pct)
        target_g_30min = mg_per_s_to_g_at_time(target_mg_per_s, 30.0)
        
        # % Power input
        new_power = st.number_input(
            "% Power",
            min_value=0,
            max_value=100,
            value=_sys.power_target_pct,
            step=1,
            key="settings_power",
            help="Direct power setting"
        )
        if new_power != _sys.power_target_pct:
            if set_power(new_power):
                st.rerun()
        
        # % vol O3 input
        new_o3_pct = st.number_input(
            "% vol O₃",
            min_value=0.0,
            max_value=5.0,
            value=float(target_o3_pct),
            step=0.01,
            format="%.2f",
            key="settings_o3_pct",
            help="Target O3 concentration"
        )
        if abs(new_o3_pct - target_o3_pct) > 0.005:
            new_pwr = int(predict_power_from_o3(new_o3_pct))
            if set_power(new_pwr):
                st.rerun()
        
        # mg O3/s input
        new_mg_s = st.number_input(
            "mg O₃/s",
            min_value=0.0,
            max_value=10.0,
            value=float(target_mg_per_s),
            step=0.01,
            format="%.2f",
            key="settings_mg_s",
            help="O3 mass flow rate"
        )
        if abs(new_mg_s - target_mg_per_s) > 0.005:
            new_o3 = mg_per_s_to_o3_pct(new_mg_s)
            new_pwr = int(predict_power_from_o3(new_o3))
            if set_power(new_pwr):
                st.rerun()
        
        # g O3 @ 30min input
        new_g_30 = st.number_input(
            "g O₃ @ 30min",
            min_value=0.0,
            max_value=20.0,
            value=float(target_g_30min),
            step=0.1,
            format="%.2f",
            key="settings_g_30",
            help="Total O3 produced in 30 minutes"
        )
        if abs(new_g_30 - target_g_30min) > 0.05:
            new_mg = g_at_time_to_mg_per_s(new_g_30, 30.0)
            new_o3 = mg_per_s_to_o3_pct(new_mg)
            new_pwr = int(predict_power_from_o3(new_o3))
            if set_power(new_pwr):
                st.rerun()
    
    # =========================================================================
    # Emergency stop at bottom
    # =========================================================================
    st.divider()
    if st.button("🛑 EMERGENCY STOP", type="primary", key="estop"):
        set_power(0)
        set_relay('ozone_gen', False)
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
    """
    Render calibration interface with automated sequence.
    
    Sequence:
    - Phase 1 (Air OFF): Baseline 30s @ 0%, sweep up 0→100%, sweep down 100→0%
    - Phase 2: 15 random power levels, each with Air OFF 20s then Air ON 20s
    
    Data saved to: Data/O3PowerCalibration/YYYY-MM-DD_PowerO3Cal_{LPM}Lpm.csv
    """
    st.header("🔧 Power-O₃ Calibration")
    
    # === Section 1: Calibration Controls ===
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.markdown("""
        **Automated Calibration Sequence** (~17 min)
        1. **Baseline**: 30s at 0% power (Air OFF)
        2. **Sweep Up**: 0→100% in 1% steps (Air OFF)
        3. **Sweep Down**: 100→0% in 1% steps (Air OFF)
        4. **Random Pairs**: 15 random power levels × (Air OFF 20s + Air ON 20s)
        """)
    
    with col2:
        st.info(f"**O₂ LPM**: {_sys.flow_lpm:.1f}")
        if _sys.relay_air_comp:
            total_lpm = _sys.flow_lpm + AIR_COMP_LPM
            st.info(f"**Air ON** → Total: {total_lpm:.1f} LPM")
    
    # Start/Stop buttons
    if _sys.cal_active:
        col_a, col_b = st.columns([1, 2])
        with col_a:
            if st.button("⏹ Stop Calibration", type="secondary"):
                stop_calibration()
                st.rerun()
        
        # Progress display
        with col_b:
            phase_names = {
                'baseline': '⏳ Baseline (0% power)',
                'sweep_up': '📈 Sweep Up (0→100%)',
                'sweep_down': '📉 Sweep Down (100→0%)',
                'random_pairs': '🎲 Random Power Pairs'
            }
            phase_str = phase_names.get(_sys.cal_phase, _sys.cal_phase or 'Starting...')
            st.progress(_sys.cal_phase_progress / 100, text=f"{phase_str}: {_sys.cal_phase_progress:.0f}%")
        
        # Show current state
        st.markdown(f"""
        | Power | O₃ | Air | Samples |
        |-------|-----|-----|---------|
        | {_sys.cal_current_power}% | {_sys.vessel_o3_pct:.2f}% | {'ON' if _sys.cal_air_state else 'OFF'} | {len(_sys.cal_data)} |
        """)
        
        if _sys.cal_phase == 'random_pairs':
            st.caption(f"Random point {_sys.cal_random_idx + 1}/{CAL_NUM_RANDOM_POINTS}: {_sys.cal_random_powers[_sys.cal_random_idx] if _sys.cal_random_idx < len(_sys.cal_random_powers) else '-'}%")
    else:
        col_a, col_b = st.columns(2)
        with col_a:
            if st.button("▶ Start Calibration", type="primary"):
                if not _sys.connected:
                    st.error("Not connected to ESP32")
                elif not _sys.relay_o2_conc:
                    st.warning("O₂ Concentrator should be ON")
                else:
                    start_calibration()
                    st.rerun()
        with col_b:
            if st.button("📊 Fit Model from Files"):
                fit_unified_model()
    
    st.divider()
    
    # === Section 2: Existing Calibration Files ===
    st.subheader("📁 Calibration Files")
    cal_files = list_calibration_files()
    
    if cal_files:
        for lpm, files in sorted(cal_files.items()):
            with st.expander(f"O₂ @ {lpm} LPM ({len(files)} files)"):
                for f in files:
                    st.text(f"  {os.path.basename(f)}")
    else:
        st.caption("No calibration files found in Data/O3PowerCalibration/")
    
    # === Section 3: Current Session Data ===
    if _sys.cal_data:
        st.subheader("Current Calibration Data")
        cal_df = pd.DataFrame(_sys.cal_data)
        
        # Plot Power vs O3 with Air state color coding
        fig = go.Figure()
        
        # Air OFF points
        air_off = cal_df[cal_df['air_comp_on'] == False]
        if len(air_off) > 0:
            fig.add_trace(go.Scatter(
                x=air_off['power_pct'], y=air_off['o3_pct'],
                mode='markers',
                name='Air OFF',
                marker=dict(color='blue', size=6)
            ))
        
        # Air ON points
        air_on = cal_df[cal_df['air_comp_on'] == True]
        if len(air_on) > 0:
            fig.add_trace(go.Scatter(
                x=air_on['power_pct'], y=air_on['o3_pct'],
                mode='markers',
                name='Air ON',
                marker=dict(color='orange', size=6)
            ))
        
        fig.update_layout(
            xaxis_title='Power (%)',
            yaxis_title='O₃ (%vol)',
            height=350,
            showlegend=True
        )
        st.plotly_chart(fig)
        
        with st.expander("Raw Data"):
            st.dataframe(cal_df.tail(20))


def list_calibration_files():
    """
    List calibration files grouped by O2 LPM setting.
    Returns: {lpm: [filepath, ...], ...}
    """
    files_by_lpm = {}
    if not os.path.exists(CALIBRATION_DIR):
        return files_by_lpm
    
    import re
    for fname in os.listdir(CALIBRATION_DIR):
        if fname.endswith('.csv') and 'PowerO3Cal' in fname:
            # Extract LPM from filename: YYYY-MM-DD_PowerO3Cal_{LPM}Lpm.csv
            match = re.search(r'_(\d+(?:\.\d+)?)Lpm\.csv$', fname)
            if match:
                lpm = float(match.group(1))
                if lpm not in files_by_lpm:
                    files_by_lpm[lpm] = []
                files_by_lpm[lpm].append(os.path.join(CALIBRATION_DIR, fname))
    
    return files_by_lpm


def start_calibration():
    """Initialize and start calibration sequence."""
    _sys.cal_active = True
    _sys.cal_phase = 'baseline'
    _sys.cal_phase_progress = 0.0
    _sys.cal_current_power = 0
    _sys.cal_air_state = False
    _sys.cal_start_time = time.time()
    _sys.cal_data = []
    _sys.cal_o2_lpm = _sys.flow_lpm
    
    # Generate random power levels for phase 2 (avoid extremes, spread evenly)
    np.random.seed(int(time.time()))
    _sys.cal_random_powers = sorted(np.random.randint(5, 96, CAL_NUM_RANDOM_POINTS).tolist())
    _sys.cal_random_idx = 0
    
    # Ensure Air is OFF at start
    send_command("relay_set,air_comp,0")
    _sys.cal_air_state = False
    
    # Set power to 0
    send_command("power_set:0")
    
    # Initialize calibration timing in session state
    if 'cal_step_start' not in st.session_state:
        st.session_state['cal_step_start'] = time.time()
    st.session_state['cal_step_start'] = time.time()


def stop_calibration():
    """Stop calibration and save data."""
    _sys.cal_active = False
    
    # Reset to safe state
    send_command("power_set:0")
    send_command("relay_set,air_comp,0")
    
    # Save data if we have any
    if _sys.cal_data:
        save_calibration_data()
    
    _sys.cal_phase = None
    _sys.cal_data = []


def save_calibration_data():
    """Save calibration data to CSV file."""
    if not _sys.cal_data:
        return
    
    # Filename: YYYY-MM-DD_PowerO3Cal_{LPM}Lpm.csv
    date_str = datetime.now().strftime('%Y-%m-%d')
    lpm_str = f"{_sys.cal_o2_lpm:.0f}" if _sys.cal_o2_lpm == int(_sys.cal_o2_lpm) else f"{_sys.cal_o2_lpm:.1f}"
    filename = f"{date_str}_PowerO3Cal_{lpm_str}Lpm.csv"
    filepath = os.path.join(CALIBRATION_DIR, filename)
    
    # Check for existing file and add suffix if needed
    if os.path.exists(filepath):
        for i in range(2, 100):
            filename = f"{date_str}_PowerO3Cal_{lpm_str}Lpm_{i}.csv"
            filepath = os.path.join(CALIBRATION_DIR, filename)
            if not os.path.exists(filepath):
                break
    
    df = pd.DataFrame(_sys.cal_data)
    df.to_csv(filepath, index=False)
    st.success(f"Saved calibration data to {filename}")


def calibration_step():
    """
    Execute one step of the calibration sequence.
    Called from the main update loop during calibration.
    """
    if not _sys.cal_active:
        return
    
    # Check if enough time has passed for current step
    if 'cal_step_start' not in st.session_state:
        st.session_state['cal_step_start'] = time.time()
    
    elapsed = time.time() - st.session_state['cal_step_start']
    
    if _sys.cal_phase == 'baseline':
        # Wait for baseline duration
        _sys.cal_phase_progress = min(100, (elapsed / CAL_BASELINE_DURATION_S) * 100)
        
        if elapsed >= CAL_STEP_DURATION_S:  # Record sample periodically
            record_calibration_sample()
            st.session_state['cal_step_start'] = time.time()
        
        if elapsed >= CAL_BASELINE_DURATION_S:
            # Move to sweep up
            _sys.cal_phase = 'sweep_up'
            _sys.cal_current_power = 0
            st.session_state['cal_step_start'] = time.time()
            send_command("power_set:0")
    
    elif _sys.cal_phase == 'sweep_up':
        _sys.cal_phase_progress = _sys.cal_current_power
        
        if elapsed >= CAL_STEP_DURATION_S:
            record_calibration_sample()
            
            if _sys.cal_current_power < 100:
                _sys.cal_current_power += 1
                send_command(f"power_set:{_sys.cal_current_power}")
            else:
                # Move to sweep down
                _sys.cal_phase = 'sweep_down'
                _sys.cal_current_power = 100
            
            st.session_state['cal_step_start'] = time.time()
    
    elif _sys.cal_phase == 'sweep_down':
        _sys.cal_phase_progress = 100 - _sys.cal_current_power
        
        if elapsed >= CAL_STEP_DURATION_S:
            record_calibration_sample()
            
            if _sys.cal_current_power > 0:
                _sys.cal_current_power -= 1
                send_command(f"power_set:{_sys.cal_current_power}")
            else:
                # Move to random pairs
                _sys.cal_phase = 'random_pairs'
                _sys.cal_random_idx = 0
                _sys.cal_air_state = False
                st.session_state['cal_step_start'] = time.time()
                if _sys.cal_random_powers:
                    _sys.cal_current_power = _sys.cal_random_powers[0]
                    send_command(f"power_set:{_sys.cal_current_power}")
            
            st.session_state['cal_step_start'] = time.time()
    
    elif _sys.cal_phase == 'random_pairs':
        # Calculate overall progress
        total_steps = CAL_NUM_RANDOM_POINTS * 2  # Each point has Air OFF and Air ON
        current_step = _sys.cal_random_idx * 2 + (1 if _sys.cal_air_state else 0)
        _sys.cal_phase_progress = (current_step / total_steps) * 100
        
        if elapsed >= CAL_AIR_DWELL_TIME_S:
            record_calibration_sample()
            
            if not _sys.cal_air_state:
                # Was Air OFF, switch to Air ON
                _sys.cal_air_state = True
                send_command("relay_set,air_comp,1")
            else:
                # Was Air ON, move to next power level
                _sys.cal_air_state = False
                send_command("relay_set,air_comp,0")
                _sys.cal_random_idx += 1
                
                if _sys.cal_random_idx >= len(_sys.cal_random_powers):
                    # Calibration complete!
                    stop_calibration()
                    st.success("✅ Calibration complete!")
                    return
                else:
                    _sys.cal_current_power = _sys.cal_random_powers[_sys.cal_random_idx]
                    send_command(f"power_set:{_sys.cal_current_power}")
            
            st.session_state['cal_step_start'] = time.time()


def record_calibration_sample():
    """Record current state as a calibration sample."""
    total_lpm = _sys.cal_o2_lpm + (AIR_COMP_LPM if _sys.cal_air_state else 0)
    o2_conc = (_sys.cal_o2_lpm * 0.93 + (AIR_COMP_LPM * 0.21 if _sys.cal_air_state else 0)) / total_lpm
    
    sample = {
        'timestamp': datetime.now().isoformat(),
        'power_pct': _sys.cal_current_power,
        'o3_pct': _sys.vessel_o3_pct,
        'o2_lpm': _sys.cal_o2_lpm,
        'air_comp_on': _sys.cal_air_state,
        'total_lpm': total_lpm,
        'o2_concentration_pct': o2_conc * 100,
        'cell_temp_c': _sys.cell_temp_c,
        'phase': _sys.cal_phase
    }
    _sys.cal_data.append(sample)


def fit_unified_model():
    """
    Fit a unified O3 model from all calibration files.
    Model: O3_pct = f(power, total_lpm, o2_concentration)
    """
    cal_files = list_calibration_files()
    if not cal_files:
        st.error("No calibration files found")
        return
    
    # Load all files
    all_data = []
    for lpm, files in cal_files.items():
        if len(files) > 1:
            # Multiple files at same LPM - use most recent
            files = sorted(files, reverse=True)
            st.info(f"Using most recent file for {lpm} LPM: {os.path.basename(files[0])}")
        
        for f in files[:1]:  # Only use first (most recent)
            try:
                df = pd.read_csv(f)
                all_data.append(df)
            except Exception as e:
                st.warning(f"Error loading {f}: {e}")
    
    if not all_data:
        st.error("No valid data loaded")
        return
    
    combined = pd.concat(all_data, ignore_index=True)
    st.success(f"Loaded {len(combined)} samples from {len(all_data)} files")
    
    # For now, just show summary - full model fitting can be added later
    st.subheader("Data Summary")
    
    # Group by Air state and show curves
    fig = go.Figure()
    
    for air_state in [False, True]:
        subset = combined[combined['air_comp_on'] == air_state]
        if len(subset) > 0:
            # Average O3 at each power level
            avg = subset.groupby('power_pct')['o3_pct'].mean().reset_index()
            fig.add_trace(go.Scatter(
                x=avg['power_pct'], y=avg['o3_pct'],
                mode='lines+markers',
                name=f"Air {'ON' if air_state else 'OFF'}"
            ))
    
    fig.update_layout(
        xaxis_title='Power (%)',
        yaxis_title='O₃ (%vol)',
        height=400
    )
    st.plotly_chart(fig)
    
    # TODO: Fit sigmoid or piecewise model and save to MODEL_DIR
    st.info("Full model fitting coming soon - will save to Model/O3Power/")


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
    
    # Run calibration step if active
    if _sys.cal_active:
        calibration_step()
    
    # Auto-refresh
    try:
        from streamlit_autorefresh import st_autorefresh
        st_autorefresh(interval=2000, limit=None, key="refresh")
    except ImportError:
        st.warning("Install streamlit-autorefresh for auto-updates")
    
    render_sidebar()
    
    tab1, tab2, tab3, tab4 = st.tabs([
        "⚡ Power", "📊 Telemetry", "🔧 Calibration", "🐛 Debug"
    ])
    
    with tab1:
        render_power_tab()
    with tab2:
        render_telemetry_tab()
    with tab3:
        render_calibration_tab()
    with tab4:
        render_debug_tab()


if __name__ == "__main__":
    main()

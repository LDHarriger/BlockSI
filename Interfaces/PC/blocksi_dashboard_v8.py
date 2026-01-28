#!/usr/bin/env python3
"""
BlockSI Dashboard v8 - Stable Architecture Edition

Fixes from v7:
- Background thread for socket receive (no session_state access from thread)
- Thread-safe queue for incoming data
- Blocking send/recv for commands (no polling loops)
- Cleaner connection state management
- Correct DATA field parsing

DATA format from ESP32:
  DATA,timestamp_ms,vessel_o3_pct,temp_c,pressure_mbar,sample_v,ref_v,
       day,month,year,hour,minute,second,room_o3_ppm,vessel_temp_c

Usage:
    pip install streamlit plotly pandas streamlit-autorefresh
    streamlit run blocksi_dashboard_v8.py -- --port 5000
"""

import streamlit as st
import socket
import threading
import queue
import time
import pandas as pd
from datetime import datetime
from collections import deque
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import argparse
import os

# =============================================================================
# Page Config (must be first)
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
CSV_FILE = "blocksi_data_v8.csv"
MOTOR_POT_OHMS = 5000

# =============================================================================
# Global State (outside session_state for thread safety)
# =============================================================================
class ReceiverState:
    """State container for the receiver thread."""
    def __init__(self):
        self.thread = None
        self.running = False
        self.server_socket = None
        self.client_socket = None
        self.lock = threading.Lock()
        self.data_queue = queue.Queue(maxsize=1000)
        self.response_queue = queue.Queue(maxsize=50)  # For RSP lines
        self.connected = False
        self.port = None


@st.cache_resource
def get_receiver_state():
    """Get singleton receiver state (cached across Streamlit reruns)."""
    return ReceiverState()


# Single global instance
_state = get_receiver_state()


def get_client_socket():
    """Thread-safe getter for client socket."""
    with _state.lock:
        return _state.client_socket


def set_client_socket(sock):
    """Thread-safe setter for client socket."""
    with _state.lock:
        _state.client_socket = sock
        _state.connected = sock is not None


def is_connected():
    """Check if connected."""
    return _state.connected


# =============================================================================
# Session State Initialization
# =============================================================================
if 'data_buffer' not in st.session_state:
    st.session_state.data_buffer = deque(maxlen=MAX_DATA_POINTS)

if 'power_state' not in st.session_state:
    st.session_state.power_state = {'percent': 0.0, 'resistance': 0}

if 'relay_state' not in st.session_state:
    st.session_state.relay_state = {'o3_gen': False, 'o2_conc': False}

if 'last_response' not in st.session_state:
    st.session_state.last_response = ""

if 'debug_logs' not in st.session_state:
    st.session_state.debug_logs = deque(maxlen=100)


def log_debug(msg: str):
    """Append debug message."""
    ts = datetime.now().strftime('%H:%M:%S')
    st.session_state.debug_logs.appendleft(f"{ts} {msg}")


# =============================================================================
# Data Parsing
# =============================================================================
def parse_data_line(line: str) -> dict:
    """
    Parse DATA line from ESP32.
    
    Format: DATA,ts,vessel_o3_pct,temp_c,press,sample_v,ref_v,d,m,y,h,min,sec,
                 room_o3_ppm,vessel_temp_c,power_target_pct,power_actual_pct,wiper_voltage
    Indices:  0   1       2         3      4       5      6   7 8 9 10  11  12
                   13           14              15              16             17
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
    
    return {
        'timestamp': datetime.now(),
        'esp_ts_ms': safe_int(1),
        'vessel_o3_pct': safe_float(2),       # 106-H ozone in %vol
        'cell_temp_c': safe_float(3),          # 106-H cell temperature
        'pressure_mbar': safe_float(4),
        'sample_v': safe_float(5),
        'ref_v': safe_float(6),
        'day': safe_int(7),
        'month': safe_int(8),
        'year': safe_int(9),
        'hour': safe_int(10),
        'minute': safe_int(11),
        'second': safe_int(12),
        'room_o3_ppm': safe_float(13),         # DFRobot sensor in ppm
        'vessel_temp_c': safe_float(14, -999.0),  # Thermocouple
        'power_target_pct': safe_int(15),      # Target power % (set value)
        'power_actual_pct': safe_float(16),    # Actual power % from ADC
        'wiper_voltage': safe_float(17),       # Wiper voltage (0-3.3V)
    }


def parse_response(response: str, expected_cmd: str) -> dict:
    """Parse RSP line from ESP32."""
    if not response:
        return None
    try:
        parts = response.strip().split(',')
        if len(parts) < 3 or parts[0] != 'RSP':
            return None
        
        status = parts[1]
        cmd = parts[2]
        
        if cmd != expected_cmd:
            return None
        
        result = {'status': status}
        for part in parts[3:]:
            if '=' in part:
                key, val = part.split('=', 1)
                try:
                    result[key] = float(val) if '.' in val else int(val)
                except ValueError:
                    result[key] = val
        return result
    except Exception:
        return None


# =============================================================================
# Background Receiver Thread
# =============================================================================
def receiver_thread_func(port: int):
    """
    Background thread that:
    1. Creates server socket and waits for connection
    2. Receives data and puts parsed samples into queue
    3. Never touches st.session_state
    """
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', port))
        server.listen(1)
        server.settimeout(1.0)
        _state.server_socket = server
        print(f"[Receiver] Server listening on port {port}")
    except Exception as e:
        print(f"[Receiver] Failed to start server: {e}")
        _state.running = False
        return
    
    rx_buffer = ""
    
    while _state.running:
        # Accept connection if needed
        if get_client_socket() is None:
            try:
                client, addr = server.accept()
                client.settimeout(0.5)
                set_client_socket(client)
                print(f"[Receiver] Connected from {addr}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[Receiver] Accept error: {e}")
                continue
        
        # Receive data
        client = get_client_socket()
        if client is None:
            continue
        
        try:
            data = client.recv(2048)
            if not data:
                # Connection closed
                print("[Receiver] Connection closed by peer")
                try:
                    client.close()
                except:
                    pass
                set_client_socket(None)
                rx_buffer = ""
                continue
            
            rx_buffer += data.decode('utf-8', errors='ignore')
            
            # Process complete lines
            while '\n' in rx_buffer:
                line, rx_buffer = rx_buffer.split('\n', 1)
                line = line.strip()
                if not line:
                    continue
                
                if line.startswith('DATA,'):
                    sample = parse_data_line(line)
                    if sample:
                        try:
                            _state.data_queue.put_nowait(sample)
                        except queue.Full:
                            pass  # Drop if queue full
                elif line.startswith('RSP,'):
                    # Response to command - put in response queue
                    try:
                        _state.response_queue.put_nowait(line)
                    except queue.Full:
                        pass
        
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Receiver] Recv error: {e}")
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
    
    if _state.server_socket:
        try:
            _state.server_socket.close()
        except:
            pass
        _state.server_socket = None
    
    print("[Receiver] Thread stopped")


def start_receiver(port: int):
    """Start the background receiver thread."""
    # Already running on this port?
    if _state.thread is not None and _state.thread.is_alive():
        if _state.port == port:
            return  # Already running on correct port
        # Wrong port - stop and restart
        stop_receiver()
        time.sleep(0.5)
    
    _state.running = True
    _state.port = port
    _state.thread = threading.Thread(target=receiver_thread_func, args=(port,), daemon=True)
    _state.thread.start()


def stop_receiver():
    """Stop the background receiver thread."""
    _state.running = False
    if _state.thread is not None:
        _state.thread.join(timeout=2.0)


# =============================================================================
# Command Sending (from main thread only)
# =============================================================================
def send_command(cmd: str, timeout: float = 2.0) -> str:
    """
    Send command to ESP32 and wait for response from queue.
    The receiver thread captures RSP lines and puts them in the response queue.
    """
    client = get_client_socket()
    if client is None:
        log_debug(f"send_command({cmd}): not connected")
        return None
    
    # Clear any old responses first
    while not _state.response_queue.empty():
        try:
            _state.response_queue.get_nowait()
        except queue.Empty:
            break
    
    try:
        # Send command
        full_cmd = f"CMD,{cmd}\n"
        with _state.lock:
            client.sendall(full_cmd.encode())
        log_debug(f"Sent: {full_cmd.strip()}")
        
        # Wait for response from queue (receiver thread will capture it)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                response = _state.response_queue.get(timeout=0.1)
                st.session_state.last_response = response
                log_debug(f"Got response: {response}")
                return response
            except queue.Empty:
                continue
        
        log_debug(f"Timeout waiting for response to {cmd}")
        return None
    except Exception as e:
        log_debug(f"send_command error: {e}")
        st.session_state.last_response = f"Error: {e}"
        return None


def send_command_with_retry(cmd: str, retries: int = 3) -> str:
    """Send command with retries."""
    for attempt in range(retries):
        resp = send_command(cmd)
        if resp:
            return resp
        log_debug(f"Retry {attempt + 1}/{retries} for {cmd}")
        time.sleep(0.2)
    return None


# =============================================================================
# Data Processing (called from main thread)
# =============================================================================
def process_queued_data():
    """Pull data from queue into session_state buffer."""
    count = 0
    while not _state.data_queue.empty() and count < 50:  # Limit per refresh
        try:
            sample = _state.data_queue.get_nowait()
            st.session_state.data_buffer.append(sample)
            save_sample_to_csv(sample)
            count += 1
        except queue.Empty:
            break


def save_sample_to_csv(sample: dict):
    """Append sample to CSV file."""
    try:
        file_exists = os.path.exists(CSV_FILE)
        with open(CSV_FILE, 'a') as f:
            if not file_exists:
                f.write("timestamp,vessel_o3_pct,cell_temp_c,pressure_mbar,room_o3_ppm,"
                        "vessel_temp_c,power_target_pct,power_actual_pct,wiper_voltage\n")
            f.write(f"{sample['timestamp']},{sample['vessel_o3_pct']},{sample['cell_temp_c']},"
                    f"{sample['pressure_mbar']},{sample['room_o3_ppm']},{sample['vessel_temp_c']},"
                    f"{sample.get('power_target_pct', 0)},{sample.get('power_actual_pct', 0)},"
                    f"{sample.get('wiper_voltage', 0)}\n")
    except Exception:
        pass


# =============================================================================
# UI Components
# =============================================================================
def render_sidebar():
    """Render compact sidebar."""
    st.sidebar.title("🍄 BlockSI Control")
    st.sidebar.caption("v8 - Stable Edition")
    
    # Connection status
    connected = is_connected()
    st.sidebar.markdown(f"### {'🟢 Connected' if connected else '🔴 Disconnected'}")
    st.sidebar.write(f"Buffer: {len(st.session_state.data_buffer)} samples")
    
    st.sidebar.divider()
    
    # Quick relay controls
    st.sidebar.subheader("Quick Controls")
    c1, c2 = st.sidebar.columns(2)
    with c1:
        if st.sidebar.button("O₃ ON", key="sb_o3_on"):
            send_command("relay_set,ozone_gen,1")
    with c2:
        if st.sidebar.button("O₃ OFF", key="sb_o3_off"):
            send_command("relay_set,ozone_gen,0")
    
    c3, c4 = st.sidebar.columns(2)
    with c3:
        if st.sidebar.button("O₂ ON", key="sb_o2_on"):
            send_command("relay_set,o2_conc,1")
    with c4:
        if st.sidebar.button("O₂ OFF", key="sb_o2_off"):
            send_command("relay_set,o2_conc,0")
    
    # Latest readings
    st.sidebar.divider()
    if len(st.session_state.data_buffer) > 0:
        latest = st.session_state.data_buffer[-1]
        st.sidebar.metric("Vessel O₃", f"{latest['vessel_o3_pct']:.3f} %vol")
        st.sidebar.metric("Room O₃", f"{latest['room_o3_ppm']:.3f} ppm")
        if latest['vessel_temp_c'] > -900:
            st.sidebar.metric("Vessel Temp", f"{latest['vessel_temp_c']:.1f} °C")
        # Power status
        if 'power_actual_pct' in latest:
            st.sidebar.metric("Power", f"{latest['power_actual_pct']:.1f}%", 
                              delta=f"{latest.get('wiper_voltage', 0):.2f}V")


def render_power_tab():
    """Render power control tab."""
    st.header("⚡ Power Control")
    
    # Get latest telemetry for actual power readings
    latest = None
    if len(st.session_state.data_buffer) > 0:
        latest = st.session_state.data_buffer[-1]
    
    # Current state - show both target and actual
    st.subheader("Current Status")
    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.metric("Target Power", f"{st.session_state.power_state['percent']:.0f}%")
    with col2:
        if latest and 'power_actual_pct' in latest:
            actual_pct = latest['power_actual_pct']
            st.metric("Actual Power", f"{actual_pct:.1f}%")
        else:
            st.metric("Actual Power", "N/A")
    with col3:
        if latest and 'wiper_voltage' in latest:
            st.metric("Wiper Voltage", f"{latest['wiper_voltage']:.3f} V")
        else:
            st.metric("Wiper Voltage", "N/A")
    with col4:
        resistance = int(st.session_state.power_state['percent'] / 100.0 * MOTOR_POT_OHMS)
        st.metric("Est. Resistance", f"{resistance} Ω")
    
    # Error detection: target vs actual mismatch
    if latest and 'power_actual_pct' in latest and 'power_target_pct' in latest:
        target = latest['power_target_pct']
        actual = latest['power_actual_pct']
        error = abs(target - actual)
        if error > 5.0:  # More than 5% deviation
            st.error(f"⚠️ Power mismatch! Target: {target}%, Actual: {actual:.1f}% (error: {error:.1f}%)")
            st.caption("Motor may have stalled or failed to reach target position.")
    
    st.divider()
    
    # Power input
    col1, col2 = st.columns([3, 1])
    with col1:
        power_pct = st.slider("Power (%)", 0, 100, int(st.session_state.power_state['percent']), key="power_slider")
    with col2:
        if st.button("Apply", type="primary", key="apply_power"):
            resp = send_command_with_retry(f"power_set,{power_pct}")
            if resp:
                parsed = parse_response(resp, "power_set")
                if parsed and parsed.get('status') == 'OK':
                    st.session_state.power_state['percent'] = float(power_pct)
                    st.session_state.power_state['resistance'] = int(power_pct / 100.0 * MOTOR_POT_OHMS)
                    st.success(f"Power set to {power_pct}%")
                else:
                    st.error(f"Command failed: {resp}")
            else:
                st.error("No response from ESP32")
    
    # Quick presets
    st.write("**Presets:**")
    cols = st.columns(6)
    for i, preset in enumerate([0, 20, 40, 60, 80, 100]):
        with cols[i]:
            if st.button(f"{preset}%", key=f"preset_{preset}"):
                resp = send_command_with_retry(f"power_set,{preset}")
                if resp and 'OK' in resp:
                    st.session_state.power_state['percent'] = float(preset)
                    st.session_state.power_state['resistance'] = int(preset / 100.0 * MOTOR_POT_OHMS)
    
    # Emergency stop
    st.divider()
    if st.button("🛑 Emergency Stop", type="secondary"):
        resp = send_command_with_retry("power_set,0")
        st.session_state.power_state['percent'] = 0.0
        st.session_state.power_state['resistance'] = 0
        if resp and 'OK' in resp:
            st.warning("Power set to 0%")
        else:
            st.error("Emergency stop command failed - check connection!")


def render_relay_tab():
    """Render relay control tab."""
    st.header("🔌 Relay Control")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("O₃ Generator")
        state = st.session_state.relay_state['o3_gen']
        st.write(f"Status: {'🟢 ON' if state else '🔴 OFF'}")
        c1, c2 = st.columns(2)
        with c1:
            if st.button("Turn ON", key="relay_o3_on"):
                resp = send_command_with_retry("relay_set,ozone_gen,1")
                if resp and 'OK' in resp:
                    st.session_state.relay_state['o3_gen'] = True
        with c2:
            if st.button("Turn OFF", key="relay_o3_off"):
                resp = send_command_with_retry("relay_set,ozone_gen,0")
                if resp and 'OK' in resp:
                    st.session_state.relay_state['o3_gen'] = False
    
    with col2:
        st.subheader("O₂ Concentrator")
        state = st.session_state.relay_state['o2_conc']
        st.write(f"Status: {'🟢 ON' if state else '🔴 OFF'}")
        c1, c2 = st.columns(2)
        with c1:
            if st.button("Turn ON", key="relay_o2_on"):
                resp = send_command_with_retry("relay_set,o2_conc,1")
                if resp and 'OK' in resp:
                    st.session_state.relay_state['o2_conc'] = True
        with c2:
            if st.button("Turn OFF", key="relay_o2_off"):
                resp = send_command_with_retry("relay_set,o2_conc,0")
                if resp and 'OK' in resp:
                    st.session_state.relay_state['o2_conc'] = False


def render_telemetry_tab():
    """Render telemetry graphs."""
    st.header("📊 Telemetry")
    
    if len(st.session_state.data_buffer) == 0:
        st.info("No data yet. Waiting for ESP32 connection...")
        return
    
    df = pd.DataFrame(list(st.session_state.data_buffer))
    
    # Metrics row
    if len(df) > 0:
        latest = df.iloc[-1]
        col1, col2, col3, col4 = st.columns(4)
        with col1:
            st.metric("Vessel O₃", f"{latest['vessel_o3_pct']:.3f} %vol")
        with col2:
            st.metric("Room O₃", f"{latest['room_o3_ppm']:.3f} ppm")
        with col3:
            if latest['vessel_temp_c'] > -900:
                st.metric("Vessel Temp", f"{latest['vessel_temp_c']:.1f} °C")
            else:
                st.metric("Vessel Temp", "N/A")
        with col4:
            st.metric("Cell Temp", f"{latest['cell_temp_c']:.1f} °C")
    
    # Plots
    fig = make_subplots(
        rows=2, cols=1,
        subplot_titles=("Ozone Concentration", "Temperature"),
        vertical_spacing=0.15,
        specs=[[{"secondary_y": True}], [{"secondary_y": False}]]
    )
    
    # Ozone plot - vessel O3 on primary Y axis
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['vessel_o3_pct'], name="Vessel O₃ (%vol)",
                   line=dict(color='blue', width=2)),
        row=1, col=1, secondary_y=False
    )
    # Room O3 on secondary Y axis (different scale: ppm vs %vol)
    if 'room_o3_ppm' in df.columns:
        fig.add_trace(
            go.Scatter(x=df['timestamp'], y=df['room_o3_ppm'], name="Room O₃ (ppm)",
                       line=dict(color='green', width=2)),
            row=1, col=1, secondary_y=True
        )
    
    # Update axes labels
    fig.update_yaxes(title_text="Vessel O₃ (%vol)", row=1, col=1, secondary_y=False)
    fig.update_yaxes(title_text="Room O₃ (ppm)", row=1, col=1, secondary_y=True)
    
    # Temperature plot
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['cell_temp_c'], name="Cell Temp (°C)",
                   line=dict(color='orange', width=2)),
        row=2, col=1
    )
    if 'vessel_temp_c' in df.columns:
        valid_temps = df[df['vessel_temp_c'] > -900]
        if len(valid_temps) > 0:
            fig.add_trace(
                go.Scatter(x=valid_temps['timestamp'], y=valid_temps['vessel_temp_c'], 
                           name="Vessel Temp (°C)", line=dict(color='red', width=2)),
                row=2, col=1
            )
    
    fig.update_layout(height=500, showlegend=True)
    st.plotly_chart(fig, use_container_width=True)
    
    # Raw data table
    with st.expander("Raw Data"):
        st.dataframe(df.tail(30))


def render_debug_tab():
    """Render debug/manual command tab."""
    st.header("🔧 Debug")
    
    # Manual command
    col1, col2 = st.columns([3, 1])
    with col1:
        cmd = st.text_input("Command", placeholder="status", key="manual_cmd")
    with col2:
        st.write("")  # Spacer
        st.write("")
        if st.button("Send", key="send_manual"):
            if cmd:
                resp = send_command(cmd)
                if resp:
                    st.code(resp)
                else:
                    st.error("No response")
    
    st.divider()
    
    # System commands
    st.subheader("System Commands")
    col1, col2, col3 = st.columns(3)
    with col1:
        if st.button("Get Status"):
            resp = send_command("status")
            if resp:
                st.code(resp)
    with col2:
        if st.button("I2C Scan"):
            resp = send_command("i2c_scan")
            if resp:
                st.code(resp)
    with col3:
        if st.button("Calibrate Motor"):
            with st.spinner("Calibrating..."):
                resp = send_command("motor_pot_calibrate", timeout=20.0)
                if resp:
                    st.code(resp)
    
    st.divider()
    
    # Debug info
    st.subheader("Connection Info")
    st.write(f"Connected: {is_connected()}")
    st.write(f"Last response: {st.session_state.last_response}")
    st.write(f"Buffer size: {len(st.session_state.data_buffer)}")
    st.write(f"Queue size: {_state.data_queue.qsize()}")


# =============================================================================
# Main
# =============================================================================
def main():
    # Parse args
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    args, _ = parser.parse_known_args()
    
    # Start receiver thread (only once)
    start_receiver(args.port)
    
    # Process queued data
    process_queued_data()
    
    # Auto-refresh
    try:
        from streamlit_autorefresh import st_autorefresh
        st_autorefresh(interval=2000, limit=None, key="refresh")
    except ImportError:
        st.warning("Install streamlit-autorefresh: pip install streamlit-autorefresh")
    
    # Render UI
    render_sidebar()
    
    # Tabs
    tab1, tab2, tab3, tab4 = st.tabs(["⚡ Power", "🔌 Relays", "📊 Telemetry", "🔧 Debug"])
    
    with tab1:
        render_power_tab()
    with tab2:
        render_relay_tab()
    with tab3:
        render_telemetry_tab()
    with tab4:
        render_debug_tab()


if __name__ == "__main__":
    main()

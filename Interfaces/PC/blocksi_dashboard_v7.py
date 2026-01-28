#!/usr/bin/env python3
"""
BlockSI Dashboard v7 - Motor Pot Power Control Edition

Updates from v6:
- Replaced DS3502 digital potentiometer with motorized potentiometer (PRM162)
- Removed wiper position controls (motor pot uses percentage only)
- Removed extended mode toggle (motor pot is always 0-100% of 5kΩ)
- Simplified resistance display
- Added motor pot status indicators (position, ADC reading)
- Added calibration command for motor pot

Hardware:
- PRM162-K415K-502B1: Dual 5kΩ motorized rotary potentiometer
- DRV8833: H-bridge motor driver
- Position feedback via ADC on servo track

Usage:
    pip install streamlit plotly pandas streamlit-autorefresh
    streamlit run blocksi_dashboard_v7.py -- --port 5000
"""

import streamlit as st
import socket
import threading
import time
import json
import pandas as pd
from datetime import datetime
from collections import deque
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import argparse
import os

# Must be first Streamlit command
st.set_page_config(
    page_title="BlockSI Control",
    page_icon="🍄",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Auto-refresh (install: pip install streamlit-autorefresh)
try:
    from streamlit_autorefresh import st_autorefresh
    # Default refresh interval (ms) - keep slightly slower to reduce UI flicker
    if 'refresh_interval' not in st.session_state:
        st.session_state.refresh_interval = 3000
    st_autorefresh(interval=st.session_state.refresh_interval, limit=None, key="data_refresh")
except ImportError:
    st.sidebar.warning("Install streamlit-autorefresh for auto-updates")

# =============================================================================
# Configuration
# =============================================================================

DEFAULT_PORT = 5000
MAX_DATA_POINTS = 500
CSV_FILE = "blocksi_data.csv"

# Motor Pot Configuration (PRM162)
MOTOR_POT_OHMS = 5000  # 5kΩ full scale

# =============================================================================
# Session State Initialization
# =============================================================================

if 'data_buffer' not in st.session_state:
    st.session_state.data_buffer = deque(maxlen=MAX_DATA_POINTS)

if 'server_socket' not in st.session_state:
    st.session_state.server_socket = None

if 'client_socket' not in st.session_state:
    st.session_state.client_socket = None

if 'connected' not in st.session_state:
    st.session_state.connected = False

if 'last_response' not in st.session_state:
    st.session_state.last_response = ""

if 'sensor_status' not in st.session_state:
    st.session_state.sensor_status = {
        'pot': 'unknown',
        'lab_o3': 'unknown',
        'thermo': 'unknown'
    }

if 'power_state' not in st.session_state:
    st.session_state.power_state = {
        'percent': 0.0,
        'resistance': 0,
        'predicted_o3': 0.0,
        'adc_raw': 0
    }

if 'relay_state' not in st.session_state:
    st.session_state.relay_state = {'o3_gen': False, 'o2_conc': False}

if 'server_thread' not in st.session_state:
    st.session_state.server_thread = None

if 'server_running' not in st.session_state:
    st.session_state.server_running = False

if 'last_responses' not in st.session_state:
    st.session_state.last_responses = {}

if 'o3_unit' not in st.session_state:
    # Default to %vol for 106H outputs
    st.session_state.o3_unit = '%vol'

if 'debug_logs' not in st.session_state:
    st.session_state.debug_logs = deque(maxlen=200)

def log_debug(msg: str):
    """Append debug message with timestamp."""
    try:
        ts = datetime.now().strftime('%H:%M:%S')
        st.session_state.debug_logs.appendleft(f"{ts} {msg}")
    except Exception:
        pass

# =============================================================================
# Helper Functions
# =============================================================================

def percent_to_resistance(percent: float) -> int:
    """Convert percentage to resistance in ohms."""
    return int((percent / 100.0) * MOTOR_POT_OHMS)

def resistance_to_percent(ohms: int) -> float:
    """Convert resistance to percentage."""
    return (ohms / MOTOR_POT_OHMS) * 100.0

def parse_response(response: str, expected_cmd: str) -> dict:
    """Parse ESP32 response into dict."""
    if not response:
        return None
    
    try:
        # Response format: RSP,OK/ERR,cmd,key=val,key=val,...
        parts = response.strip().split(',')
        if len(parts) < 3:
            return None
        
        status = parts[1]
        cmd = parts[2]
        
        if cmd != expected_cmd:
            return None
        
        result = {'status': status}
        
        for part in parts[3:]:
            if '=' in part:
                key, val = part.split('=', 1)
                # Try to convert to number
                try:
                    if '.' in val:
                        result[key] = float(val)
                    else:
                        result[key] = int(val)
                except ValueError:
                    result[key] = val
        
        return result
    except Exception as e:
        st.error(f"Parse error: {e}")
        return None

# =============================================================================
# Network Functions
# =============================================================================

def start_server(port: int):
    """Start a non-blocking TCP server to receive data from ESP32.

    This function creates the listening socket only. The Streamlit main
    loop must call `check_server()` to accept connections and receive data.
    """
    if st.session_state.server_running:
        return

    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', port))
        server.listen(1)
        server.setblocking(False)
        st.session_state.server_socket = server
        st.session_state.server_running = True
        log_debug(f"Server started on port {port}")
    except Exception as e:
        st.error(f"Server error: {e}")
        log_debug(f"Server error: {e}")

def stop_server():
    """Stop TCP server."""
    st.session_state.server_running = False
    if st.session_state.client_socket:
        try:
            st.session_state.client_socket.close()
        except:
            pass
        st.session_state.client_socket = None
    if st.session_state.server_socket:
        try:
            st.session_state.server_socket.close()
        except:
            pass
        st.session_state.server_socket = None
    st.session_state.connected = False


def check_server():
    """Accept new connections and receive data (call from Streamlit main loop)."""
    server = st.session_state.server_socket
    if server is None:
        return

    # Accept new connection if none
    if st.session_state.client_socket is None:
        try:
            client, addr = server.accept()
            client.setblocking(False)
            st.session_state.client_socket = client
            st.session_state.connected = True
            log_debug(f"Accepted connection from {addr}")
        except BlockingIOError:
            pass
        except Exception:
            pass

    # Receive data from existing connection
    if st.session_state.client_socket is not None:
        try:
            data = st.session_state.client_socket.recv(1024)
            if data:
                s = data.decode('utf-8', errors='ignore')
                log_debug(f"Received {len(s)} bytes")
                process_incoming_data(s)
            else:
                # Connection closed
                try:
                    st.session_state.client_socket.close()
                except:
                    pass
                st.session_state.client_socket = None
                st.session_state.connected = False
        except BlockingIOError:
            pass
        except Exception:
            try:
                st.session_state.client_socket.close()
            except:
                pass
            st.session_state.client_socket = None
            st.session_state.connected = False
            log_debug("Client connection lost")

def send_command(cmd: str) -> str:
    """Send command to ESP32 and wait for response."""
    if not st.session_state.client_socket:
        st.warning("Not connected to ESP32")
        return None
    try:
        full_cmd = f"CMD,{cmd}\n"
        st.session_state.client_socket.send(full_cmd.encode())
        log_debug(f"Sent CMD: {cmd}")

        # Expected command token (before any commas)
        expected_cmd = cmd.split(',')[0]
        timeout = 2.0
        deadline = time.time() + timeout
        # Poll for response populated by check_server()
        while time.time() < deadline:
            # Process any incoming bytes
            try:
                check_server()
            except Exception:
                pass
            # Check if response arrived
            resp = st.session_state.last_responses.get(expected_cmd)
            if resp:
                return resp
            time.sleep(0.05)

        st.warning("Command timeout")
        log_debug(f"Command timeout: {cmd}")
        return None
    except Exception as e:
        st.error(f"Command error: {e}")
        log_debug(f"Command error: {e}")
        return None


def send_command_with_retry(cmd: str, retries: int = 3, delay: float = 0.25):
    """Send a command and retry if no response.

    Returns the response string or None.
    """
    for attempt in range(1, retries + 1):
        resp = send_command(cmd)
        if resp:
            return resp
        time.sleep(delay)
    return None

def process_incoming_data(data: str):
    """Process incoming telemetry data."""
    for line in data.strip().split('\n'):
        if line.startswith('DATA,'):
            # Parse telemetry: DATA,timestamp,vessel_o3_pct,temp,pressure,sample_v,ref_v,day,month,year,hour,minute,second,room_o3_ppm,vessel_temp_c
            try:
                parts = line.split(',')
                if len(parts) >= 5:
                    sample = {
                        'timestamp': datetime.now(),
                        'vessel_o3_pct': float(parts[2]) if parts[2] else 0.0,
                        'temp_c': float(parts[3]) if parts[3] else 0.0,
                        'pressure_mbar': float(parts[4]) if parts[4] else 0.0,
                        'sample_v': float(parts[5]) if len(parts) > 5 and parts[5] else None,
                        'ref_v': float(parts[6]) if len(parts) > 6 and parts[6] else None,
                        'day': int(parts[7]) if len(parts) > 7 and parts[7] else 0,
                        'month': int(parts[8]) if len(parts) > 8 and parts[8] else 0,
                        'year': int(parts[9]) if len(parts) > 9 and parts[9] else 0,
                        'hour': int(parts[10]) if len(parts) > 10 and parts[10] else 0,
                        'minute': int(parts[11]) if len(parts) > 11 and parts[11] else 0,
                        'second': int(parts[12]) if len(parts) > 12 and parts[12] else 0,
                        'room_o3_ppm': float(parts[13]) if len(parts) > 13 and parts[13] else None,
                        'vessel_temp_c': float(parts[14]) if len(parts) > 14 and parts[14] else None,
                    }
                    st.session_state.data_buffer.append(sample)
                    
                    # Save to CSV
                    save_sample_to_csv(sample)
            except Exception:
                pass  # Ignore parse errors
        elif line.startswith('RSP,'):
            # Store latest response and index by command for polling
            try:
                st.session_state.last_response = line.strip()
                parts = line.split(',')
                if len(parts) >= 3:
                    cmd = parts[2]
                    st.session_state.last_responses[cmd] = line.strip()
            except Exception:
                pass

def save_sample_to_csv(sample: dict):
    """Append sample to CSV file."""
    try:
        file_exists = os.path.exists(CSV_FILE)
        with open(CSV_FILE, 'a') as f:
            if not file_exists:
                f.write("timestamp,vessel_o3_pct,temp_c,pressure_mbar,sample_v,ref_v,day,month,year,hour,minute,second,room_o3_ppm,vessel_temp_c\n")
            f.write(
                f"{sample.get('timestamp')},{sample.get('vessel_o3_pct')},{sample.get('temp_c')},{sample.get('pressure_mbar')},"
                f"{sample.get('sample_v')},{sample.get('ref_v')},{sample.get('day')},{sample.get('month')},{sample.get('year')},"
                f"{sample.get('hour')},{sample.get('minute')},{sample.get('second')},{sample.get('room_o3_ppm')},{sample.get('vessel_temp_c')}\n"
            )
    except Exception:
        pass

# =============================================================================
# UI Components
# =============================================================================

def render_sidebar():
    """Render sidebar with connection controls."""
    st.sidebar.title("🍄 BlockSI Control")
    st.sidebar.caption("Motor Pot Edition v7")
    # Compact connection indicator and quick stats
    st.sidebar.divider()
    connected = st.session_state.connected
    status_icon = "🟢" if connected else "🔴"
    st.sidebar.write(f"{status_icon} {'Connected' if connected else 'Disconnected'}")
    st.sidebar.write(f"Buffer: {len(st.session_state.data_buffer)}")
    st.sidebar.write(f"Last: {st.session_state.last_response or 'None'}")
    st.sidebar.divider()
    st.sidebar.write("Quick Actions")
    # Quick relay toggles (mirror of main controls)
    c1, c2 = st.sidebar.columns(2)
    with c1:
        if st.button("O3 ON"):
            send_command("relay_set,1,1")
    with c2:
        if st.button("O3 OFF"):
            send_command("relay_set,1,0")
    c3, c4 = st.sidebar.columns(2)
    with c3:
        if st.button("O2 ON"):
            send_command("relay_set,2,1")
    with c4:
        if st.button("O2 OFF"):
            send_command("relay_set,2,0")

def render_power_control():
    """Render power control section."""
    st.header("⚡ Power Control")
    
    st.info(f"""
    **Motorized Potentiometer (PRM162)**
    - Resistance range: 0-{MOTOR_POT_OHMS}Ω
    - Position feedback via ADC
    - Closed-loop position control
    """)
    
    st.divider()
    
    # Top quick controls: relays and power numeric + vessel O3
    top1, top2, top3, top4 = st.columns([1, 1, 2, 2])
    with top1:
        if st.button("O3 ON", key="top_o3_on"):
            send_command("relay_set,1,1")
    with top2:
        if st.button("O3 OFF", key="top_o3_off"):
            send_command("relay_set,1,0")
    with top3:
        power_pct_input = st.number_input("Power (%)", min_value=0, max_value=100, value=int(st.session_state.power_state['percent']), step=1, key="power_input")
        if st.button("Apply Power", key="apply_power_btn"):
            resp = send_command_with_retry(f"power_set,{power_pct_input}")
            if resp:
                parsed = parse_response(resp, "power_set")
                if parsed and parsed.get('status') == 'OK':
                    st.success(f"Power set to {power_pct_input}%")
                    st.session_state.power_state['percent'] = float(power_pct_input)
                else:
                    st.error("Failed to apply power")
            else:
                st.error("No response from device")
    with top4:
        # Display latest vessel and room O3 readings if available
        latest_vessel = None
        latest_room = None
        if len(st.session_state.data_buffer) > 0:
            latest = st.session_state.data_buffer[-1]
            latest_vessel = latest.get('vessel_o3_pct')
            latest_room = latest.get('room_o3_ppm')

        if latest_vessel is not None:
            st.metric("Vessel O₃", f"{latest_vessel:.3f} %vol")
        else:
            st.metric("Vessel O₃", "-")

        if latest_room is not None:
            st.metric("Room O₃", f"{latest_room:.3f} ppm")
        else:
            st.metric("Room O₃", "-")
    
    # Current state display
    col1, col2, col3 = st.columns(3)
    
    power_state = st.session_state.power_state
    with col1:
        st.metric("Power", f"{power_state['percent']:.1f}%")
    with col2:
        st.metric("Resistance", f"{power_state['resistance']}Ω")
    with col3:
        st.metric("Predicted O₃", f"{power_state['predicted_o3']:.2f} ppm")
    
    st.divider()
    
    # Power control slider
    st.subheader("Set Power Level")
    
    # Percentage slider
    power_pct = st.slider(
        "Power (%)",
        min_value=0.0,
        max_value=100.0,
        value=float(power_state['percent']),
        step=1.0,
        key="power_slider"
    )
    
    # Calculate corresponding resistance
    target_resistance = percent_to_resistance(power_pct)
    
    # Show what will be set
    st.write(f"→ Target Resistance: {target_resistance}Ω")
    
    # Apply button
    col1, col2, col3 = st.columns([1, 1, 2])
    with col1:
        if st.button("Apply", type="primary"):
            response = send_command(f"power_set,{power_pct:.0f}")
            if response:
                parsed = parse_response(response, "power_set")
                if parsed and parsed.get('status') == 'OK':
                    st.success(f"Power set to {power_pct:.0f}%")
                    # Update state
                    st.session_state.power_state['percent'] = power_pct
                    st.session_state.power_state['resistance'] = target_resistance
                else:
                    st.error("Failed to set power")
    
    with col2:
        if st.button("🛑 Emergency Stop", type="secondary"):
            response = send_command("power_set,0")
            if response:
                st.session_state.power_state = {
                    'percent': 0.0,
                    'resistance': 0,
                    'predicted_o3': 0.0,
                    'adc_raw': 0
                }
                st.warning("Emergency stop activated")
    
    st.divider()
    
    # Quick presets
    st.subheader("Quick Presets")
    cols = st.columns(5)
    presets = [0, 25, 50, 75, 100]
    for i, preset in enumerate(presets):
        with cols[i]:
            if st.button(f"{preset}%", key=f"preset_{preset}"):
                response = send_command(f"power_set,{preset}")
                if response:
                    st.session_state.power_state['percent'] = preset
                    st.session_state.power_state['resistance'] = percent_to_resistance(preset)
    
    st.divider()
    
    # Motor pot calibration
    with st.expander("🔧 Motor Pot Calibration"):
        st.write("""
        Calibration drives the motor to both end stops to determine the full ADC range.
        This improves position accuracy.
        """)
        
        if st.button("Run Calibration"):
            with st.spinner("Calibrating... (may take up to 20 seconds)"):
                response = send_command("motor_pot_calibrate")
                if response:
                    st.success(f"Calibration complete: {response}")
                else:
                    st.error("Calibration failed or timed out")

def render_relay_control():
    """Render relay control section."""
    st.header("🔌 Relay Control")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("O₃ Generator")
        relay_state = st.session_state.relay_state['o3_gen']
        st.write(f"Status: {'🟢 ON' if relay_state else '🔴 OFF'}")
        
        col_on, col_off = st.columns(2)
        with col_on:
            if st.button("Turn ON", key="o3_on"):
                response = send_command_with_retry("relay_set,1,1")
                if response:
                    parsed = parse_response(response, "relay_set")
                    if parsed and parsed.get('status') == 'OK':
                        st.session_state.relay_state['o3_gen'] = True
        with col_off:
            if st.button("Turn OFF", key="o3_off"):
                response = send_command_with_retry("relay_set,1,0")
                if response:
                    parsed = parse_response(response, "relay_set")
                    if parsed and parsed.get('status') == 'OK':
                        st.session_state.relay_state['o3_gen'] = False
    
    with col2:
        st.subheader("O₂ Concentrator")
        relay_state = st.session_state.relay_state['o2_conc']
        st.write(f"Status: {'🟢 ON' if relay_state else '🔴 OFF'}")
        
        col_on, col_off = st.columns(2)
        with col_on:
            if st.button("Turn ON", key="o2_on"):
                response = send_command_with_retry("relay_set,2,1")
                if response:
                    parsed = parse_response(response, "relay_set")
                    if parsed and parsed.get('status') == 'OK':
                        st.session_state.relay_state['o2_conc'] = True
        with col_off:
            if st.button("Turn OFF", key="o2_off"):
                response = send_command_with_retry("relay_set,2,0")
                if response:
                    parsed = parse_response(response, "relay_set")
                    if parsed and parsed.get('status') == 'OK':
                        st.session_state.relay_state['o2_conc'] = False


def render_settings():
    """Render settings such as server port and refresh interval."""
    st.header("⚙️ Settings")
    col1, col2 = st.columns(2)
    with col1:
        port = st.number_input("TCP Port", value=DEFAULT_PORT, min_value=1024, max_value=65535, key="settings_port")
        if st.button("Start Server", key="settings_start"):
            start_server(port)
        if st.button("Stop Server", key="settings_stop"):
            stop_server()

    with col2:
        st.subheader("UI / Refresh")
        interval = st.number_input("Refresh interval (ms)", value=int(st.session_state.refresh_interval), min_value=500, max_value=60000, step=500, key="refresh_interval_input")
        if st.button("Apply Refresh", key="apply_refresh"):
            st.session_state.refresh_interval = int(interval)
            st.success("Refresh interval updated (applies on next rerun)")

    st.divider()
    st.subheader("O₃ Display")
    unit = st.selectbox("O₃ unit", ['%vol', 'ppm'], index=0)
    st.session_state.o3_unit = unit
    
    st.divider()
    st.subheader("Debug Logs")
    if st.button("Clear Logs"):
        st.session_state.debug_logs.clear()
    with st.expander("Recent Logs", expanded=False):
        for line in list(st.session_state.debug_logs)[:200]:
            st.text(line)

def render_telemetry():
    """Render telemetry graphs."""
    st.header("📊 Telemetry")
    
    if len(st.session_state.data_buffer) == 0:
        st.info("No data yet. Connect to ESP32 to start receiving telemetry.")
        return
    
    df = pd.DataFrame(list(st.session_state.data_buffer))
    
    # Create plots
    fig = make_subplots(
        rows=2, cols=1,
        subplot_titles=("Ozone Concentration", "Temperature"),
        vertical_spacing=0.15
    )
    
    # O3 plot (vessel percent) and Room O3 as secondary series if present
    o3_label = "Vessel O₃ (%vol)"
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['vessel_o3_pct'], name=o3_label,
                   line=dict(color='blue')),
        row=1, col=1
    )
    if 'room_o3_ppm' in df.columns:
        fig.add_trace(
            go.Scatter(x=df['timestamp'], y=df['room_o3_ppm'], name='Room O₃ (ppm)',
                       line=dict(color='green')),
            row=1, col=1
        )
    
    # Temperature plot (vessel)
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['vessel_temp_c'] if 'vessel_temp_c' in df.columns else df['temp_c'], name="Vessel Temp (°C)",
                   line=dict(color='red')),
        row=2, col=1
    )
    
    fig.update_layout(height=500, showlegend=True)
    st.plotly_chart(fig, use_container_width=True)
    
    # Data table
    with st.expander("Raw Data"):
        st.dataframe(df.tail(20))

def render_manual_command():
    """Render manual command entry."""
    with st.expander("🔧 Manual Command"):
        cmd = st.text_input("Command (without CMD, prefix)", placeholder="power_get")
        if st.button("Send"):
            if cmd:
                response = send_command(cmd)
                if response:
                    st.code(response)

# =============================================================================
# Main App
# =============================================================================

def main():
    # Parse command line args
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    
    # Streamlit passes extra args, so ignore unknown
    args, _ = parser.parse_known_args()
    # Start server automatically (match v6 behavior)
    start_server(args.port)

    # Render UI
    render_sidebar()
    # Process any incoming connections/data
    check_server()
    
    # Main content tabs
    tab1, tab2, tab3, tab4, tab5 = st.tabs(["⚡ Power", "🔌 Relays", "📊 Telemetry", "🔧 Debug", "⚙️ Settings"])
    
    with tab1:
        render_power_control()
    
    with tab2:
        render_relay_control()
    
    with tab3:
        render_telemetry()
    
    with tab4:
        render_manual_command()
        
        st.divider()
        
        # System info
        st.subheader("System Commands")
        col1, col2, col3 = st.columns(3)
        with col1:
            if st.button("Get Status"):
                response = send_command("status")
                if response:
                    st.code(response)
        with col2:
            if st.button("I2C Scan"):
                response = send_command("i2c_scan")
                if response:
                    st.code(response)
        with col3:
            if st.button("Reboot ESP32"):
                if st.checkbox("Confirm reboot"):
                    send_command("reboot")
                    st.warning("Rebooting...")

    with tab5:
        render_settings()

if __name__ == "__main__":
    main()

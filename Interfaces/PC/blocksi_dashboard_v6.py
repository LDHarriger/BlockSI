#!/usr/bin/env python3
"""
BlockSI Dashboard v6 - DS3502 Power Control Edition

Updates from v5:
- Replaced MCP4725 DAC control with DS3502 digital potentiometer
- Added extended range mode (full 10kΩ vs original 4.7kΩ)
- Added resistance display alongside percentage
- Added calibration sweep interface
- Improved power control visualization

Usage:
    pip install streamlit plotly pandas streamlit-autorefresh
    streamlit run blocksi_dashboard_v6.py -- --port 5000
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
    st_autorefresh(interval=2000, limit=None, key="data_refresh")
except ImportError:
    st.sidebar.warning("Install streamlit-autorefresh for auto-updates")

# =============================================================================
# Configuration
# =============================================================================

DEFAULT_PORT = 5000
MAX_DATA_POINTS = 500
CSV_FILE = "blocksi_data.csv"

# DS3502 Configuration
DS3502_WIPER_MAX = 127
DS3502_FULL_SCALE_OHMS = 10000
DS3502_WIPER_RESISTANCE = 40
ORIGINAL_POT_OHMS = 4700
ORIGINAL_POT_WIPER_MAX = 60  # Wiper position for ~4.7kΩ

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
    st.session_state.sensor_status = {}

if 'power_state' not in st.session_state:
    st.session_state.power_state = {
        'wiper': 0,
        'resistance': 40,
        'percent': 0,
        'mode': 'original',
        'predicted_o3': 0
    }

if 'relay_state' not in st.session_state:
    st.session_state.relay_state = {'ozone_gen': 0, 'o2_conc': 0}

if 'recording' not in st.session_state:
    st.session_state.recording = False

# =============================================================================
# Helper Functions
# =============================================================================

def wiper_to_resistance(wiper):
    """Convert wiper position to resistance in ohms"""
    r = (wiper * DS3502_FULL_SCALE_OHMS) / DS3502_WIPER_MAX
    return int(r + DS3502_WIPER_RESISTANCE)

def resistance_to_wiper(ohms):
    """Convert resistance to wiper position"""
    if ohms <= DS3502_WIPER_RESISTANCE:
        return 0
    adjusted = ohms - DS3502_WIPER_RESISTANCE
    wiper = (adjusted * DS3502_WIPER_MAX) / DS3502_FULL_SCALE_OHMS
    return min(int(wiper), DS3502_WIPER_MAX)

def percent_to_wiper(percent, extended_mode=False):
    """Convert percentage to wiper position"""
    max_wiper = DS3502_WIPER_MAX if extended_mode else ORIGINAL_POT_WIPER_MAX
    return int((percent / 100.0) * max_wiper)

def wiper_to_percent(wiper, extended_mode=False):
    """Convert wiper position to percentage"""
    max_wiper = DS3502_WIPER_MAX if extended_mode else ORIGINAL_POT_WIPER_MAX
    percent = (wiper / max_wiper) * 100.0
    return min(percent, 100.0)

def send_command(cmd):
    """Send command to ESP32 and return response"""
    if st.session_state.client_socket is None:
        return None
    
    try:
        st.session_state.client_socket.sendall(f"CMD,{cmd}\n".encode())
        st.session_state.client_socket.settimeout(2.0)
        response = st.session_state.client_socket.recv(256).decode().strip()
        st.session_state.last_response = response
        return response
    except Exception as e:
        st.session_state.last_response = f"Error: {e}"
        return None

def parse_data_message(msg):
    """Parse DATA message from ESP32"""
    try:
        parts = msg.split(',')
        if len(parts) >= 12 and parts[0] == 'DATA':
            return {
                'timestamp': datetime.now(),
                'esp_ts': int(parts[1]),
                'o3_ppm': float(parts[2]),
                'temp_c': float(parts[3]),
                'press_mbar': float(parts[4]),
                'sample_v': float(parts[5]),
                'ref_v': float(parts[6]),
                'day': int(parts[7]),
                'month': int(parts[8]),
                'year': int(parts[9]),
                'hour': int(parts[10]),
                'minute': int(parts[11]),
                'second': int(parts[12]) if len(parts) > 12 else 0
            }
    except (ValueError, IndexError) as e:
        pass
    return None

def parse_response(response, expected_cmd):
    """Parse RSP message and return dict of key=value pairs"""
    if response is None:
        return None
    
    try:
        parts = response.split(',')
        if len(parts) >= 3 and parts[0] == 'RSP' and parts[1] == 'OK':
            result = {}
            for part in parts[3:]:
                if '=' in part:
                    k, v = part.split('=', 1)
                    result[k] = v
            return result
    except:
        pass
    return None

# =============================================================================
# TCP Server
# =============================================================================

def start_server(port):
    """Start TCP server for ESP32 connection"""
    if st.session_state.server_socket is not None:
        return True
    
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', port))
        server.listen(1)
        server.setblocking(False)
        st.session_state.server_socket = server
        return True
    except Exception as e:
        st.error(f"Failed to start server: {e}")
        return False

def check_connection():
    """Check for new connections and receive data"""
    if st.session_state.server_socket is None:
        return
    
    # Accept new connections
    if st.session_state.client_socket is None:
        try:
            client, addr = st.session_state.server_socket.accept()
            client.setblocking(False)
            st.session_state.client_socket = client
            st.session_state.connected = True
        except BlockingIOError:
            pass
    
    # Receive data from existing connection
    if st.session_state.client_socket is not None:
        try:
            data = st.session_state.client_socket.recv(1024).decode()
            if data:
                for line in data.strip().split('\n'):
                    parsed = parse_data_message(line)
                    if parsed:
                        st.session_state.data_buffer.append(parsed)
                        if st.session_state.recording:
                            save_to_csv(parsed)
            else:
                # Connection closed
                st.session_state.client_socket.close()
                st.session_state.client_socket = None
                st.session_state.connected = False
        except BlockingIOError:
            pass
        except Exception as e:
            st.session_state.client_socket = None
            st.session_state.connected = False

def save_to_csv(data):
    """Append data to CSV file"""
    df = pd.DataFrame([data])
    header = not os.path.exists(CSV_FILE)
    df.to_csv(CSV_FILE, mode='a', header=header, index=False)

# =============================================================================
# UI Components
# =============================================================================

def render_sidebar():
    """Render sidebar with connection status and controls"""
    st.sidebar.title("🍄 BlockSI Control")
    
    # Connection status
    if st.session_state.connected:
        st.sidebar.success("● ESP32 Connected")
    else:
        st.sidebar.warning("○ Waiting for ESP32...")
    
    st.sidebar.divider()
    
    # Recording control
    col1, col2 = st.sidebar.columns(2)
    with col1:
        if st.button("🔴 Record" if not st.session_state.recording else "⏹ Stop"):
            st.session_state.recording = not st.session_state.recording
    with col2:
        st.write("Recording" if st.session_state.recording else "Stopped")
    
    # Data info
    st.sidebar.write(f"Buffer: {len(st.session_state.data_buffer)} points")
    
    st.sidebar.divider()
    
    # Last response
    st.sidebar.text("Last Response:")
    st.sidebar.code(st.session_state.last_response or "None", language=None)

def render_live_data_tab():
    """Render live data visualization"""
    st.header("📊 Live Data")
    
    if len(st.session_state.data_buffer) < 2:
        st.info("Waiting for data...")
        return
    
    df = pd.DataFrame(list(st.session_state.data_buffer))
    
    # Current values
    latest = df.iloc[-1]
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        st.metric("O₃ Concentration", f"{latest['o3_ppm']:.3f} ppm")
    with col2:
        st.metric("Temperature", f"{latest['temp_c']:.1f} °C")
    with col3:
        st.metric("Pressure", f"{latest['press_mbar']:.1f} mbar")
    with col4:
        st.metric("Data Points", len(df))
    
    # Plot
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True,
                        subplot_titles=("Ozone Concentration", "Temperature & Pressure"),
                        vertical_spacing=0.1)
    
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['o3_ppm'], name='O₃ (ppm)',
                   line=dict(color='#FF6B6B', width=2)),
        row=1, col=1
    )
    
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['temp_c'], name='Temp (°C)',
                   line=dict(color='#4ECDC4', width=2)),
        row=2, col=1
    )
    
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['press_mbar'], name='Pressure (mbar)',
                   line=dict(color='#45B7D1', width=2), yaxis='y3'),
        row=2, col=1
    )
    
    fig.update_layout(height=500, showlegend=True,
                      legend=dict(orientation="h", yanchor="bottom", y=1.02))
    
    st.plotly_chart(fig, use_container_width=True)

def render_power_control_tab():
    """Render DS3502 power control interface"""
    st.header("⚡ Power Control (DS3502)")
    
    # Mode selection
    col1, col2 = st.columns([1, 2])
    with col1:
        mode = st.radio(
            "Operating Mode",
            ["Original (4.7kΩ)", "Extended (10kΩ)"],
            help="Original mode limits to the original potentiometer range"
        )
        extended_mode = "Extended" in mode
        st.session_state.power_state['mode'] = 'extended' if extended_mode else 'original'
    
    with col2:
        st.info(f"""
        **{'Extended' if extended_mode else 'Original'} Mode**
        - Wiper range: 0-{DS3502_WIPER_MAX if extended_mode else ORIGINAL_POT_WIPER_MAX}
        - Resistance: 0-{DS3502_FULL_SCALE_OHMS if extended_mode else ORIGINAL_POT_OHMS}Ω
        - {'Full DS3502 range - may extend useful control range' if extended_mode else 'Matches original 4.7kΩ potentiometer behavior'}
        """)
    
    st.divider()
    
    # Current state display
    col1, col2, col3, col4 = st.columns(4)
    
    power_state = st.session_state.power_state
    with col1:
        st.metric("Power", f"{power_state['percent']:.1f}%")
    with col2:
        st.metric("Wiper", f"{power_state['wiper']}")
    with col3:
        st.metric("Resistance", f"{power_state['resistance']}Ω")
    with col4:
        st.metric("Predicted O₃", f"{power_state['predicted_o3']:.2f} ppm")
    
    st.divider()
    
    # Power control slider
    max_wiper = DS3502_WIPER_MAX if extended_mode else ORIGINAL_POT_WIPER_MAX
    
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
    
    # Calculate corresponding values
    target_wiper = percent_to_wiper(power_pct, extended_mode)
    target_resistance = wiper_to_resistance(target_wiper)
    
    # Show what will be set
    st.write(f"→ Wiper: {target_wiper}, Resistance: {target_resistance}Ω")
    
    # Apply button
    col1, col2, col3 = st.columns([1, 1, 2])
    with col1:
        if st.button("Apply", type="primary"):
            response = send_command(f"power_set,{power_pct:.0f}")
            if response:
                parsed = parse_response(response, "power_set")
                if parsed:
                    st.success(f"Power set to {power_pct:.0f}%")
                    # Update state
                    st.session_state.power_state['percent'] = power_pct
                    st.session_state.power_state['wiper'] = target_wiper
                    st.session_state.power_state['resistance'] = target_resistance
    
    with col2:
        if st.button("🛑 Emergency Stop", type="secondary"):
            send_command("power_set,0")
            st.session_state.power_state = {
                'wiper': 0, 'resistance': 40, 'percent': 0,
                'mode': st.session_state.power_state['mode'],
                'predicted_o3': 0
            }
            st.warning("Power set to 0%")
    
    # Refresh state
    with col3:
        if st.button("🔄 Refresh"):
            response = send_command("power_get")
            if response:
                parsed = parse_response(response, "power_get")
                if parsed:
                    st.session_state.power_state.update({
                        'percent': float(parsed.get('pct', 0)),
                        'wiper': int(parsed.get('wiper', 0)),
                        'resistance': int(parsed.get('resistance', 40)),
                        'predicted_o3': float(parsed.get('pred', 0))
                    })
    
    st.divider()
    
    # Advanced: Direct wiper control
    with st.expander("Advanced: Direct Wiper Control"):
        wiper_val = st.number_input(
            "Wiper Position (0-127)",
            min_value=0,
            max_value=127,
            value=power_state['wiper'],
            step=1
        )
        
        if st.button("Set Wiper Directly"):
            response = send_command(f"wiper_set,{wiper_val}")
            if response:
                st.success(f"Wiper set to {wiper_val}")

def render_relay_tab():
    """Render relay control interface"""
    st.header("🔌 Relay Control")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("Ozone Generator")
        relay_state = st.session_state.relay_state.get('ozone_gen', 0)
        
        if st.button(
            "Turn OFF" if relay_state else "Turn ON",
            key="relay_o3",
            type="primary" if not relay_state else "secondary"
        ):
            new_state = 0 if relay_state else 1
            response = send_command(f"relay_set,ozone_gen,{new_state}")
            if response and 'OK' in response:
                st.session_state.relay_state['ozone_gen'] = new_state
        
        st.write(f"Status: {'🟢 ON' if relay_state else '🔴 OFF'}")
    
    with col2:
        st.subheader("O₂ Concentrator")
        relay_state = st.session_state.relay_state.get('o2_conc', 0)
        
        if st.button(
            "Turn OFF" if relay_state else "Turn ON",
            key="relay_o2",
            type="primary" if not relay_state else "secondary"
        ):
            new_state = 0 if relay_state else 1
            response = send_command(f"relay_set,o2_conc,{new_state}")
            if response and 'OK' in response:
                st.session_state.relay_state['o2_conc'] = new_state
        
        st.write(f"Status: {'🟢 ON' if relay_state else '🔴 OFF'}")
    
    # Refresh
    if st.button("🔄 Refresh Relay Status"):
        response = send_command("relay_get")
        if response:
            parsed = parse_response(response, "relay_get")
            if parsed:
                st.session_state.relay_state = {
                    'ozone_gen': int(parsed.get('ozone_gen', 0)),
                    'o2_conc': int(parsed.get('o2_conc', 0))
                }

def render_sensors_tab():
    """Render secondary sensors status"""
    st.header("🌡 Sensors")
    
    # Refresh button
    if st.button("🔄 Refresh Sensors"):
        response = send_command("sensors_get")
        if response:
            parsed = parse_response(response, "sensors_get")
            if parsed:
                st.session_state.sensor_status = parsed
    
    status = st.session_state.sensor_status
    
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.subheader("DS3502 DigiPot")
        digipot_ok = status.get('digipot', 'unknown')
        if digipot_ok == 'ok':
            st.success("✓ Connected")
        else:
            st.error("✗ Not found")
    
    with col2:
        st.subheader("Room O₃ Sensor")
        room_o3 = status.get('room_o3', 'unknown')
        try:
            o3_val = float(room_o3)
            st.metric("Concentration", f"{o3_val:.3f} ppm")
        except:
            st.warning(f"Status: {room_o3}")
    
    with col3:
        st.subheader("Thermocouple")
        thermo = status.get('vessel_temp', 'unknown')
        try:
            temp_val = float(thermo)
            st.metric("Temperature", f"{temp_val:.1f} °C")
        except:
            st.warning(f"Status: {thermo}")

def render_calibration_tab():
    """Render power calibration interface"""
    st.header("📈 Power Calibration")
    
    st.info("""
    Run a calibration sweep to characterize the O₃ output vs power relationship.
    The ESP32 will step through wiper positions and record O₃ readings.
    """)
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("Sweep Parameters")
        start_wiper = st.number_input("Start Wiper", 0, 127, 0)
        end_wiper = st.number_input("End Wiper", 0, 127, 127)
        step_size = st.number_input("Step Size", 1, 20, 5)
        hold_time = st.number_input("Hold Time (seconds)", 10, 300, 30)
    
    with col2:
        st.subheader("Estimated Duration")
        num_steps = abs(end_wiper - start_wiper) // step_size + 1
        total_time = num_steps * hold_time
        st.write(f"Steps: {num_steps}")
        st.write(f"Total time: {total_time // 60}m {total_time % 60}s")
        
        st.divider()
        
        if st.button("🚀 Start Calibration", type="primary"):
            response = send_command(f"cal_start,{start_wiper},{end_wiper},{step_size},{hold_time * 1000}")
            if response:
                st.success("Calibration started!")
        
        if st.button("⏹ Stop Calibration"):
            send_command("cal_stop")
            st.warning("Calibration stopped")

def render_106h_tab():
    """Render 106-H configuration interface"""
    st.header("📟 106-H Configuration")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("Averaging Time")
        avg_options = ["2s", "10s", "1m", "5m", "1h"]
        avg_time = st.selectbox("Select averaging time", avg_options, index=1)
        
        if st.button("Set Averaging Time"):
            response = send_command(f"106h_avg,{avg_time}")
            if response and 'OK' in response:
                st.success(f"Averaging time set to {avg_time}")
    
    with col2:
        st.subheader("Zero Calibration")
        st.warning("Ensure clean air before zeroing!")
        
        if st.button("Send Zero Command"):
            response = send_command("106h_zero")
            if response and 'OK' in response:
                st.success("Zero command sent")

# =============================================================================
# Main
# =============================================================================

def main():
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    args, _ = parser.parse_known_args()
    
    # Start server
    if start_server(args.port):
        st.sidebar.info(f"Server listening on port {args.port}")
    
    # Check for connections/data
    check_connection()
    
    # Render sidebar
    render_sidebar()
    
    # Main tabs
    tabs = st.tabs([
        "📊 Live Data",
        "⚡ Power Control", 
        "🔌 Relays",
        "🌡 Sensors",
        "📈 Calibration",
        "📟 106-H"
    ])
    
    with tabs[0]:
        render_live_data_tab()
    
    with tabs[1]:
        render_power_control_tab()
    
    with tabs[2]:
        render_relay_tab()
    
    with tabs[3]:
        render_sensors_tab()
    
    with tabs[4]:
        render_calibration_tab()
    
    with tabs[5]:
        render_106h_tab()

if __name__ == "__main__":
    main()

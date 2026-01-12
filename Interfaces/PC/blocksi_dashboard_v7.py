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
    st_autorefresh(interval=2000, limit=None, key="data_refresh")
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
    """Start TCP server to receive data from ESP32."""
    if st.session_state.server_running:
        return
    
    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', port))
        server.listen(1)
        server.settimeout(1.0)
        st.session_state.server_socket = server
        st.session_state.server_running = True
        
        def server_loop():
            while st.session_state.server_running:
                try:
                    if st.session_state.client_socket is None:
                        try:
                            client, addr = server.accept()
                            client.settimeout(0.5)
                            st.session_state.client_socket = client
                            st.session_state.connected = True
                        except socket.timeout:
                            continue
                    else:
                        try:
                            data = st.session_state.client_socket.recv(1024)
                            if data:
                                process_incoming_data(data.decode('utf-8', errors='ignore'))
                            else:
                                # Connection closed
                                st.session_state.client_socket.close()
                                st.session_state.client_socket = None
                                st.session_state.connected = False
                        except socket.timeout:
                            pass
                        except Exception:
                            st.session_state.client_socket = None
                            st.session_state.connected = False
                except Exception as e:
                    time.sleep(0.1)
        
        thread = threading.Thread(target=server_loop, daemon=True)
        thread.start()
        st.session_state.server_thread = thread
        
    except Exception as e:
        st.error(f"Server error: {e}")

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

def send_command(cmd: str) -> str:
    """Send command to ESP32 and wait for response."""
    if not st.session_state.client_socket:
        st.warning("Not connected to ESP32")
        return None
    
    try:
        full_cmd = f"CMD,{cmd}\n"
        st.session_state.client_socket.send(full_cmd.encode())
        
        # Wait for response
        st.session_state.client_socket.settimeout(2.0)
        response = st.session_state.client_socket.recv(256).decode('utf-8', errors='ignore')
        st.session_state.last_response = response.strip()
        return response.strip()
    except socket.timeout:
        st.warning("Command timeout")
        return None
    except Exception as e:
        st.error(f"Command error: {e}")
        return None

def process_incoming_data(data: str):
    """Process incoming telemetry data."""
    for line in data.strip().split('\n'):
        if line.startswith('DATA,'):
            # Parse telemetry: DATA,timestamp,o3_ppm,temp,pressure,...
            try:
                parts = line.split(',')
                if len(parts) >= 5:
                    sample = {
                        'timestamp': datetime.now(),
                        'o3_ppm': float(parts[2]) if parts[2] else 0,
                        'temp_c': float(parts[3]) if parts[3] else 0,
                        'pressure_mbar': float(parts[4]) if parts[4] else 0,
                    }
                    st.session_state.data_buffer.append(sample)
                    
                    # Save to CSV
                    save_sample_to_csv(sample)
            except Exception as e:
                pass  # Ignore parse errors

def save_sample_to_csv(sample: dict):
    """Append sample to CSV file."""
    try:
        file_exists = os.path.exists(CSV_FILE)
        with open(CSV_FILE, 'a') as f:
            if not file_exists:
                f.write("timestamp,o3_ppm,temp_c,pressure_mbar\n")
            f.write(f"{sample['timestamp']},{sample['o3_ppm']},{sample['temp_c']},{sample['pressure_mbar']}\n")
    except Exception:
        pass

# =============================================================================
# UI Components
# =============================================================================

def render_sidebar():
    """Render sidebar with connection controls."""
    st.sidebar.title("🍄 BlockSI Control")
    st.sidebar.caption("Motor Pot Edition v7")
    
    st.sidebar.divider()
    
    # Connection settings
    st.sidebar.subheader("Connection")
    port = st.sidebar.number_input("TCP Port", value=DEFAULT_PORT, min_value=1024, max_value=65535)
    
    col1, col2 = st.sidebar.columns(2)
    with col1:
        if st.button("Start Server"):
            start_server(port)
    with col2:
        if st.button("Stop Server"):
            stop_server()
    
    # Connection status
    if st.session_state.connected:
        st.sidebar.success("✅ ESP32 Connected")
    elif st.session_state.server_running:
        st.sidebar.info("🔄 Waiting for connection...")
    else:
        st.sidebar.warning("⚪ Server stopped")
    
    st.sidebar.divider()
    
    # Sensor status
    st.sidebar.subheader("Sensor Status")
    if st.sidebar.button("Refresh Status"):
        response = send_command("sensors_get")
        if response:
            # Parse: pot=ok,lab_o3=ok,thermo=ok,...
            for part in response.split(','):
                if '=' in part:
                    key, val = part.split('=', 1)
                    if key in ['pot', 'lab_o3', 'thermo']:
                        st.session_state.sensor_status[key] = val
    
    status = st.session_state.sensor_status
    st.sidebar.write(f"Motor Pot: {'✅' if status['pot'] == 'ok' else '❌'} {status['pot']}")
    st.sidebar.write(f"Lab O₃: {'✅' if status['lab_o3'] == 'ok' else '❌'} {status['lab_o3']}")
    st.sidebar.write(f"Thermocouple: {'✅' if status['thermo'] == 'ok' else '❌'} {status['thermo']}")
    
    st.sidebar.divider()
    
    # Last response debug
    with st.sidebar.expander("Debug"):
        st.text(st.session_state.last_response)

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
    
    # Refresh power state
    col_refresh, col_spacer = st.columns([1, 4])
    with col_refresh:
        if st.button("🔄 Refresh", key="refresh_power"):
            response = send_command("power_get")
            if response:
                parsed = parse_response(response, "power_get")
                if parsed and parsed.get('status') == 'OK':
                    st.session_state.power_state['percent'] = parsed.get('pct', 0)
                    st.session_state.power_state['resistance'] = parsed.get('resistance', 0)
                    st.session_state.power_state['predicted_o3'] = parsed.get('pred', 0)
    
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
                response = send_command("relay_set,1,1")
                if response:
                    st.session_state.relay_state['o3_gen'] = True
        with col_off:
            if st.button("Turn OFF", key="o3_off"):
                response = send_command("relay_set,1,0")
                if response:
                    st.session_state.relay_state['o3_gen'] = False
    
    with col2:
        st.subheader("O₂ Concentrator")
        relay_state = st.session_state.relay_state['o2_conc']
        st.write(f"Status: {'🟢 ON' if relay_state else '🔴 OFF'}")
        
        col_on, col_off = st.columns(2)
        with col_on:
            if st.button("Turn ON", key="o2_on"):
                response = send_command("relay_set,2,1")
                if response:
                    st.session_state.relay_state['o2_conc'] = True
        with col_off:
            if st.button("Turn OFF", key="o2_off"):
                response = send_command("relay_set,2,0")
                if response:
                    st.session_state.relay_state['o2_conc'] = False

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
    
    # O3 plot
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['o3_ppm'], name="O₃ (ppm)", 
                   line=dict(color='blue')),
        row=1, col=1
    )
    
    # Temperature plot
    fig.add_trace(
        go.Scatter(x=df['timestamp'], y=df['temp_c'], name="Temp (°C)",
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
    
    # Render UI
    render_sidebar()
    
    # Main content tabs
    tab1, tab2, tab3, tab4 = st.tabs(["⚡ Power", "🔌 Relays", "📊 Telemetry", "🔧 Debug"])
    
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

if __name__ == "__main__":
    main()

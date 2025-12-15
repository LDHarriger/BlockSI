#!/usr/bin/env python3
"""
BlockSI Receiver v2 - TCP server for ESP32 ozone control system

Features:
- Continuous data streaming display
- Automatic date-based CSV files
- Sequence-triggered file creation
- Press ':' for command mode
"""

import socket
import threading
import argparse
import sys
import os
import csv
import time
from datetime import datetime
from queue import Queue, Empty

# Platform-specific keyboard handling
if sys.platform == 'win32':
    import msvcrt
else:
    import select
    import tty
    import termios


class DataLogger:
    """Manages CSV file logging with sequence support"""
    
    def __init__(self, output_dir='.'):
        self.output_dir = output_dir
        self.current_file = None
        self.csv_writer = None
        self.current_filename = None
        self.sequence_name = None
        self.sample_count = 0
        
    def _get_date_prefix(self):
        return datetime.now().strftime('%Y-%m-%d')
    
    def _write_header(self):
        self.csv_writer.writerow([
            'pc_timestamp', 'esp_timestamp_ms', 'ozone_wt_pct', 'temperature_c',
            'pressure_mbar', 'sample_pdv_v', 'ref_pdv_v',
            'day', 'month', 'year', 'hour', 'minute', 'second',
            'sequence'
        ])
        self.current_file.flush()
    
    def start_stream(self):
        """Start or resume default stream file"""
        if self.sequence_name:
            self.stop_sequence()
        
        filename = f"{self._get_date_prefix()}_Stream.csv"
        filepath = os.path.join(self.output_dir, filename)
        
        file_exists = os.path.exists(filepath)
        self.current_file = open(filepath, 'a', newline='')
        self.csv_writer = csv.writer(self.current_file)
        self.current_filename = filename
        self.sequence_name = None
        
        if not file_exists:
            self._write_header()
            return f"Created: {filename}"
        return f"Appending to: {filename}"
    
    def start_sequence(self, sequence_type):
        """Start a new sequence file"""
        if self.current_file:
            self.current_file.close()
        
        self.sequence_name = sequence_type
        filename = f"{self._get_date_prefix()}_{sequence_type}.csv"
        filepath = os.path.join(self.output_dir, filename)
        
        self.current_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.current_file)
        self.current_filename = filename
        self.sample_count = 0
        
        self._write_header()
        return f"Started sequence: {filename}"
    
    def stop_sequence(self):
        """Stop sequence and return to stream file"""
        if self.current_file:
            self.current_file.close()
        
        old_sequence = self.sequence_name
        self.sequence_name = None
        self.start_stream()
        return f"Ended sequence: {old_sequence}"
    
    def write_sample(self, data):
        """Write a sample to current file"""
        if not self.csv_writer:
            return
        
        pc_timestamp = datetime.now().isoformat()
        row = [
            pc_timestamp,
            data.get('esp_ts', 0),
            data.get('o3', 0),
            data.get('temp', 0),
            data.get('press', 0),
            data.get('sample_v', 0),
            data.get('ref_v', 0),
            data.get('day', 0),
            data.get('month', 0),
            data.get('year', 0),
            data.get('hour', 0),
            data.get('minute', 0),
            data.get('second', 0),
            self.sequence_name or 'stream'
        ]
        self.csv_writer.writerow(row)
        self.current_file.flush()
        self.sample_count += 1
    
    def close(self):
        if self.current_file:
            self.current_file.close()
            self.current_file = None


class BlockSIReceiver:
    def __init__(self, port, output_dir):
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_addr = None
        self.running = False
        self.connected = False
        
        self.last_sample = None
        self.device_info = None
        self.logger = DataLogger(output_dir)
        self.display_queue = Queue()
        self.command_mode = False
        self.command_buffer = ""
        
    def queue_message(self, msg, is_status=False):
        """Queue a message for display"""
        self.display_queue.put(('status' if is_status else 'message', msg))
    
    def process_data(self, line):
        """Process DATA message from ESP32"""
        parts = line.split(',')
        if len(parts) < 13:
            return
        
        try:
            data = {
                'esp_ts': int(parts[1]),
                'o3': float(parts[2]),
                'temp': float(parts[3]),
                'press': float(parts[4]),
                'sample_v': float(parts[5]),
                'ref_v': float(parts[6]),
                'day': int(parts[7]),
                'month': int(parts[8]),
                'year': int(parts[9]),
                'hour': int(parts[10]),
                'minute': int(parts[11]),
                'second': int(parts[12])
            }
            
            self.last_sample = data
            self.logger.write_sample(data)
            
            seq = f" [{self.logger.sequence_name}]" if self.logger.sequence_name else ""
            status = (f"O3={data['o3']:.3f}%  T={data['temp']:.1f}C  "
                     f"P={data['press']:.1f}mbar  [#{self.logger.sample_count}]{seq}")
            self.queue_message(status, is_status=True)
            
        except (ValueError, IndexError) as e:
            self.queue_message(f"[ERR] Parse error: {e}")
    
    def process_response(self, line):
        """Process RSP message from ESP32"""
        parts = line.split(',', 3)
        if len(parts) >= 3:
            status = parts[1]
            cmd = parts[2]
            details = parts[3] if len(parts) > 3 else ''
            prefix = "[OK]" if status == 'OK' else "[ERR]"
            self.queue_message(f"{prefix} {cmd}: {details}")
    
    def process_file_command(self, line):
        """Process FILE command from ESP32"""
        parts = line.split(',')
        if len(parts) >= 2:
            action = parts[1].upper()
            if action == 'START' and len(parts) >= 3:
                msg = self.logger.start_sequence(parts[2])
                self.queue_message(f"[FILE] {msg}")
            elif action == 'STOP':
                msg = self.logger.stop_sequence()
                self.queue_message(f"[FILE] {msg}")
    
    def process_line(self, line):
        """Process a line received from ESP32"""
        line = line.strip()
        if not line:
            return
        
        if line.startswith('DATA,'):
            self.process_data(line)
        elif line.startswith('RSP,'):
            self.process_response(line)
        elif line.startswith('FILE,'):
            self.process_file_command(line)
        elif line.startswith('HELLO,'):
            self.device_info = line[6:]
            self.queue_message(f"[DEVICE] {self.device_info}")
        else:
            self.queue_message(f"[RX] {line}")
    
    def send_command(self, cmd_line):
        """Send command to ESP32"""
        if not self.connected or not self.client_socket:
            self.queue_message("[ERR] Not connected")
            return False
        
        parts = cmd_line.strip().split()
        if not parts:
            return False
        
        cmd = parts[0].lower()
        
        if cmd == 'relay_set' and len(parts) >= 3:
            msg = f"CMD,relay_set,{parts[1]},{parts[2]}\n"
        elif cmd == 'relay_get':
            msg = "CMD,relay_get\n"
        elif cmd == 'relay_all_off':
            msg = "CMD,relay_all_off\n"
        elif cmd == 'status':
            msg = "CMD,status\n"
        elif cmd == 'seq_start' and len(parts) >= 2:
            result = self.logger.start_sequence(parts[1])
            self.queue_message(f"[FILE] {result}")
            return True
        elif cmd == 'seq_stop':
            result = self.logger.stop_sequence()
            self.queue_message(f"[FILE] {result}")
            return True
        elif cmd == 'raw' and len(parts) >= 2:
            msg = "CMD," + ','.join(parts[1:]) + "\n"
        else:
            self.queue_message(f"[ERR] Unknown: {cmd}")
            return False
        
        try:
            self.client_socket.sendall(msg.encode())
            return True
        except Exception as e:
            self.queue_message(f"[ERR] Send failed: {e}")
            return False
    
    def receiver_thread(self):
        """Thread to receive data from ESP32"""
        buffer = ""
        
        while self.running:
            if not self.connected:
                try:
                    self.server_socket.settimeout(0.5)
                    self.client_socket, self.client_addr = self.server_socket.accept()
                    self.client_socket.settimeout(0.1)
                    self.connected = True
                    self.queue_message(f"\n[CONNECTED] {self.client_addr[0]}:{self.client_addr[1]}")
                except socket.timeout:
                    continue
                except Exception:
                    continue
            
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    self.queue_message("\n[DISCONNECTED]")
                    self.connected = False
                    self.client_socket = None
                    continue
                
                buffer += data.decode('utf-8', errors='replace')
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.process_line(line)
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self.connected:
                    self.queue_message(f"\n[ERR] {e}")
                    self.connected = False
                    self.client_socket = None
    
    def show_help(self):
        """Show command help"""
        help_text = """
══════════════════════════════════════════════════════════
 Commands:
   relay_set ozone_gen 1  - Turn on ozone generator
   relay_set ozone_gen 0  - Turn off ozone generator  
   relay_set o2_conc 1    - Turn on O2 concentrator
   relay_set o2_conc 0    - Turn off O2 concentrator
   relay_get              - Get relay states
   relay_all_off          - Emergency stop all relays
   status                 - Get device status
   
   seq_start <name>       - Start recording to new file
   seq_stop               - Stop recording, return to stream
   
   help                   - Show this help
   quit                   - Exit program
══════════════════════════════════════════════════════════"""
        print(help_text)
    
    def process_display_queue(self):
        """Process pending display messages"""
        try:
            while True:
                msg_type, msg = self.display_queue.get_nowait()
                
                if self.command_mode:
                    sys.stdout.write('\r' + ' ' * 80 + '\r')
                    print(msg)
                    sys.stdout.write(':' + self.command_buffer)
                    sys.stdout.flush()
                else:
                    if msg_type == 'status':
                        sys.stdout.write('\r' + msg + ' ' * 20)
                        sys.stdout.flush()
                    else:
                        sys.stdout.write('\r' + ' ' * 80 + '\r')
                        print(msg)
                        sys.stdout.flush()
                        
        except Empty:
            pass
    
    def handle_keyboard_windows(self):
        """Handle keyboard input on Windows"""
        if not msvcrt.kbhit():
            return True
        
        ch = msvcrt.getwch()
        
        if not self.command_mode:
            if ch == ':' or ch == '/':
                self.command_mode = True
                self.command_buffer = ""
                sys.stdout.write('\n:')
                sys.stdout.flush()
            elif ch == 'q' or ch == 'Q':
                return False
            elif ch == '?':
                print()
                self.show_help()
        else:
            if ch == '\r' or ch == '\n':
                print()
                cmd = self.command_buffer.strip()
                self.command_mode = False
                self.command_buffer = ""
                
                if cmd:
                    if cmd.lower() in ('quit', 'q'):
                        return False
                    elif cmd.lower() in ('help', '?'):
                        self.show_help()
                    else:
                        self.send_command(cmd)
                        
            elif ch == '\x1b':
                print()
                self.command_mode = False
                self.command_buffer = ""
                
            elif ch == '\x08':
                if self.command_buffer:
                    self.command_buffer = self.command_buffer[:-1]
                    sys.stdout.write('\b \b')
                    sys.stdout.flush()
            else:
                self.command_buffer += ch
                sys.stdout.write(ch)
                sys.stdout.flush()
        
        return True
    
    def handle_keyboard_unix(self):
        """Handle keyboard input on Unix"""
        if not select.select([sys.stdin], [], [], 0)[0]:
            return True
        
        ch = sys.stdin.read(1)
        
        if not self.command_mode:
            if ch == ':' or ch == '/':
                self.command_mode = True
                self.command_buffer = ""
                sys.stdout.write('\n:')
                sys.stdout.flush()
            elif ch == 'q' or ch == 'Q':
                return False
            elif ch == '?':
                print()
                self.show_help()
        else:
            if ch == '\n':
                print()
                cmd = self.command_buffer.strip()
                self.command_mode = False
                self.command_buffer = ""
                
                if cmd:
                    if cmd.lower() in ('quit', 'q'):
                        return False
                    elif cmd.lower() in ('help', '?'):
                        self.show_help()
                    else:
                        self.send_command(cmd)
                        
            elif ch == '\x1b':
                print()
                self.command_mode = False
                self.command_buffer = ""
                
            elif ch == '\x7f':
                if self.command_buffer:
                    self.command_buffer = self.command_buffer[:-1]
                    sys.stdout.write('\b \b')
                    sys.stdout.flush()
            else:
                self.command_buffer += ch
                sys.stdout.write(ch)
                sys.stdout.flush()
        
        return True
    
    def run(self):
        """Main run loop"""
        msg = self.logger.start_stream()
        print(f"[CSV] {msg}")
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(1)
        
        self.running = True
        
        print("\n" + "=" * 60)
        print(f"  BlockSI Receiver v2 - Port {self.port}")
        print(f"  Press ':' for commands, '?' for help, 'q' to quit")
        print("=" * 60)
        print("\nWaiting for ESP32 connection...\n")
        
        rx_thread = threading.Thread(target=self.receiver_thread, daemon=True)
        rx_thread.start()
        
        old_settings = None
        if sys.platform != 'win32':
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        
        try:
            while self.running:
                self.process_display_queue()
                
                if sys.platform == 'win32':
                    if not self.handle_keyboard_windows():
                        break
                else:
                    if not self.handle_keyboard_unix():
                        break
                
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            pass
        finally:
            if old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
            self.running = False
            print("\n\nShutting down...")
            
            if self.client_socket:
                self.client_socket.close()
            if self.server_socket:
                self.server_socket.close()
            
            self.logger.close()
            
            print(f"Total samples: {self.logger.sample_count}")
            print(f"Last file: {self.logger.current_filename}")


def main():
    parser = argparse.ArgumentParser(description='BlockSI ESP32 Receiver v2')
    parser.add_argument('--port', '-p', type=int, default=5000,
                        help='TCP port (default: 5000)')
    parser.add_argument('--output-dir', '-d', type=str, default='.',
                        help='Output directory for CSV files (default: current)')
    
    args = parser.parse_args()
    
    if args.output_dir != '.' and not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    
    receiver = BlockSIReceiver(args.port, args.output_dir)
    receiver.run()


if __name__ == '__main__':
    main()

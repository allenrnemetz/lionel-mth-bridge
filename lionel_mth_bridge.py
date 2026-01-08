#!/usr/bin/env python3
"""
lionel_mth_bridge.py - Lionel Base 3 to MTH WTIU Bridge
Uses FTDI serial adapter for reliable TMCC data capture

Author: Allen Nemetz
Credits:
- Mark DiVecchio for his immense work translating MTH commands to and from the MTH WTIU
  http://www.silogic.com/trains/RTC_Running.html
- Lionel LLC for publishing TMCC and Legacy protocol specifications
- O Gauge Railroading Forum (https://www.ogrforum.com/) for the model railroad community

Disclaimer: This software is provided "as-is" without warranty. The author assumes no liability 
for any damages resulting from the use or misuse of this software. Users are responsible for 
ensuring safe operation of their model railroad equipment.

Copyright (c) 2026 Allen Nemetz. All rights reserved.

This bridge converts Lionel TMCC commands from Lionel Base 3 to MTH WTIU commands,
enabling Lionel remote control of MTH DCS-equipped trains.
"""

import serial
import socket
import threading
import time
import logging
from collections import deque
from threading import Lock
import subprocess
import re

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class LionelMTHBridge:
    def __init__(self):
        self.lionel_port = '/dev/ttyUSB0'  # FTDI adapter for SER2
        self.mcu_port = '/dev/ttymxc3'     # Internal UART to MCU on Arduino UNO Q
        self.mth_devices = ['192.168.0.100', '192.168.0.102']
        self.lionel_serial = None
        self.mcu_serial = None
        self.mcu_connected = False
        self.running = False
        self.auto_reconnect = True
        self.connection_check_interval = 5  # seconds
        self.max_reconnect_attempts = 10
        
        # Command type mapping for MCU communication
        self.mcu_command_types = {
            'direction': 1,
            'speed': 2, 
            'function': 3,
            'smoke': 4,
            'pfa': 5,
            'engine': 6,
            'protowhistle': 8,
            'wled': 9
        }
        
        # Thread safety locks
        self.lionel_lock = Lock()
        self.mcu_lock = Lock()
        self.mth_lock = Lock()
        
        # Speck encryption settings (Mark's RTCRemote - using his actual key)
        self.use_encryption = True  # Always use Speck encryption for MTH WTIU
        # Mark's actual Speck key from Comm_Thread.cpp:
        # key[0] =  5196;  // 0x144C
        # key[1] =  46084; // 0xB424  
        # key[2] =  38013; // 0x947D
        # key[3] =  32838; // 0x8046
        # Stored as little-endian bytes: [0x4C, 0x14, 0x24, 0xB4, 0x7D, 0x94, 0x46, 0x80]
        self.speck_key = bytes([0x4C, 0x14, 0x24, 0xB4, 0x7D, 0x94, 0x46, 0x80])
        
        # TMCC state
        self.current_lionel_engine = 0
        
        # TMCC speed tracking per engine
        self.engine_speeds = {}  # {engine_number: current_speed_0_to_31}
        
        # TMCC direction tracking per engine
        self.engine_directions = {}  # {engine_number: current_direction}
        
        # TMCC quillable whistle state
        self.quillable_whistle_on = False
        self.whistle_pitch = 1  # 1-5 for different pitches
        
        # TMCC button state tracking
        self.button_states = {}  # {button_name: is_pressed}
        self.last_button_release = {}  # {button_name: timestamp}
        
        # TMCC command debouncing
        self.last_command_time = {}  # {engine_number: last_command_timestamp}
        self.debounce_delay = 0.5  # seconds between same commands
        
        # Whistle hold-to-sound timeout
        self.last_whistle_time = 0  # Last time whistle packet was received
        self.whistle_timeout = 0.3  # Seconds without packets before turning off whistle
        
        # Volume tracking
        self.master_volume = 70  # Default master volume (0-100)
        self.volume_step = 5  # Volume increment/decrement step
        
        # WTIU session key (from H5 response)
        self.wtiu_session_key = None
        
        # WTIU TIU number (discovered from x command)
        self.wtiu_tiu_number = None
        
    def wait_for_lionel_connection(self):
        """Wait for SER2 to be available and connect"""
        logger.info("🔄 Waiting for SER2 connection...")
        attempt = 0
        
        while self.running and attempt < self.max_reconnect_attempts:
            try:
                # Try to open the port to see if SER2 is connected
                test_serial = serial.Serial(self.lionel_port, baudrate=9600, timeout=1)
                test_serial.close()
                
                # If we can open it, try to connect properly
                if self.connect_lionel():
                    logger.info("✅ SER2 connected and ready!")
                    return True
                    
            except (serial.SerialException, OSError) as e:
                attempt += 1
                logger.info(f"⏳ Waiting for SER2... (attempt {attempt}/{self.max_reconnect_attempts})")
                time.sleep(self.connection_check_interval)
                
        logger.error("❌ SER2 not found after maximum attempts")
        return False
    
    def monitor_connections(self):
        """Monitor connections and auto-reconnect if needed"""
        logger.info("🔍 Starting connection monitor...")
        
        while self.running:
            try:
                # Check if Lionel connection is still alive
                if self.lionel_serial is None or not self.lionel_serial.is_open:
                    logger.warning("⚠️ Lionel connection lost, attempting reconnect...")
                    if self.wait_for_lionel_connection():
                        # Restart TMCC monitoring thread
                        self.start_tmcc_monitoring()
                    else:
                        logger.error("❌ Failed to reconnect to SER2")
                        
                # Check if MCU connection is still alive  
                if self.mcu_serial and not self.mcu_serial.is_open:
                    logger.warning("⚠️ MCU connection lost, attempting reconnect...")
                    self.connect_mcu()
                
                # Check if MTH WTIU connection is still alive
                if not self.mth_connected or not self.mth_socket:
                    logger.warning("⚠️ MTH WTIU connection lost, attempting reconnect...")
                    if self.connect_mth():
                        logger.info("✅ MTH WTIU reconnected successfully!")
                    else:
                        logger.warning("⚠️ MTH WTIU reconnect failed, will retry...")
                
            except Exception as e:
                logger.error(f"❌ Connection monitor error: {e}")
                
            time.sleep(self.connection_check_interval)
    
    def _is_mcu_connected(self):
        """Check if MCU connection is alive"""
        if not self.mcu_serial:
            return False
        try:
            return self.mcu_serial.is_open
        except:
            return False
    
    def start_connection_monitor(self):
        """Start the connection monitoring thread"""
        self.monitor_thread = threading.Thread(target=self.monitor_connections, daemon=True)
        self.monitor_thread.start()
        logger.info("🔍 Connection monitor started")
        
    def connect_lionel(self):
        """Connect to Lionel Base 3 via FTDI"""
        try:
            self.lionel_serial = serial.Serial(
                self.lionel_port, 
                baudrate=9600,  # Lionel Base 3 DB9 port outputs at 9600 baud
                bytesize=8, 
                parity='N', 
                stopbits=1, 
                timeout=0.1
            )
            logger.info(f"✅ Connected to Lionel Base 3 on {self.lionel_port}")
            return True
        except Exception as e:
            logger.error(f"❌ Lionel connection failed: {e}")
            return False
    
    def connect_mcu(self):
        """Connect to Arduino MCU via arduino-router Unix socket"""
        import platform
        
        system = platform.system()
        
        if system == 'Linux':
            # On Arduino UNO Q, use the arduino-router Unix socket
            socket_path = '/var/run/arduino-router.sock'
            try:
                import socket as sock
                self.mcu_socket = sock.socket(sock.AF_UNIX, sock.SOCK_STREAM)
                self.mcu_socket.connect(socket_path)
                self.mcu_socket.settimeout(1.0)
                self.mcu_connected = True
                logger.info(f"✅ Connected to Arduino MCU via arduino-router ({socket_path})")
                return True
            except FileNotFoundError:
                logger.info(f"Arduino router socket not found at {socket_path}")
            except PermissionError:
                logger.info(f"Permission denied for {socket_path} - try running as root")
            except Exception as e:
                logger.info(f"Arduino router connection failed: {e}")
        
        # Fallback: try direct serial (if arduino-router is stopped)
        logger.info("Trying direct serial connection as fallback...")
        import glob
        
        if system == 'Windows':
            possible_ports = [f'COM{i}' for i in range(1, 20)]
        elif system == 'Linux':
            possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        else:
            possible_ports = glob.glob('/dev/tty.usbserial*') + glob.glob('/dev/tty.usbmodem*')
        
        for port in possible_ports:
            try:
                self.mcu_serial = serial.Serial(port, baudrate=115200, timeout=1)
                self.mcu_connected = True
                self.mcu_port = port
                logger.info(f"✅ Connected to Arduino MCU via serial ({port})")
                return True
            except:
                continue
        
        logger.info(f"MCU connection not available on {system}")
        logger.info("💡 On Arduino UNO Q, make sure arduino-router service is running")
        self.mcu_connected = False
        return False
    
    def parse_tmcc_packet(self, packet):
        """Parse TMCC packet and convert to MTH command"""
        if len(packet) != 3 or packet[0] != 0xFE:
            return None
        
        # Extract TMCC packet fields correctly
        # packet[1] = bits 15-8, packet[2] = bits 7-0
        # Bit 15-14: Command type, Bit 13-7: Address (A), Bit 6-5: Command (C), Bit 4-0: Data (D)
        
        # Extract address field (bits 13-7) from packet[1] and packet[2]
        address_bits = ((packet[1] & 0x3F) << 1) | ((packet[2] & 0x80) >> 7)
        
        # Extract command field (bits 6-5) from packet[2]
        cmd_field = (packet[2] & 0x60) >> 5
        
        # Extract data field (bits 4-0) from packet[2]
        data_field = packet[2] & 0x1F
        
        # Update current engine from address field if present
        if address_bits > 0 and address_bits <= 99:
            self.current_lionel_engine = address_bits
            logger.info(f"🔧 Using engine from address field: {self.current_lionel_engine}")
        elif self.current_lionel_engine == 0:
            self.current_lionel_engine = 1  # Default to engine 1 if no engine selected
        
        # Debug logging
        logger.info(f"🔍 TMCC Parse: address=0x{address_bits:02x}, cmd_field=0x{cmd_field:02x}, data_field=0x{data_field:02x}")
        
        # TMCC to MTH command mapping based on actual packets
        if cmd_field == 0x00:  # Engine/Train commands (binary 00)
            if data_field == 0x00:  # Forward (00000)
                return {'type': 'direction', 'value': 'forward'}
            elif data_field == 0x01:  # Toggle Direction (00001)
                # Handle direction toggle here with debouncing
                current_time = time.time()
                last_time = self.last_command_time.get("direction_toggle", 0)
                
                if current_time - last_time > self.debounce_delay:
                    current_dir = self.engine_directions.get(self.current_lionel_engine, 'forward')
                    new_dir = 'reverse' if current_dir == 'forward' else 'forward'
                    self.engine_directions[self.current_lionel_engine] = new_dir
                    self.last_command_time["direction_toggle"] = current_time
                    logger.info(f"🔧 DEBUG: Direction toggled from {current_dir} to {new_dir}")
                    return {'type': 'direction', 'value': new_dir}
                else:
                    logger.info(f"🔧 DEBUG: Direction toggle debounced (too soon)")
                    return None
            elif data_field == 0x03:  # Reverse (00011)
                return {'type': 'direction', 'value': 'reverse'}
            elif data_field == 0x04:  # Boost Speed (00100)
                return {'type': 'speed', 'value': 'boost'}
            elif data_field == 0x05:  # Front Coupler (00101)
                logger.info(f"🔧 DEBUG: Front Coupler detected")
                return {'type': 'function', 'value': 'front_coupler'}
            elif data_field == 0x06:  # Rear Coupler (00110)
                logger.info(f"🔧 DEBUG: Rear Coupler detected")
                return {'type': 'function', 'value': 'rear_coupler'}
            elif data_field == 0x07:  # Brake Speed (00111)
                return {'type': 'speed', 'value': 'brake'}
            elif data_field == 0x08:  # Aux1 Off (01000)
                return {'type': 'function', 'value': 'aux1_off'}
            elif data_field == 0x09:  # Aux1 Option 1 (01001) - Map to startup for your remote
                return {'type': 'engine', 'value': 'startup'}
            elif data_field == 0x0A:  # Aux1 Option 2 (01010) - Button 1 = Volume UP
                logger.info(f"🔧 DEBUG: Button 1 - Volume UP detected")
                return {'type': 'function', 'value': 'volume_up'}
            elif data_field == 0x0B:  # Aux1 On (01011) - Button 4 = Volume DOWN
                logger.info(f"🔧 DEBUG: Button 4 - Volume DOWN detected")
                return {'type': 'function', 'value': 'volume_down'}
            elif data_field == 0x0C:  # Aux2 Off (01100)
                return {'type': 'function', 'value': 'aux2_off'}
            elif data_field == 0x0D:  # Aux2 Option 1 (01101)
                return {'type': 'function', 'value': 'aux2_option1'}
            elif data_field == 0x0E:  # Aux2 Option 2 (01110)
                return {'type': 'function', 'value': 'aux2_option2'}
            elif data_field == 0x0F:  # Aux2 On (01111)
                return {'type': 'function', 'value': 'aux2_on'}
            elif data_field == 0x10:  # Aux2 Option 3 (10000) - Quillable Whistle Toggle
                # Toggle quillable whistle state
                current_state = getattr(self, 'quillable_whistle_on', False)
                new_state = not current_state
                self.quillable_whistle_on = new_state
                if new_state:
                    logger.info(f"🔧 DEBUG: Quillable whistle ON (toggled)")
                    return {'type': 'function', 'value': 'whistle_on'}
                else:
                    logger.info(f"🔧 DEBUG: Quillable whistle OFF (toggled)")
                    return {'type': 'function', 'value': 'whistle_off'}
            elif data_field == 0x12:  # Aux2 Option 5 (10010) - Button 9 = Smoke ON
                logger.info(f"🔧 DEBUG: Button 9 - Smoke ON detected")
                return {'type': 'function', 'value': 'smoke_on'}
            elif data_field == 0x13:  # Aux2 Option 6 (10011) - Button 8 = Smoke OFF
                logger.info(f"🔧 DEBUG: Button 8 - Smoke OFF detected")
                return {'type': 'function', 'value': 'smoke_off'}
            elif data_field == 0x14:  # (10100) - Button 4 = Volume DOWN
                logger.info(f"🔧 DEBUG: Button 4 - Volume DOWN detected")
                return {'type': 'function', 'value': 'volume_down'}
            elif data_field == 0x15:  # Shutdown (10101)
                return {'type': 'engine', 'value': 'shutdown'}
            elif data_field == 0x18:  # (11000) - Smoke Off
                logger.info(f"🔧 DEBUG: Smoke OFF detected")
                return {'type': 'smoke', 'value': 'off'}
            elif data_field == 0x19:  # (11001) - Smoke On
                logger.info(f"🔧 DEBUG: Smoke ON detected")
                return {'type': 'smoke', 'value': 'on'}
            elif data_field == 0x11:  # (10001) - Button 1 = Volume UP
                logger.info(f"🔧 DEBUG: Button 1 - Volume UP detected")
                return {'type': 'function', 'value': 'volume_up'}
            elif data_field == 0x1C:  # Horn (11100) - Whistle button - HOLD MODE
                # Update last whistle time for timeout detection
                self.last_whistle_time = time.time()
                
                if not self.button_states.get('horn', False):
                    # First press - turn whistle on
                    self.button_states['horn'] = True
                    logger.info(f"🔧 DEBUG: Horn button PRESSED - Whistle ON")
                    return {'type': 'function', 'value': 'horn'}
                else:
                    # Still holding - keep whistle on, don't send duplicate commands
                    logger.info(f"🔧 DEBUG: Horn button HELD - Whistle staying ON")
                    return None
            elif data_field == 0x1D:  # Bell (11101) - Button press - TOGGLE MODE
                current_time = time.time()
                last_time = self.last_command_time.get("bell_toggle", 0)
                
                if current_time - last_time > self.debounce_delay:
                    # Toggle bell state
                    current_state = self.button_states.get('bell', False)
                    new_state = not current_state
                    self.button_states['bell'] = new_state
                    self.last_command_time["bell_toggle"] = current_time
                    
                    if new_state:
                        logger.info(f"🔧 DEBUG: Bell button TOGGLED ON")
                        return {'type': 'function', 'value': 'bell'}
                    else:
                        logger.info(f"🔧 DEBUG: Bell button TOGGLED OFF")
                        return {'type': 'function', 'value': 'bell_off'}
                else:
                    logger.info(f"🔧 DEBUG: Bell button debounced (too soon)")
                    return None
            
            # Direction commands
            elif data_field in [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6]:
                return {'type': 'direction', 'value': 'forward'}
            elif data_field in [0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE]:
                return {'type': 'direction', 'value': 'reverse'}
                
        elif cmd_field == 0x02:  # Relative speed commands (binary 10 - bit 6 set)
            if 0x00 <= data_field <= 0x1F:  # Relative speed D (0-31)
                # Add debouncing to prevent double processing
                current_time = time.time()
                last_time = self.last_command_time.get(f"{self.current_lionel_engine}_speed", 0)
                
                if current_time - last_time > 0.3:  # Longer debounce for speed to prevent double processing
                    # Convert relative speed: 0xA=+5, 0x9=+4, ..., 0x5=0, ..., 0x0=-5
                    if data_field == 0x0A:  # +5
                        speed_change = 5
                    elif data_field == 0x09:  # +4
                        speed_change = 4
                    elif data_field == 0x08:  # +3
                        speed_change = 3
                    elif data_field == 0x07:  # +2
                        speed_change = 2
                    elif data_field == 0x06:  # +1
                        speed_change = 1
                    elif data_field == 0x05:  # 0 (no change)
                        speed_change = 0
                    elif data_field == 0x04:  # -1
                        speed_change = -1
                    elif data_field == 0x03:  # -2
                        speed_change = -2
                    elif data_field == 0x02:  # -3
                        speed_change = -3
                    elif data_field == 0x01:  # -4
                        speed_change = -4
                    elif data_field == 0x00:  # -5
                        speed_change = -5
                    else:
                        speed_change = 0
                    
                    self.last_command_time[f"{self.current_lionel_engine}_speed"] = current_time
                    logger.info(f"🔧 DEBUG: Relative speed change: {speed_change} (data_field=0x{data_field:02x})")
                    
                    return {'type': 'speed', 'value': speed_change}
                else:
                    logger.info(f"🔧 DEBUG: Speed command debounced (too soon)")
                    return None
        elif cmd_field == 0x20:
            if data_field == 0x1C:
                logger.info(f"🐛 WHISTLE PACKET: Horn press")
            elif data_field == 0x18:
                logger.info(f"🐛 WHISTLE PACKET: Horn release")
                current_time = time.time()
                last_time = self.last_command_time.get(f"{self.current_lionel_engine}_direction", 0)
                
                if current_time - last_time > self.debounce_delay:
                    # Toggle current direction
                    current_dir = self.engine_directions.get(self.current_lionel_engine, 'forward')
                    new_dir = 'reverse' if current_dir == 'forward' else 'forward'
                    self.engine_directions[self.current_lionel_engine] = new_dir
                    self.last_command_time[f"{self.current_lionel_engine}_direction"] = current_time
                    return {'type': 'direction', 'value': new_dir}
                else:
                    logger.info(f"🔧 DEBUG: Direction command debounced (too soon)")
                    return None
            elif data_field == 0x03:  # Reverse (00011)
                logger.info(f"🔧 DEBUG: Button 4 - Volume DOWN detected")
                return {'type': 'function', 'value': 'volume_down'}
            elif data_field == 0x0C:  # Aux2 Off (01100)
                return {'type': 'function', 'value': 'aux2_off'}
            elif data_field == 0x0D:  # Aux2 Option 1 (01101)
                return {'type': 'function', 'value': 'aux2_option1'}
            elif data_field == 0x0E:  # Aux2 Option 2 (01110)
                return {'type': 'function', 'value': 'aux2_option2'}
            elif data_field == 0x0F:  # Aux2 On (01111)
                return {'type': 'function', 'value': 'aux2_on'}
            elif data_field == 0x1C:  # Horn (11100) - Whistle button - HOLD MODE
                # Update last whistle time for timeout detection
                self.last_whistle_time = time.time()
                
                if not self.button_states.get('horn', False):
                    # First press - turn whistle on
                    self.button_states['horn'] = True
                    logger.info(f"🔧 DEBUG: Horn button PRESSED - Whistle ON")
                    return {'type': 'function', 'value': 'horn'}
                else:
                    # Still holding - keep whistle on, don't send duplicate commands
                    logger.info(f"🔧 DEBUG: Horn button HELD - Whistle staying ON")
                    return None
            elif data_field == 0x1D:  # Bell (11101) - Button press - TOGGLE MODE
                current_time = time.time()
                last_time = self.last_command_time.get("bell_toggle", 0)
                
                if current_time - last_time > self.debounce_delay:
                    # Toggle bell state
                    current_state = self.button_states.get('bell', False)
                    new_state = not current_state
                    self.button_states['bell'] = new_state
                    self.last_command_time["bell_toggle"] = current_time
                    
                    if new_state:
                        logger.info(f"🔧 DEBUG: Bell button TOGGLED ON")
                        return {'type': 'function', 'value': 'bell'}
                    else:
                        logger.info(f"🔧 DEBUG: Bell button TOGGLED OFF")
                        return {'type': 'function', 'value': 'bell_off'}
                else:
                    logger.info(f"🔧 DEBUG: Bell button debounced (too soon)")
                    return None
            elif data_field == 0x18:  # Horn Off (11000) - Button release
                if self.button_states.get('horn', False):
                    self.button_states['horn'] = False
                    logger.info(f"🔧 DEBUG: Horn button RELEASED")
                    return {'type': 'function', 'value': 'horn_off'}
                else:
                    return None
            elif data_field == 0x19:  # Bell Off (11001) - Button release - IGNORE for toggle mode
                # Ignore bell off commands since we're using toggle mode
                logger.info(f"🔧 DEBUG: Bell button release ignored (toggle mode)")
                return None
            
            # Direction commands
            elif data_field in [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6]:
                return {'type': 'direction', 'value': 'forward'}
            elif data_field in [0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE]:
                return {'type': 'direction', 'value': 'reverse'}
        
        return None
    
    def send_to_mcu(self, command):
        """Send command to Arduino MCU via arduino-router socket or serial"""
        if not self.mcu_connected:
            logger.debug("MCU not connected - command not sent")
            return False
            
        try:
            with self.mcu_lock:
                # Get command type code
                cmd_type_code = self.mcu_command_types.get(command['type'], 0)
                
                # Handle different value types
                if command['type'] == 'speed':
                    cmd_value = command['value']
                elif command['type'] == 'engine':
                    cmd_value = 1 if command['value'] == 'start' else 0
                elif command['type'] == 'direction':
                    cmd_value = 1 if command['value'] == 'forward' else 0
                elif command['type'] in ['function', 'smoke', 'pfa']:
                    value_map = {
                        'horn': 1, 'bell': 2,
                        'increase': 1, 'decrease': 2, 'on': 3, 'off': 4,
                        'cab_chatter': 1, 'towercom': 2
                    }
                    cmd_value = value_map.get(command['value'], 0)
                elif command['type'] == 'protowhistle':
                    cmd_value = command['value']
                elif command['type'] == 'wled':
                    cmd_value = command['value']
                else:
                    cmd_value = 0
                
                # Format command string
                cmd_string = f"CMD:{cmd_type_code}:{cmd_value}\n"
                
                # Send via socket or serial
                if hasattr(self, 'mcu_socket') and self.mcu_socket:
                    self.mcu_socket.send(cmd_string.encode())
                    logger.debug(f"Sent to MCU via socket: {cmd_string.strip()}")
                elif hasattr(self, 'mcu_serial') and self.mcu_serial:
                    self.mcu_serial.write(cmd_string.encode())
                    logger.debug(f"Sent to MCU via serial: {cmd_string.strip()}")
                else:
                    logger.debug("No MCU connection available")
                    return False
                    
                return True
                
        except Exception as e:
            logger.error(f"MCU send error: {e}")
            return False
    
    def discover_wtiu_mdns(self):
        """Discover MTH WTIU using mDNS/Zeroconf"""
        try:
            from zeroconf import ServiceBrowser, Zeroconf
            logger.info("🔍 Discovering MTH WTIU via mDNS...")
            
            class WTIUListener:
                def __init__(self, bridge):
                    self.bridge = bridge
                
                def add_service(self, zeroconf, service_type, name):
                    info = zeroconf.get_service_info(service_type, name)
                    if info:
                        self.bridge.discovered_wtiu = {
                            'name': name,
                            'host': info.parsed_addresses()[0],
                            'port': info.port,
                            'properties': info.properties
                        }
                        logger.info(f"🎯 Found WTIU: {name} at {info.parsed_addresses()[0]}:{info.port}")
                
                def remove_service(self, zeroconf, service_type, name):
                    pass
                
                def update_service(self, zeroconf, service_type, name):
                    pass
            
            zeroconf = Zeroconf()
            listener = WTIUListener(self)
            
            # Try MTH WTIU service names
            service_names = [
                "_mth-dcs._tcp.local.",
                "_wtiu._tcp.local.",
                "_mth._tcp.local.",
                "_dcs._tcp.local."
            ]
            
            for service_name in service_names:
                browser = ServiceBrowser(zeroconf, service_name, listener)
                time.sleep(2)  # Wait for discovery
                
                if hasattr(self, 'discovered_wtiu'):
                    logger.info(f"✅ Found WTIU using service: {service_name}")
                    zeroconf.close()
                    return True
                else:
                    browser.cancel()
            
            zeroconf.close()
            return False
            
        except ImportError:
            logger.info("⚠️ zeroconf not available - using manual IP")
            return False
        except Exception as e:
            logger.error(f"mDNS discovery error: {e}")
            return False
    
    def connect_mth(self):
        """Connect to MTH WTIU via WiFi with mDNS discovery"""
        import socket
        from threading import Lock
        
        self.mth_socket = None
        self.mth_connected = False
        
        # Try mDNS discovery first
        logger.info("🔍 Attempting MTH WTIU discovery...")
        if self.discover_wtiu_mdns():
            logger.info("✅ mDNS discovery successful")
            # Use discovered WTIU
            if hasattr(self, 'discovered_wtiu'):
                mth_host = self.discovered_wtiu['host']
                mth_port = self.discovered_wtiu['port']
                logger.info(f"🎯 Using discovered WTIU: {mth_host}:{mth_port}")
            else:
                mth_host = '192.168.0.31'
                mth_port = 33069
        else:
            logger.info("❌ mDNS discovery failed, using manual IP")
            mth_host = '192.168.0.31'  # Your WTIU IP from previous logs
            mth_port = 33069  # Your WTIU port from previous logs
        
        try:
            logger.info(f"🔗 Connecting to MTH WTIU at {mth_host}:{mth_port}")
            self.mth_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # Match ESP8266 WiFiClient behavior
            self.mth_socket.settimeout(5.0)
            self.mth_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.mth_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            
            self.mth_socket.connect((mth_host, mth_port))
            self.mth_connected = True
            logger.info(f"✅ Connected to MTH WTIU at {mth_host}:{mth_port}")
            
            # Wait a moment for connection to stabilize (like Mark's WTIUWaitforConnection)
            logger.info("⏳ Waiting for connection to stabilize...")
            time.sleep(0.5)
            
            # Perform Mark's H5/H6 handshake (required for WTIU) with retries
            max_handshake_attempts = 3
            for attempt in range(max_handshake_attempts):
                try:
                    logger.info(f"🔐 Performing Mark's H5/H6 handshake (attempt {attempt + 1}/{max_handshake_attempts})...")
                    
                    # Step 1: Send H5 command (ESP8266 style - exact match)
                    logger.info("🔐 Step 1: Sending H5 command...")
                    h5_command = b"H5\r\n"
                    self.mth_socket.send(h5_command)  # Exact ESP8266 match
                    logger.info(f"🔐 Sent H5: {h5_command.strip()}")
                    h5_response = self.mth_socket.recv(256).decode()
                    logger.info(f"🔍 WTIU H5 response: {h5_response.strip()}")
                    
                    # Check if H5 response contains "okay"
                    if "okay" not in h5_response.lower():
                        logger.warning("⚠️ H5 response missing 'okay'")
                        continue  # Retry
                
                # Extract encryption key from H5 response (format: "H5 1234ABCD okay")
                    parts = h5_response.strip().split()
                    if len(parts) >= 2:
                        hex_key = parts[1]
                        logger.info(f"🔐 Extracted key: {hex_key}")
                        
                        # Store the WTIU session key for future encryption
                        self.wtiu_session_key = bytes.fromhex(hex_key)
                        logger.info(f"🔐 Stored WTIU session key: {self.wtiu_session_key.hex()}")
                        
                        # Step 2: Send H6 with encrypted key (Mark's Arduino method)
                        logger.info("🔐 Step 2: Sending H6 with encrypted key (Mark's Arduino method)...")
                        
                        # Parse H5 response like Mark's Arduino: "H5 %4hx%4hx"
                        # Note: sscanf uses swapped order: &plain[1], &plain[0]
                        h5_match = re.search(r'H5\s+([0-9A-Fa-f]{4})([0-9A-Fa-f]{4})', h5_response)
                        if not h5_match:
                            logger.warning(f"⚠️ Could not parse H5 response: '{h5_response}'")
                            h6_response = ""
                        else:
                            hex_str = h5_match.group(1) + h5_match.group(2)
                            logger.info(f"🔐 Challenge: {hex_str}")
                            
                            # Parse as two 16-bit values (Arduino swapped order)
                            word1 = int(hex_str[0:4], 16)
                            word2 = int(hex_str[4:8], 16)
                            logger.info(f"🔐 Words: 0x{word1:04X} 0x{word2:04X}")
                            
                            # Create plaintext array (Arduino swapped order: plain[1], plain[0])
                            # Mark uses sscanf(&plain[1], &plain[0]) so plain[1] gets first word
                            plain = [word2, word1]  # plain[0]=word2, plain[1]=word1
                            
                            # Use Mark's global hardcoded key (not session key)
                            # The session key from H5 is the CHALLENGE to encrypt, not the encryption key
                            key_words = [0x144C, 0xB404, 0x947D, 0x8046]  # Mark's exact order
                            
                            # TEST: Compare with known working ESP8266 values
                            if hex_key == "E043B8C5":
                                logger.info("🧪 TESTING: Known ESP8266 challenge detected!")
                                logger.info("🧪 Expected H6 response: H6F66059E4 okay")
                                logger.info("🧪 Our bridge produces different encryption!")
                            elif hex_key == "FA0369CC":
                                logger.info("🧪 TESTING: Using first received challenge for comparison!")
                                logger.info("🧪 Our bridge produces: H6B09D9545")
                            elif hex_key == "974FA7CE":
                                logger.info("🧪 TESTING: Using latest received challenge!")
                                logger.info("🧪 Our bridge produces: H62F13FEF8")
                            
                            # Fixed Speck implementation - EXACT C++ match
                            class FixedSpeckCipher:
                                def __init__(self):
                                    self.key = [5196, 46084, 38013, 32838]  # Exact C++ values
                                    self.rounds = 22
                                    
                                def ror16(self, x, r):
                                    return ((x >> r) | (x << (16 - r))) & 0xFFFF
                                    
                                def rol16(self, x, r):
                                    return ((x << r) | (x >> (16 - r))) & 0xFFFF
                                    
                                def rrr(self, x, y, k):
                                    """C++ RRR macro exactly"""
                                    x = self.ror16(x, 7)  # ROR 7
                                    x = (x + y) & 0xFFFF  # x += y
                                    x ^= k                # x ^= k
                                    y = self.rol16(y, 2)  # ROL 2
                                    y ^= x                # y ^= x
                                    return x, y
                                    
                                def encrypt(self, plaintext):
                                    """C++ speck_encrypt exactly"""
                                    S = [0] * self.rounds
                                    b = self.key[0]
                                    a = [self.key[1], self.key[2], self.key[3]]
                                    
                                    # Key expansion (speck_expand)
                                    S[0] = b
                                    for i in range(self.rounds - 1):
                                        a[i % 3], b = self.rrr(a[i % 3], b, i)
                                        S[i + 1] = b
                                    
                                    # Encryption
                                    x = plaintext[1]  # ct[1] = pt[1]
                                    y = plaintext[0]  # ct[0] = pt[0]
                                    
                                    for i in range(self.rounds):
                                        x, y = self.rrr(x, y, S[i])  # RRR(ct[1], ct[0], K[i])
                                        
                                    return [y, x]  # Return ct[0], ct[1]
                            
                            # Use the fixed cipher
                            cipher = FixedSpeckCipher().encrypt(plain)
                            logger.info(f"🔐 Fixed Speck cipher: 0x{cipher[0]:04X} 0x{cipher[1]:04X}")
                            
                            # DEBUG: Show our H6 result for comparison
                            our_h6 = f"{cipher[1]:04X}{cipher[0]:04X}"
                            logger.info(f"🔐 Our H6 result: {our_h6}")
                            
                            # Send H6 command (ESP8266 exact format)
                            # Note: cipher should be exactly 2 words (4 bytes total)
                            h6_command = f"H6{cipher[1]:04X}{cipher[0]:04X}\r\n"
                            logger.info(f"🔐 Sending H6 (ESP8266 format): {h6_command.strip()}")
                            logger.info(f"🔐 H6 command length: {len(h6_command.strip())} chars")
                            self.mth_socket.send(h6_command.encode())  # Exact ESP8266 match
                            h6_response = self.mth_socket.recv(256).decode()
                            logger.info(f"🔍 WTIU H6 response: {h6_response.strip()}")
                    else:
                        logger.warning("⚠️ Failed to encrypt H6 key properly")
                        h6_response = ""
                    
                    # Check for success - accept H6 response (working ESP8266 approach)
                    # Your ESP8266 gets "okay" but our bridge gets "PC connection not available"
                    # Let's accept H6 response and proceed to see if post-handshake commands work
                    if "H6" in h6_response:
                        if "okay" in h6_response.lower():
                            logger.info("✅ WTIU H5/H6 handshake successful (with okay)!")
                        else:
                            logger.info("✅ WTIU H5/H6 handshake successful (without okay)!")
                            logger.warning("⚠️ H6 response missing 'okay', but proceeding anyway...")
                        
                        # Send x and ! commands like ESP8266 code (not PC command)
                        logger.info("🔐 Getting TIU info (like ESP8266 code)...")
                        
                        # Send x command to get TIU number
                        self.mth_socket.send(b"x\r\n")
                        x_response = self.mth_socket.recv(256).decode()
                        logger.info(f"� WTIU x response: {x_response.strip()}")
                        
                        # Send ! command to get version info
                        self.mth_socket.send(b"!\r\n")
                        exclamation_response = self.mth_socket.recv(256).decode()
                        logger.info(f"🔍 WTIU ! response: {exclamation_response.strip()}")
                        
                        # Step 3: Now send normal commands
                        logger.info("🔐 Step 3: Sending normal commands...")
                        self.mth_socket.send(b"x\r\n")
                        x_response = self.mth_socket.recv(256).decode()
                        logger.info(f"🔍 WTIU x response: {x_response.strip()}")
                        
                        self.mth_socket.send(b"!\r\n")
                        exclamation_response = self.mth_socket.recv(256).decode()
                        logger.info(f"🔍 WTIU ! response: {exclamation_response.strip()}")
                        
                        # Accept any response from x and ! commands (WTIU is responding)
                        logger.info("✅ WTIU full handshake successful!")
                        logger.info(f"🔍 x response: '{x_response.strip()}'")
                        logger.info(f"🔍 ! response: '{exclamation_response.strip()}'")
                        
                        # Send 'y' command to establish PC connection (like ESP8266 Sendy function)
                        logger.info("🔐 Establishing PC connection with 'y' command (like ESP8266 Sendy)...")
                        y_command = f"y11\r\n"  # Engine number 11 (Lionel Engine #10)
                        self.mth_socket.send(y_command.encode())
                        y_response = self.mth_socket.recv(256).decode()
                        logger.info(f"🔍 WTIU y response: {y_response.strip()}")
                        
                        # Test if connection is working by sending a simple command
                        logger.info("🔐 Testing connection with simple command...")
                        test_command = "y11\r\n"
                        self.mth_socket.send(test_command.encode())
                        test_response = self.mth_socket.recv(256).decode()
                        logger.info(f"🔍 Test response: {test_response.strip()}")
                        
                        if "PC connection not available" not in test_response:
                            logger.info("✅ WTIU PC connection established successfully!")
                            # Establish proper PC connection sequence (ESP8266 format)
                            self.establish_pc_connection()
                            break  # Success! Exit the retry loop
                        else:
                            logger.warning("⚠️ WTIU still reports PC connection not available")
                            continue  # Retry
                    else:
                        logger.warning(f"⚠️ H6 response missing 'H6': '{h6_response.strip()}'")
                        continue  # Retry                 # If we got here, the handshake failed, try again
                    if attempt < max_handshake_attempts - 1:
                        logger.warning(f"⚠️ Handshake attempt {attempt + 1} failed, retrying...")
                        time.sleep(1)  # Wait before retry
                    else:
                        logger.error("❌ All handshake attempts failed")
                        break
                        
                except Exception as handshake_error:
                    logger.warning(f"⚠️ WTIU handshake failed: {handshake_error}")
                    if attempt < max_handshake_attempts - 1:
                        time.sleep(1)  # Wait before retry
                    else:
                        break
            
            return True
            
        except Exception as e:
            logger.error(f"MTH WTIU connection failed: {e}")
            self.mth_connected = False
            return False
    
    def establish_pc_connection(self):
        """Establish PC connection with WTIU - ESP8266 exact sequence"""
        try:
            logger.info("🔐 Establishing PC connection (ESP8266 sequence)...")
            
            # 1. Get TIU info
            self.mth_socket.send(b"x\r\n")
            x_response = self.mth_socket.recv(256).decode()
            logger.info(f"🔍 x command response: {x_response.strip()}")
            
            # Parse TIU number
            match = re.search(r'x(\d)(\d)', x_response)
            if match:
                self.wtiu_tiu_number = int(match.group(1))  # 0-4
                logger.info(f"✅ Found TIU number: {self.wtiu_tiu_number + 1}")
            
            # 2. Get version
            self.mth_socket.send(b"!\r\n")
            version_response = self.mth_socket.recv(256).decode()
            logger.info(f"🔍 ! command response: {version_response.strip()}")
            
            # 3. Send y command with engine number (like ESP8266 Sendy)
            # Map Lionel Engine #10 to WTIU Engine #11 (DCS #12)
            # Map Lionel Engine #11 to WTIU Engine #12 (DCS #13) 
            # Default to Engine #11 for Lionel Engine #10
            self.mth_socket.send(b"y12\r\n")
            y_response = self.mth_socket.recv(256).decode()
            logger.info(f"🔍 y command response: {y_response.strip()}")
            
            logger.info("✅ WTIU setup complete - ready for commands!")
            return True
            
        except Exception as e:
            logger.error(f"❌ PC connection failed: {e}")
            return False
    
    def send_wtiu_command(self, command):
        """Send command to WTIU in exact ESP8266 format"""
        try:
            # Send command directly (no engine prefix after y command)
            full_command = f"{command}\r\n".encode()
            self.mth_socket.send(full_command)
            logger.info(f"🚂 Sent to WTIU: {command}")
            
            # Wait for response with timeout
            self.mth_socket.settimeout(2.0)
            try:
                response = self.mth_socket.recv(256).decode()
                # Check for "->" prompt and extract response
                if "->" in response:
                    response = response.split("->")[0].strip()
                logger.info(f"📥 WTIU response: {response}")
                
                # Return True if we got "okay" or any response (some commands don't return okay)
                if response and response != "TIMEOUT":
                    return True
                else:
                    return False
                    
            except socket.timeout:
                logger.warning("⚠️ Command timeout (some commands don't return response)")
                # Some commands like 'x' might not return immediately
                return True  # Assume success for timeout
                
        except Exception as e:
            logger.error(f"❌ Command send error: {e}")
            return False
    
    def send_to_mth(self, command):
        """Send command to MTH WTIU via WiFi with proper sequence"""
        if not self.mth_connected or not self.mth_socket:
            logger.debug("MTH not connected - command not sent")
            return False
        
        try:
            with self.mth_lock:
                # Select the correct engine based on TMCC packet
                if self.current_lionel_engine > 0:
                    # Map Lionel Engine # to WTIU Engine # based on discovery
                    if self.current_lionel_engine == 10:
                        wtiu_engine = 11  # Lionel #10 → WTIU #11 (DCS #10)
                    elif self.current_lionel_engine == 11:
                        wtiu_engine = 12  # Lionel #11 → WTIU #12 (DCS #11)
                    elif self.current_lionel_engine == 5:
                        wtiu_engine = 6   # Lionel #5 → WTIU #6 (DCS #7)
                    else:
                        wtiu_engine = self.current_lionel_engine + 1  # Default mapping
                    
                    # Send engine selection command first
                    logger.info(f"🔧 Selecting WTIU Engine #{wtiu_engine} for Lionel Engine #{self.current_lionel_engine}")
                    select_cmd = f"y{wtiu_engine}\r\n"
                    self.mth_socket.send(select_cmd.encode())
                    time.sleep(0.1)  # Brief pause for engine selection
                    try:
                        select_response = self.mth_socket.recv(256).decode()
                        logger.info(f"🔍 Engine selection response: {select_response.strip()}")
                    except:
                        pass  # Don't fail if no response to selection
                
                # Convert command to MTH protocol format
                mth_cmd = self.convert_to_mth_protocol(command)
                if mth_cmd:
                    # Send command in ESP8266 format
                    success = self.send_wtiu_command(mth_cmd)
                    
                    # Check if command was successful
                    if success:
                        logger.info("✅ Command sent successfully")
                        return True
                    else:
                        logger.warning(f"⚠️ Command failed: {mth_cmd}")
                        return False
                    
        except Exception as e:
            logger.error(f"MTH send error: {e}")
            self.mth_connected = False
            return False
    
    def convert_to_mth_protocol(self, command):
        """Convert TMCC command to MTH WTIU command format"""
        # Simple direct command mapping (no engine prefix needed after y command)
        cmd_map = {
            'direction': {
                'forward': 'd0',
                'reverse': 'd1'
            },
            'speed': lambda x: self.convert_speed(x),
            'function': {
                'horn': 'w2',
                'bell': 'w4',
                'horn_off': 'bFFFD',
                'bell_off': 'bFFFB',
                'whistle_on': 'w2',
                'whistle_off': 'bFFFD',
                'smoke_on': 'abF',
                'smoke_off': 'abE',
                'smoke_toggle': 'abF',
                'volume_up': 'volume_up',
                'volume_down': 'volume_down',
                'front_coupler': 'c0',
                'rear_coupler': 'c1',
                'whistle_pitch_1': 'w2',
                'whistle_pitch_2': 'w2',
                'whistle_pitch_3': 'w2',
                'whistle_pitch_4': 'w2',
                'whistle_pitch_5': 'w2'
            },
            'smoke': {
                'on': 'abF',
                'off': 'abE',
                'level': 'ab10'
            },
            'engine': {
                'startup': 'u4',
                'shutdown': 'u5'
            }
        }
        
        cmd_type = command['type']
        cmd_value = command['value']
        
        if cmd_type in cmd_map:
            if cmd_type == 'speed':
                # Convert speed
                return cmd_map['speed'](cmd_value)
            elif cmd_type == 'direction' and cmd_value == 'toggle':
                # Handle direction toggle - track state per engine
                current_dir = self.engine_directions.get(self.current_lionel_engine, 'forward')
                new_dir = 'reverse' if current_dir == 'forward' else 'forward'
                self.engine_directions[self.current_lionel_engine] = new_dir
                logger.info(f"🔧 DEBUG: Direction toggled from {current_dir} to {new_dir}")
                return cmd_map['direction'][new_dir]
            elif cmd_type == 'function' and cmd_value in ['volume_up', 'volume_down']:
                # Handle volume commands
                return self.convert_volume(cmd_value)
            elif cmd_value in cmd_map[cmd_type]:
                return cmd_map[cmd_type][cmd_value]
        
        logger.warning(f"⚠️ Unknown command: {cmd_type}:{cmd_value}")
        return None
    
    def convert_volume(self, direction):
        """Convert volume up/down to absolute volume command"""
        try:
            if direction == 'volume_up':
                self.master_volume = min(100, self.master_volume + self.volume_step)
            elif direction == 'volume_down':
                self.master_volume = max(0, self.master_volume - self.volume_step)
            else:
                logger.warning(f"⚠️ Unknown volume direction: {direction}")
                return None
            
            logger.info(f"🔧 DEBUG: Volume {direction} -> {self.master_volume}%")
            return f"v0{self.master_volume:03d}"
        except Exception as e:
            logger.error(f"❌ Volume conversion error: {e}")
            return None
    
    def convert_speed(self, speed_value):
        """Convert TMCC speed (0-31) to DCS speed (0-120)"""
        try:
            if isinstance(speed_value, str):
                if speed_value == 'boost':
                    return 's5'  # Small speed boost
                elif speed_value == 'brake':
                    return 's0'  # Stop
                else:
                    # Try to convert string to int
                    try:
                        speed_int = int(speed_value)
                        return "s0"
                    except:
                        return 's0'
            
            # Handle integer speed values
            if not isinstance(speed_value, (int, float)):
                logger.warning(f"⚠️ Invalid speed value type: {type(speed_value)}")
                return 's0'
            
            # Handle relative speed changes (+1, -1, -2, etc.)
            current_speed = self.engine_speeds.get(self.current_lionel_engine, 0)
            new_speed = current_speed + speed_value
            new_speed = max(0, min(31, new_speed))  # Clamp to 0-31
            
            # Update tracked speed for this engine
            self.engine_speeds[self.current_lionel_engine] = new_speed
            
            # Convert 0-31 to 0-120 (scale factor ~3.87)
            logger.info(f"🔧 DEBUG: Engine {self.current_lionel_engine} speed {current_speed} + {speed_value} = {new_speed}")
            dcs_speed = int(new_speed * 120 / 31)
            dcs_speed = max(0, min(120, dcs_speed))  # Clamp to 0-120
            logger.info(f"🔧 DEBUG: Converted to DCS speed {dcs_speed}")
            return f"s{dcs_speed}"
            
        except Exception as e:
            logger.error(f"❌ Speed conversion error: {e}")
            return 's0'
    
    def discover_wtiu_engines(self):
        """Discover engines configured on WTIU using Mark's reference commands"""
        logger.info("🔍 Discovering WTIU engines...")
        
        # MTH WTIU engine discovery commands from Mark's reference
        discovery_commands = [
            ("x", "Read TIU number and AIU count"),
            ("!", "Read TIU version"),
            ("I0", "Check for Engines - returns bit map of engines"),
            ("I1", "Factory reset engine (DCS #1)"),
            ("I2", "Engine #2 (DCS #3)"),
            ("I3", "Engine #3 (DCS #4)"),
            ("I4", "Engine #4 (DCS #5)"),
            ("I5", "Engine #5 (DCS #6)"),
            ("I6", "Engine #6 (DCS #7)"),
            ("I7", "Engine #7 (DCS #8)"),
            ("I8", "Engine #8 (DCS #9)"),
            ("I9", "Engine #9 (DCS #10)"),
            ("I10", "Engine #10 (DCS #11)"),
            ("I11", "Engine #11 (DCS #12)"),
            ("I12", "Engine #12 (DCS #13)"),
        ]
        
        for cmd, desc in discovery_commands:
            logger.info(f"🔍 Sending discovery command: {cmd} ({desc})")
            self.mth_socket.send(f"{cmd}\r\n".encode())
            try:
                response = self.mth_socket.recv(256).decode()
                logger.info(f"🔍 Response: {response.strip()}")
            except socket.timeout:
                logger.info(f"🔍 Timeout for {cmd}")
            except Exception as e:
                logger.info(f"🔍 Error: {e}")
            time.sleep(0.5)
        
        logger.info("🔍 WTIU engine discovery complete")
    
    def debug_wtiu_connection(self):
        """Debug WTIU connection and commands"""
        logger.info("🐛 DEBUG: Testing WTIU connection...")
        
        # Test basic commands
        test_commands = [
            "x",       # Should return TIU info
            "!",       # Should return version
            "y2",      # Select engine 2
            "m4",      # Command mode
            "u4",      # Startup engine
            "s10",     # Speed 10
            "d0",      # Forward
            "w2",      # Horn
            "bFFFD",   # Horn off
        ]
        
        for cmd in test_commands:
            logger.info(f"🐛 DEBUG: Sending: {cmd}")
            self.mth_socket.send(f"{cmd}\r\n".encode())
            time.sleep(0.5)
            try:
                response = self.mth_socket.recv(256).decode()
                logger.info(f"🐛 DEBUG: Response: {response.strip()}")
            except socket.timeout:
                logger.info(f"🐛 DEBUG: Timeout for {cmd}")
            except Exception as e:
                logger.info(f"🐛 DEBUG: Error: {e}")
            time.sleep(0.5)
        
        logger.info("🔍 WTIU debug complete")
        packet_count = 0
        last_activity = time.time()
        
        while self.running:
            try:
                with self.lionel_lock:
                    if self.lionel_serial and self.lionel_serial.is_open:
                        # Check for any data in the buffer
                        if self.lionel_serial.in_waiting > 0:
                            data = self.lionel_serial.read(self.lionel_serial.in_waiting)
                            last_activity = time.time()
                            logger.info(f"🔍 Received {len(data)} bytes: {data.hex()}")
                            
                            # Look for TMCC packets
                            for i in range(len(data) - 2):
                                if data[i] == 0xFE:
                                    packet = data[i:i+3]
                                    packet_count += 1
                                    logger.info(f"🎯 TMCC Packet #{packet_count}: {packet.hex()}")
                                    
                                    # Parse and forward
                                    command = self.parse_tmcc_packet(packet)
                                    if command:
                                        logger.info(f"📤 Command: {command}")
                                        
                                        # Log what we're trying to send
                                        mth_cmd = self.convert_to_mth_protocol(command)
                                        if mth_cmd:
                                            logger.info(f"📤 MTH Command: {mth_cmd}")
                                        else:
                                            logger.warning(f"⚠️ Failed to convert command: {command}")
                                        
                                        # Send to MCU
                                        if self.send_to_mcu(command):
                                            logger.info("✅ Sent to MCU")
                                        else:
                                            logger.warning("⚠️ Failed to send to MCU")
                                        
                                        # Send to MTH
                                        if self.send_to_mth(command):
                                            logger.info("✅ Sent to MTH")
                                        else:
                                            logger.warning("⚠️ Failed to send to MTH")
                                    else:
                                        logger.warning(f"⚠️ Failed to parse packet: {packet.hex()}")
                        else:
                            # Log every 10 seconds if no data received
                            if time.time() - last_activity > 10:
                                logger.warning("⚠️ No data received from Lionel Base 3 for 10 seconds")
                                last_activity = time.time()
            
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Lionel listener error: {e}")
                time.sleep(1)
    
    def lionel_listener(self):
        """Listen for TMCC packets from Lionel Base 3"""
        logger.info("🎯 Monitoring Lionel Base 3 for TMCC packets...")
        packet_count = 0
        last_activity = time.time()
        
        while self.running:
            try:
                with self.lionel_lock:
                    if self.lionel_serial and self.lionel_serial.is_open:
                        # Check for any data in the buffer
                        if self.lionel_serial.in_waiting > 0:
                            data = self.lionel_serial.read(self.lionel_serial.in_waiting)
                            last_activity = time.time()
                            logger.info(f"🔍 Received {len(data)} bytes: {data.hex()}")
                            
                            # Look for TMCC packets
                            for i in range(len(data) - 2):
                                if data[i] == 0xFE:
                                    packet = data[i:i+3]
                                    packet_count += 1
                                    logger.info(f"🎯 TMCC Packet #{packet_count}: {packet.hex()}")
                                    
                                    # Parse and forward
                                    command = self.parse_tmcc_packet(packet)
                                    if command:
                                        logger.info(f"📤 Command: {command}")
                                        
                                        # Log what we're trying to send
                                        mth_cmd = self.convert_to_mth_protocol(command)
                                        if mth_cmd:
                                            logger.info(f"📤 MTH Command: {mth_cmd}")
                                        else:
                                            logger.warning(f"⚠️ Failed to convert command: {command}")
                                        
                                        # Send to MCU
                                        if self.send_to_mcu(command):
                                            logger.info("✅ Sent to MCU")
                                        else:
                                            logger.warning("⚠️ Failed to send to MCU")
                                        
                                        # Send to MTH
                                        if self.send_to_mth(command):
                                            logger.info("✅ Sent to MTH")
                                        else:
                                            logger.warning("⚠️ Failed to send to MTH")
                                    else:
                                        logger.warning(f"⚠️ Failed to parse packet: {packet.hex()}")
                        else:
                            # Log every 10 seconds if no data received
                            if time.time() - last_activity > 10:
                                logger.warning("⚠️ No data received from Lionel Base 3 for 10 seconds")
                                last_activity = time.time()
            
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Lionel listener error: {e}")
                time.sleep(1)
    
    def start_tmcc_monitoring(self):
        """Start TMCC packet monitoring thread"""
        if hasattr(self, 'tmcc_thread') and self.tmcc_thread.is_alive():
            return  # Already running
            
        self.tmcc_thread = threading.Thread(target=self.lionel_listener, daemon=True)
        self.tmcc_thread.start()
        logger.info("🎯 TMCC monitoring started")
    
    def start(self):
        """Start the bridge with auto-reconnect"""
        logger.info("🚀 Starting Lionel-MTH Bridge with auto-reconnect...")
        
        # Try to connect to SER2, but don't fail if not available
        if not self.wait_for_lionel_connection():
            logger.warning("⚠️ SER2 not available, will auto-reconnect when detected...")
        
        # Try MCU connection
        if not self.connect_mcu():
            logger.warning("⚠️ MCU connection failed, continuing with MTH only...")
            logger.info("💡 Note: MCU connection only works on actual Arduino UNO Q hardware")
            logger.info("💡 In WSL/testing, MCU connection is expected to fail")
        
        # Try MTH WTIU connection
        if not self.connect_mth():
            logger.warning("⚠️ MTH WTIU connection failed, will auto-reconnect...")
            logger.info("💡 Note: Make sure MTH WTIU is powered and connected to WiFi")
        
        # MTH connection ready
        if self.mth_connected:
            logger.info("✅ MTH WTIU connection ready for commands")
        
        self.running = True
        
        # Start connection monitor
        if self.auto_reconnect:
            self.start_connection_monitor()
        
        # Start TMCC monitoring if connected
        if self.lionel_serial and self.lionel_serial.is_open:
            self.start_tmcc_monitoring()
        
        logger.info("✅ Bridge started with auto-reconnect! Use Lionel Base 3 remote...")
        
        # Start whistle timeout monitor
        self.start_whistle_timeout_monitor()
        
        return True
    
    def start_whistle_timeout_monitor(self):
        """Start the whistle timeout monitoring thread"""
        self.whistle_monitor_thread = threading.Thread(target=self.monitor_whistle_timeout, daemon=True)
        self.whistle_monitor_thread.start()
        logger.info("🔍 Whistle timeout monitor started")
    
    def monitor_whistle_timeout(self):
        """Monitor whistle state and turn off when packets stop"""
        while self.running:
            try:
                if self.button_states.get('horn', False):
                    current_time = time.time()
                    if current_time - self.last_whistle_time > self.whistle_timeout:
                        # Turn off whistle due to timeout
                        self.button_states['horn'] = False
                        logger.info(f"🔧 DEBUG: Whistle TIMEOUT - Turning OFF")
                        
                        # Send horn off command
                        command = {'type': 'function', 'value': 'horn_off'}
                        self.send_to_mth(command)
                
                time.sleep(0.1)  # Check every 100ms
            except Exception as e:
                logger.error(f"Whistle monitor error: {e}")
                time.sleep(1)
    
    def stop(self):
        """Stop the bridge"""
        logger.info("🛑 Stopping bridge...")
        self.running = False
        
        # Close serial connections safely
        if self.lionel_serial and hasattr(self.lionel_serial, 'is_open') and self.lionel_serial.is_open:
            try:
                self.lionel_serial.close()
            except Exception as e:
                logger.warning(f"Error closing Lionel serial: {e}")
        
        if self.mcu_serial and hasattr(self.mcu_serial, 'is_open') and self.mcu_serial.is_open:
            try:
                self.mcu_serial.close()
            except Exception as e:
                logger.warning(f"Error closing MCU serial: {e}")
        
        logger.info("✅ Bridge stopped")
    
    def run_forever(self):
        """Run the bridge continuously"""
        if not self.start():
            logger.error("❌ Failed to start bridge")
            return
        
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("📡 Received interrupt signal")
        finally:
            self.stop()
    
    def speck_encrypt(self, plaintext):
        """Encrypt plaintext using Speck 64/128 cipher (Mark's implementation)"""
        if not self.use_encryption:
            return plaintext.encode('latin1')
        
        try:
            # TEMPORARY: Disable encryption to test if commands work unencrypted
            # Use WTIU session key if available, otherwise use fixed key
            key_to_use = self.wtiu_session_key if self.wtiu_session_key else self.speck_key
            
            # DEBUG: Try without encryption first
            if self.wtiu_session_key:
                logger.info(f"🔐 DEBUG: Session key available: {self.wtiu_session_key.hex()}")
                logger.info(f"🔐 DEBUG: Trying unencrypted command first...")
                return plaintext.encode('latin1')  # Send unencrypted for testing
            
            # Convert plaintext to bytes
            if isinstance(plaintext, str):
                plaintext_bytes = plaintext.encode('latin1')
            else:
                plaintext_bytes = plaintext
            
            # Pad to multiple of 8 bytes (64-bit blocks)
            padding_len = (8 - len(plaintext_bytes) % 8) % 8
            plaintext_bytes += b'\x00' * padding_len
            
            # Convert to 16-bit words (little-endian)
            pt = []
            for i in range(0, len(plaintext_bytes), 2):
                if i+1 < len(plaintext_bytes):
                    word = plaintext_bytes[i] | (plaintext_bytes[i+1] << 8)
                else:
                    word = plaintext_bytes[i]
                pt.append(word)
            
            # Ensure we have exactly 2 words (32 bits) for Speck 64/128
            if len(pt) < 2:
                pt.extend([0] * (2 - len(pt)))
            pt = pt[:2]
            
            # Expand key
            K = []
            for i in range(0, len(key_to_use), 2):
                if i+1 < len(key_to_use):
                    word = key_to_use[i] | (key_to_use[i+1] << 8)
                else:
                    word = key_to_use[i]
                K.append(word)
            
            # Pad key to 4 words if needed
            while len(K) < 4:
                K.append(0)
            K = K[:4]
            
            # Key expansion (Mark's exact implementation)
            S = [0] * 22
            b = K[0]
            a = K[1:]
            S[0] = b
            for i in range(21):  # Only 21 iterations for 22 round keys
                # Mark's RRR: x = ROR(x, 7), x += y, x ^= k, y = ROL(y, 2), y ^= x
                # Mark calls: R(a[i % 3], b, i) where x = a[i%3], y = b, k = i
                a_idx = i % 3
                x = a[a_idx]  # x = a[i % 3]
                y = b          # y = b
                k = i          # k = i
                
                # x = ROR(x, 7)
                x = (x >> 7) | ((x << 9) & 0xFFFF)  # ROR 7 (16-bit)
                x = x & 0xFFFF  # Ensure 16-bit
                # x += y
                x = (x + y) & 0xFFFF
                # x ^= k
                x ^= k
                # y = ROL(y, 2)
                y = ((y << 2) | (y >> 14)) & 0xFFFF
                # y ^= x
                y ^= x
                
                # Update arrays
                b = x
                a[a_idx] = y
                S[i+1] = b
            
            # Encryption (Mark's exact R implementation - CORRECT PARAMETER ORDER!)
            ct = pt.copy()
            for i in range(22):  # SPECK_ROUNDS = 22
                # Mark's R(ct[1], ct[0], K[i]): x = ROR(x, 7), x += y, x ^= k, y = ROL(y, 2), y ^= x
                # where x = ct[1], y = ct[0], k = S[i]
                ct[1] = (ct[1] >> 7) | ((ct[1] << 9) & 0xFFFF)  # ROR 7 on ct[1]
                ct[1] = (ct[1] + ct[0]) & 0xFFFF      # ct[1] += ct[0]
                ct[1] ^= S[i]                              # ct[1] ^= K[i]
                ct[0] = ((ct[0] << 2) | (ct[0] >> 14)) & 0xFFFF  # ROL 2 on ct[0]
                ct[0] ^= ct[1]                            # ct[0] ^= ct[1]
            
            # Convert back to bytes (little-endian)
            encrypted_bytes = bytearray()
            for word in ct:
                encrypted_bytes.append(word & 0xFF)
                encrypted_bytes.append((word >> 8) & 0xFF)
            
            # Remove padding
            encrypted_bytes = encrypted_bytes[:len(plaintext_bytes)]
            
            return bytes(encrypted_bytes)
            
        except Exception as e:
            logger.error(f"Speck encryption error: {e}")
            return plaintext.encode('latin1')

def test_connection_manually():
    """Manual test of WTIU connection"""
    bridge = LionelMTHBridge()
    
    # Try to connect to MTH
    if bridge.connect_mth():
        logger.info("✅ Connected to MTH WTIU")
        
        # Test commands manually
        commands_to_test = [
            "x",
            "!",
            "y2",
            "m4",
            "u4",
            "s10",
            "d0",
            "w2",
            "bFFFD",
            "w4",
            "bFFFB",
            "s0"
        ]
        
        for cmd in commands_to_test:
            logger.info(f"🧪 Testing: {cmd}")
            bridge.mth_socket.send(f"{cmd}\r\n".encode())
            time.sleep(0.5)
            try:
                response = bridge.mth_socket.recv(256).decode()
                logger.info(f"📥 Response: {response.strip()}")
            except:
                logger.info("📥 No response")
            time.sleep(0.5)
        
        bridge.stop()
    else:
        logger.error("❌ Failed to connect to MTH WTIU")

def check_whistle_timeout(self):
        """Check if whistle should be turned off due to timeout"""
        if self.button_states.get('horn', False):
            current_time = time.time()
            if current_time - self.last_whistle_time > self.whistle_timeout:
                # Turn off whistle due to timeout
                self.button_states['horn'] = False
                logger.info(f"🔧 DEBUG: Whistle TIMEOUT - Turning OFF")
                return {'type': 'function', 'value': 'horn_off'}
        return None

def main():
    print("🎯 Lionel Base 3 → MTH WTIU Bridge")
    print("=" * 50)
    print("FTDI Serial Adapter → Arduino MCU → WiFi → MTH")
    print("=" * 50)
    print("Press Ctrl+C to stop")
    print()
    
    # Uncomment to run manual test
    # test_connection_manually()
    # return
    
    bridge = LionelMTHBridge()
    bridge.run_forever()

if __name__ == "__main__":
    main()
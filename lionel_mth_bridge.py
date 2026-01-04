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
import time
import logging
import threading
from threading import Lock
import subprocess
import re

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class LionelMTHBridge:
    def __init__(self):
        self.lionel_port = '/dev/ttyUSB0'  # FTDI adapter
        self.mcu_port = '/dev/ttyHS1'      # Arduino MCU
        self.mth_devices = ['192.168.0.100', '192.168.0.102']
        self.lionel_serial = None
        self.mcu_serial = None
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
        
    def wait_for_lionel_connection(self):
        """Wait for SER2 to be available and connect"""
        logger.info("🔄 Waiting for SER2 connection...")
        attempt = 0
        
        while self.running and attempt < self.max_reconnect_attempts:
            try:
                # Try to open the port to see if SER2 is connected
                test_serial = serial.Serial(self.lionel_port, baudrate=115200, timeout=1)
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
                if self.mcu_serial is None or not self.mcu_serial.is_open:
                    logger.warning("⚠️ MCU connection lost, attempting reconnect...")
                    self.connect_mcu()
                
            except Exception as e:
                logger.error(f"❌ Connection monitor error: {e}")
                
            time.sleep(self.connection_check_interval)
    
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
                baudrate=115200, 
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
        """Connect to Arduino MCU via Arduino Router"""
        try:
            # Stop Arduino Router to use MCU directly
            subprocess.run(['sudo', 'systemctl', 'stop', 'arduino-router'], 
                         capture_output=True)
            time.sleep(2)
            
            self.mcu_serial = serial.Serial(
                self.mcu_port, 
                baudrate=115200, 
                timeout=1
            )
            logger.info(f"✅ Connected to MCU on {self.mcu_port}")
            return True
        except Exception as e:
            logger.error(f"❌ MCU connection failed: {e}")
            return False
    
    def parse_tmcc_packet(self, packet):
        """Parse TMCC packet and convert to MTH command"""
        if len(packet) != 3 or packet[0] != 0xFE:
            return None
        
        cmd_field = packet[1] & 0x3F
        data_field = packet[2]
        
        # TMCC to MTH command mapping
        if cmd_field == 0x00:  # Direction/Function
            if data_field == 0x00:
                return {'type': 'direction', 'value': 'forward'}
            elif data_field == 0x1F:
                return {'type': 'direction', 'value': 'reverse'}
            elif data_field == 0x1C:
                return {'type': 'function', 'value': 'horn'}
            elif data_field == 0x1D:
                return {'type': 'function', 'value': 'bell'}
            elif data_field == 0x18:  # Smoke Increase
                return {'type': 'smoke', 'value': 'increase'}
            elif data_field == 0x19:  # Smoke Decrease
                return {'type': 'smoke', 'value': 'decrease'}
            elif data_field == 0x1A:  # Smoke On
                return {'type': 'smoke', 'value': 'on'}
            elif data_field == 0x1B:  # Smoke Off
                return {'type': 'smoke', 'value': 'off'}
            elif data_field == 0x16:  # Cab Chatter
                return {'type': 'pfa', 'value': 'cab_chatter'}
            elif data_field == 0x17:  # TowerCom
                return {'type': 'pfa', 'value': 'towercom'}
                
        elif cmd_field == 0x03:  # Speed
            return {'type': 'speed', 'value': data_field}
            
        elif cmd_field == 0x01:  # Engine/Address
            if data_field == 0x00:  # Engine Start
                return {'type': 'engine', 'value': 'start'}
            elif data_field == 0xFF:  # Engine Stop
                return {'type': 'engine', 'value': 'stop'}
            else:
                return {'type': 'engine', 'value': data_field}
        
        return None
    
    def send_to_mcu(self, command):
        """Send command to Arduino MCU"""
        if not self.mcu_serial:
            return False
        
        try:
            with self.mcu_lock:
                # Get command type code
                cmd_type_code = self.mcu_command_types.get(command['type'], 0)
                
                # Handle different value types
                if command['type'] == 'speed':
                    cmd_value = command['value']  # Speed is 0-31
                elif command['type'] == 'engine':
                    cmd_value = 1 if command['value'] == 'start' else 0
                elif command['type'] == 'direction':
                    cmd_value = 1 if command['value'] == 'forward' else 0
                elif command['type'] in ['function', 'smoke', 'pfa']:
                    # Convert string values to codes
                    value_map = {
                        'horn': 1, 'bell': 2,
                        'increase': 1, 'decrease': 2, 'on': 3, 'off': 4,
                        'cab_chatter': 1, 'towercom': 2
                    }
                    cmd_value = value_map.get(command['value'], 0)
                elif command['type'] == 'protowhistle':
                    # ProtoWhistle commands use value field directly
                    cmd_value = command['value']
                elif command['type'] == 'wled':
                    # WLED commands use engine number directly
                    cmd_value = command['value']
                else:
                    cmd_value = 0
                
                # Format command packet: [0xAA, type_code, value, 0xFF]
                cmd_bytes = bytes([0xAA, cmd_type_code, cmd_value, 0xFF])
                self.mcu_serial.write(cmd_bytes)
                logger.debug(f"Sent to MCU: {cmd_bytes.hex()}")
                return True
        except Exception as e:
            logger.error(f"MCU send error: {e}")
            return False
    
    def send_to_mth(self, command):
        """Send command to MTH WTIU via MCU (Arduino handles MTH communication)"""
        # Note: MTH communication is handled by Arduino MCU
        # Python only needs to send commands to MCU via serial
        # This method is kept for compatibility but MCU handles all MTH communication
        logger.debug(f"MTH command handled by MCU: {command}")
        return True

    def send_to_mth_original(self, command):
        """Send command to MTH WTIU via WiFi"""
        for ip in self.mth_devices:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2)
                s.connect((ip, 8882))  # MTH WTIU uses port 8882, not 80
                
                # Send HTTP request with command
                if command['type'] == 'direction':
                    path = f"/control/direction/{command['value']}"
                elif command['type'] == 'speed':
                    path = f"/control/speed/{command['value']}"
                elif command['type'] == 'function':
                    path = f"/control/function/{command['value']}"
                elif command['type'] == 'smoke':
                    path = f"/control/smoke/{command['value']}"
                elif command['type'] == 'pfa':
                    path = f"/control/pfa/{command['value']}"
                elif command['type'] == 'engine':
                    if command['value'] == 'start':
                        path = "/control/engine/start"
                    elif command['value'] == 'stop':
                        path = "/control/engine/stop"
                    else:
                        path = f"/control/engine/{command['value']}"
                else:
                    path = f"/control/{command['type']}/{command['value']}"
                
                http_request = f"GET {path} HTTP/1.1\r\nHost: {ip}\r\n\r\n"
                s.send(http_request.encode())
                
                response = s.recv(1024)
                logger.debug(f"MTH {ip} response: {response[:50]}...")
                s.close()
                
                return True
                
            except Exception as e:
                logger.debug(f"MTH {ip} error: {e}")
        
        return False
    
    def lionel_listener(self):
        """Listen for TMCC packets from Lionel Base 3"""
        logger.info("🎯 Monitoring Lionel Base 3 for TMCC packets...")
        
        while self.running:
            try:
                with self.lionel_lock:
                    if self.lionel_serial and self.lionel_serial.is_open:
                        if self.lionel_serial.in_waiting > 0:
                            data = self.lionel_serial.read(self.lionel_serial.in_waiting)
                            
                            # Look for TMCC packets
                            for i in range(len(data) - 2):
                                if data[i] == 0xFE:
                                    packet = data[i:i+3]
                                    logger.info(f"🎯 TMCC Packet: {packet.hex()}")
                                    
                                    # Parse and forward
                                    command = self.parse_tmcc_packet(packet)
                                    if command:
                                        logger.info(f"📤 Command: {command}")
                                        
                                        # Send to MCU
                                        self.send_to_mcu(command)
                                        
                                        # Send to MTH
                                        self.send_to_mth(command)
                
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
        
        self.running = True
        
        # Start connection monitor
        if self.auto_reconnect:
            self.start_connection_monitor()
        
        # Start TMCC monitoring if connected
        if self.lionel_serial and self.lionel_serial.is_open:
            self.start_tmcc_monitoring()
        
        logger.info("✅ Bridge started with auto-reconnect! Use Lionel Base 3 remote...")
        return True
    
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
        
        # Restart Arduino Router
        try:
            subprocess.run(['sudo', 'systemctl', 'start', 'arduino-router'], 
                         capture_output=True)
        except Exception as e:
            logger.debug(f"Could not restart Arduino Router: {e}")
        
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

def main():
    print("🎯 Lionel Base 3 → MTH WTIU Bridge")
    print("=" * 50)
    print("FTDI Serial Adapter → Arduino MCU → WiFi → MTH")
    print("=" * 50)
    print("Press Ctrl+C to stop")
    print()
    
    bridge = LionelMTHBridge()
    bridge.run_forever()

if __name__ == "__main__":
    main()

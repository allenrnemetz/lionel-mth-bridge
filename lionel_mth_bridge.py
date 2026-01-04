#!/usr/bin/env python3
"""
lionel_mth_bridge.py - Lionel Base 3 to MTH WTIU Bridge
Uses FTDI serial adapter for reliable TMCC data capture

Author: Allen Nemetz
Credits:
- Mark Divechhio for his immense work translating MTH commands to and from the MTH WTIU
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
import threading
import logging
import sys

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
            import subprocess
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
            elif data_field == 0x1E:  # Engine Start
                return {'type': 'engine', 'value': 'start'}
            elif data_field == 0x1F:  # Engine Stop (already used for reverse, need different mapping)
                return {'type': 'engine', 'value': 'stop'}
                
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
            # Format command for MCU
            cmd_bytes = bytes([0xAA, command['type'][0], command['value'], 0xFF])
            self.mcu_serial.write(cmd_bytes)
            logger.debug(f"Sent to MCU: {cmd_bytes.hex()}")
            return True
        except Exception as e:
            logger.error(f"MCU send error: {e}")
            return False
    
    def send_to_mth(self, command):
        """Send command to MTH WTIU via WiFi"""
        for ip in self.mth_devices:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2)
                s.connect((ip, 80))
                
                # Send HTTP request with command
                if command['type'] == 'direction':
                    path = f"/control/direction/{command['value']}"
                elif command['type'] == 'speed':
                    path = f"/control/speed/{command['value']}"
                elif command['type'] == 'function':
                    path = f"/control/function/{command['value']}"
                elif command['type'] == 'smoke':
                    if command['value'] in ['increase', 'decrease', 'on', 'off']:
                        path = f"/control/smoke/{command['value']}"
                    else:
                        path = f"/control/smoke/{command['value']}"
                elif command['type'] == 'pfa':
                    if command['value'] in ['cab_chatter', 'towercom']:
                        path = f"/control/pfa/{command['value']}"
                    else:
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
    
    def start(self):
        """Start the bridge"""
        logger.info("🚀 Starting Lionel-MTH Bridge...")
        
        if not self.connect_lionel():
            return False
        
        if not self.connect_mcu():
            logger.warning("⚠️ MCU connection failed, continuing with MTH only...")
        
        self.running = True
        
        # Start listener thread
        listener_thread = threading.Thread(target=self.lionel_listener)
        listener_thread.daemon = True
        listener_thread.start()
        
        logger.info("✅ Bridge started! Use Lionel Base 3 remote...")
        return True
    
    def stop(self):
        """Stop the bridge"""
        logger.info("🛑 Stopping bridge...")
        self.running = False
        
        if self.lionel_serial:
            self.lionel_serial.close()
        
        if self.mcu_serial:
            self.mcu_serial.close()
        
        # Restart Arduino Router
        try:
            import subprocess
            subprocess.run(['sudo', 'systemctl', 'start', 'arduino-router'], 
                         capture_output=True)
        except:
            pass
        
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

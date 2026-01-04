#!/usr/bin/env python3
"""
test_ftdi.py - Test FTDI connection and TMCC data capture

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

Test script for verifying FTDI cable connection and monitoring TMCC packets
from Lionel Base 3 via SER2 box.
"""

import serial
import time

def test_ftdi_connection():
    """Test FTDI cable and monitor for TMCC data"""
    print("🎯 FTDI CABLE TEST")
    print("=" * 40)
    
    try:
        # Connect to FTDI
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        print("✅ Connected to /dev/ttyUSB0")
        print(f"Port: {ser}")
        print()
        
        print("📡 Monitoring for TMCC data...")
        print("Use your Lionel Base 3 remote now!")
        print("Press Ctrl+C to stop")
        print("-" * 40)
        
        packet_count = 0
        
        try:
            while True:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    print(f"Raw data: {data.hex()}")
                    
                    # Look for TMCC packets
                    for i in range(len(data) - 2):
                        if data[i] == 0xFE:
                            packet = data[i:i+3]
                            packet_count += 1
                            print(f"🎯 TMCC Packet #{packet_count}: {packet.hex()}")
                            
                            # Basic packet interpretation
                            cmd_field = packet[1] & 0x3F
                            data_field = packet[2]
                            
                            if cmd_field == 0x00:
                                if data_field == 0x00:
                                    print("   → Forward Direction")
                                elif data_field == 0x1C:
                                    print("   → Horn/Whistle")
                                elif data_field == 0x1D:
                                    print("   → Bell")
                                elif data_field == 0x18:
                                    print("   → 💨 Smoke Increase (Number 9 on Cab-1, SMOKE INCREASE on Cab-2/3)")
                                elif data_field == 0x19:
                                    print("   → 💨 Smoke Decrease (Number 8 on Cab-1, SMOKE DECREASE on Cab-2/3)")
                                elif data_field == 0x1A:
                                    print("   → 💨 Smoke On (SMOKE ON on Cab-2/3)")
                                elif data_field == 0x1B:
                                    print("   → 💨 Smoke Off (SMOKE OFF on Cab-2/3)")
                                elif data_field == 0x16:
                                    print("   → 🗣️ Cab Chatter (CAB CHATTER on Cab-2/3, AUX on Cab-1)")
                                elif data_field == 0x17:
                                    print("   → 📢 TowerCom (TOWERCOM on Cab-2/3)")
                            elif cmd_field == 0x03:
                                print(f"   → Speed: {data_field}")
                            elif cmd_field == 0x01:
                                if data_field == 0x00:
                                    print("   → 🚂 ENGINE START (AUX 1 on all Cab remotes)")
                                elif data_field == 0xFF:
                                    print("   → 🛑 ENGINE STOP (Number 5 on all Cab remotes)")
                                else:
                                    print(f"   → Engine: {data_field}")
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print(f"\n📊 Monitoring stopped")
            print(f"Total TMCC packets found: {packet_count}")
        
        ser.close()
        print("✅ Test complete")
        
    except Exception as e:
        print(f"❌ Error: {e}")

def test_mcu_connection():
    """Test Arduino MCU connection"""
    print("\n🔧 MCU CONNECTION TEST")
    print("=" * 40)
    
    try:
        import subprocess
        
        # Stop Arduino Router
        print("Stopping Arduino Router...")
        subprocess.run(['sudo', 'systemctl', 'stop', 'arduino-router'], 
                     capture_output=True)
        time.sleep(2)
        
        # Connect to MCU
        ser = serial.Serial('/dev/ttyHS1', 115200, timeout=1)
        print("✅ Connected to MCU on /dev/ttyHS1")
        
        # Send test command
        ser.write(b'TEST\n')
        print("Sent: TEST")
        
        # Read response
        time.sleep(0.5)
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"MCU response: {data}")
        else:
            print("No MCU response")
        
        ser.close()
        
        # Restart Arduino Router
        subprocess.run(['sudo', 'systemctl', 'start', 'arduino-router'], 
                     capture_output=True)
        print("✅ MCU test complete")
        
    except Exception as e:
        print(f"❌ MCU error: {e}")

def test_mth_wifi():
    """Test MTH WTIU WiFi connection"""
    print("\n🌐 MTH WiFi TEST")
    print("=" * 40)
    
    mth_devices = ['192.168.0.100', '192.168.0.102']
    
    for ip in mth_devices:
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)
            s.connect((ip, 80))
            
            # Simple HTTP request
            s.send(b"GET / HTTP/1.1\r\nHost: " + ip.encode() + b"\r\n\r\n")
            response = s.recv(1024)
            s.close()
            
            print(f"✅ {ip}: {response[:50]}...")
            
        except Exception as e:
            print(f"❌ {ip}: {e}")

if __name__ == "__main__":
    print("🎯 LIONEL-MTH BRIDGE TEST SUITE")
    print("=" * 50)
    
    # Test FTDI connection
    test_ftdi_connection()
    
    # Test MCU connection
    test_mcu_connection()
    
    # Test MTH WiFi
    test_mth_wifi()
    
    print("\n🚀 All tests complete!")
    print("If FTDI shows TMCC packets, run: python3 lionel_mth_bridge.py")

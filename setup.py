#!/usr/bin/env python3
"""
setup.py - One-click setup for Lionel-MTH Bridge on Arduino UNO Q

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

Automated setup script for installing dependencies and testing hardware connections.
"""

import subprocess
import sys
import os

def run_command(cmd, description):
    """Run a command and show results"""
    print(f"🔧 {description}...")
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            print(f"✅ {description} - SUCCESS")
            return True
        else:
            print(f"❌ {description} - FAILED")
            print(f"Error: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print(f"⏰ {description} - TIMEOUT")
        return False
    except Exception as e:
        print(f"❌ {description} - ERROR: {e}")
        return False

def check_dependencies():
    """Check if required dependencies are installed"""
    print("🔍 Checking dependencies...")
    
    # Check pyserial
    try:
        import serial
        print("✅ pyserial installed")
        return True
    except ImportError:
        print("❌ pyserial not installed")
        return False

def install_dependencies():
    """Install required dependencies"""
    print("📦 Installing dependencies...")
    
    # Update package list
    if not run_command("sudo apt update", "Updating package list"):
        return False
    
    # Install pyserial
    if not run_command("sudo apt install python3-serial -y", "Installing pyserial"):
        return False
    
    return True

def test_hardware():
    """Test hardware connections"""
    print("🔌 Testing hardware connections...")
    
    # Check FTDI device
    if not run_command("lsusb | grep -i ftdi", "Checking FTDI device"):
        print("⚠️ FTDI device not found - connect FTDI cable")
        return False
    
    # Check ttyUSB device
    if not run_command("ls -la /dev/ttyUSB0", "Checking /dev/ttyUSB0"):
        print("⚠️ /dev/ttyUSB0 not found - check FTDI driver")
        return False
    
    # Test serial connection
    try:
        import serial
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        ser.close()
        print("✅ FTDI serial connection working")
        return True
    except Exception as e:
        print(f"❌ FTDI serial test failed: {e}")
        return False

def main():
    print("🎯 Lionel-MTH Bridge Setup")
    print("=" * 40)
    
    # Step 1: Check dependencies
    if not check_dependencies():
        print("\n📦 Installing missing dependencies...")
        if not install_dependencies():
            print("❌ Failed to install dependencies")
            sys.exit(1)
    
    # Step 2: Test hardware
    print("\n🔌 Testing hardware connections...")
    if not test_hardware():
        print("\n⚠️ Hardware issues detected:")
        print("1. Ensure FTDI cable is connected")
        print("2. Check FTDI driver: lsmod | grep ftdi_sio")
        print("3. Verify device: ls /dev/ttyUSB*")
        print("\n🔧 You can run setup again after fixing hardware issues")
        sys.exit(1)
    
    # Step 3: Test bridge
    print("\n🚀 Testing bridge...")
    print("Connect Lionel SER2 box to FTDI cable and use Lionel remote...")
    
    try:
        result = subprocess.run([sys.executable, "test_ftdi.py"], timeout=30)
        if result.returncode == 0:
            print("✅ Bridge test completed")
        else:
            print("⚠️ Bridge test had issues - check hardware connections")
    except subprocess.TimeoutExpired:
        print("⏰ Bridge test timed out - this is normal without SER2 box")
    except Exception as e:
        print(f"❌ Bridge test error: {e}")
    
    print("\n🎉 Setup complete!")
    print("📋 Next steps:")
    print("1. Connect Lionel Base 3 → SER2 box → FTDI cable")
    print("2. Run: python3 test_ftdi.py (to test TMCC capture)")
    print("3. Run: python3 main.py (to start bridge)")
    print("\n🎯 Your Lionel-MTH bridge is ready!")

if __name__ == "__main__":
    main()

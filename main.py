#!/usr/bin/env python3
"""
main.py - Entry point for Lionel-MTH Bridge

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

Entry point and error handling for the Lionel Base 3 to MTH WTIU bridge.
"""

import time
import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def main():
    print("=== LIONEL-MTH BRIDGE STARTING ===")
    print("This script will run continuously")
    print("Press Ctrl+C to stop")
    print("Checking imports...")
    
    try:
        from lionel_mth_bridge import LionelMTHBridge
        bridge = LionelMTHBridge()
        bridge.run_forever()
    except ImportError as e:
        print(f"❌ Import error: {e}")
        if "serial" in str(e):
            print("❌ pyserial not installed")
            print("Run: sudo apt install python3-serial")
        else:
            print(f"❌ Missing module: {e}")
    except KeyboardInterrupt:
        print("\n📡 Bridge stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

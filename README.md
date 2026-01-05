# Lionel Base 3 → MTH WTIU Bridge

**Control MTH DCS trains using Lionel Cab-1L, Cab-2, or Cab-3 remotes via Arduino UNO Q**

Author: Allen Nemetz  
Copyright © 2026 Allen Nemetz. All rights reserved.  
License: GNU General Public License v3.0

---

## Credits

- **Mark DiVecchio** for MTH WTIU protocol translation work  
  http://www.silogic.com/trains/RTC_Running.html
- **Lionel LLC** for TMCC and Legacy protocol specifications
- **O Gauge Railroading Forum** (https://www.ogrforum.com/)

## Disclaimer

This software is provided "as-is" without warranty. The author assumes no liability for damages resulting from use or misuse. Users are responsible for safe operation of model railroad equipment.

---

## Overview

This bridge enables Lionel Base 3 systems to control MTH DCS trains using Arduino UNO Q's dual-processor architecture:

- **MPU (Qualcomm QRB2210)**: Runs Python, handles Lionel Base 3 TMCC via USB, WiFi to MTH WTIU
- **MCU (STM32U585)**: Receives commands from MPU, processes locally if needed

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│ Arduino UNO Q                                           │
│                                                         │
│  MPU (Linux - Qualcomm QRB2210)                         │
│  • Python: lionel_mth_bridge_fixed.py                   │
│  • WiFi to MTH WTIU (with mDNS discovery)               │
│  • Speck encryption for secure communication            │
│  • Serial to Lionel Base 3 (SER2 via FTDI)              │
│                                                         │
│         ↓ arduino-router socket                         │
│                                                         │
│  MCU (Arduino - STM32U585)                              │
│  • Sketch: mcu_mth_handler.ino                          │
│  • NO WiFi (handled by Python on MPU)                   │
│  • Receives commands via Serial1                        │
│  • USB Serial for debugging                             │
└─────────────────────────────────────────────────────────┘
```

---

## Features

- ✅ **Auto-Reconnect**: Detects and connects when SER2 is powered on
- ✅ **Power-Cycle Resilient**: Handles SER2 power cycling
- ✅ **mDNS Discovery**: Auto-finds MTH WTIU (no hardcoded IP)
- ✅ **Speck Encryption**: Secure WTIU communication
- ✅ **Direct Engine Mapping**: Lionel 1-99 → MTH 1-99
- ✅ **Fine Speed Control**: Ultra-fine low-speed control
- ✅ **Smart Whistle**: Auto-switches between regular/protowhistle
- ✅ **ProtoWhistle Support**: Full MTH protowhistle control
- ✅ **LED Indicator**: Shows WTIU connection status

---

## Hardware Requirements

| Component | Model | Description |
|-----------|-------|-------------|
| **Lionel Base 3** | 2208010 | TMCC command base |
| **Lionel Remote** | Cab-1L/Cab-2/Cab-3 | Base 3 compatible |
| **Lionel LCS SER2** | 6-81326 | TMCC to serial converter |
| **FTDI Cable** | USB-SER9 | USB serial adapter |
| **Arduino UNO Q** | ABX00162 | Dual-processor board |
| **MTH WTIU** | 50-1039 | WiFi DCS controller |

### Connection Diagram
```
Lionel Base 3 → SER2 → FTDI Cable → Arduino UNO Q → WiFi → MTH WTIU
```

---

## Quick Start

### 1. Upload MCU Sketch

```
1. Open mcu_mth_handler.ino in Arduino IDE
2. Tools → Board → Arduino UNO Q (or STM32U585)
3. Tools → Port → (your COM port)
4. Click Upload
5. Open Serial Monitor at 115200 baud
6. You should see: "=== MTH WTIU Handler Starting ==="
```

### 2. Test MCU

Type in Serial Monitor:
```
CMD:2:15
```

Expected output:
```
RX USB: CMD:2:15
Parsed - Type: 2, Value: 15
Executing command: type=2, engine=1, value=15
Speed: 15
Command processed (MTH connection handled by Python)
Sent ACK to MPU
```

### 3. Install Python Dependencies

SSH into Arduino UNO Q:
```bash
# Replace <YOUR_BOARD_IP> with your board's actual IP address
ssh root@<YOUR_BOARD_IP>

# Update package list
apt update

# Install required packages using apt
apt install -y python3-serial python3-zeroconf python3-pycryptodome
```

See `INSTALL_DEPENDENCIES.md` for details.

### 4. Deploy Python Script

```bash
# From your computer (replace <YOUR_BOARD_IP> with your board's IP)
scp lionel_mth_bridge_fixed.py root@<YOUR_BOARD_IP>:/home/

# SSH into board
ssh root@<YOUR_BOARD_IP>
cd /home
python3 lionel_mth_bridge_fixed.py
```

### 5. Test MPU-MCU Connection

```bash
# On the board via SSH
python3 -c "
import socket
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect('/var/run/arduino-router.sock')
s.send(b'CMD:2:15\n')
s.close()
print('Command sent to MCU')
"
```

---

## Configuration

### Python Script Settings

Edit `lionel_mth_bridge_fixed.py`:

```python
# MTH WTIU (auto-discovered via mDNS, or manual fallback)
self.mth_host = None  # Auto-discover, or set to 'YOUR_WTIU_IP'
self.mth_port = 50001

# Lionel Base 3 SER2
self.lionel_port = '/dev/ttyUSB0'  # FTDI adapter

# Features
self.use_mdns = True         # Auto-discover WTIU
self.use_encryption = True   # Speck encryption
```

### Network Requirements

- **Same network** as MTH WTIU
- **mDNS/Bonjour enabled** (usually default on modern routers)

---

## Command Format

Commands between MPU and MCU use: `CMD:type:value`

### Command Types

| Type | Description | Values |
|------|-------------|--------|
| `1` | Direction | 0=reverse, 1=forward |
| `2` | Speed | 0-31 |
| `3` | Function | 1=horn, 2=bell |
| `4` | Smoke | 1-4 (increase/decrease/on/off) |
| `5` | PFA | 1=cab_chatter, 2=towercom |
| `6` | Engine | 0=stop, 1=start |
| `8` | ProtoWhistle | Various |
| `9` | WLED | Engine number |

### Examples

```
CMD:2:15    # Set speed to 15
CMD:1:1     # Set direction forward
CMD:3:1     # Activate horn
CMD:6:1     # Engine startup
```

---

## TMCC to MTH Command Mapping

| TMCC Command | Packet | MTH Command | Description |
|--------------|--------|-------------|-------------|
| Forward | FE 00 00 | d0 | Forward motion |
| Reverse | FE 00 1F | d1 | Reverse motion |
| Speed | FE 03 XX | sXX | Speed control (0-31) |
| Horn | FE 00 1C | w2 | Horn/whistle |
| Bell | FE 00 1D | w4 | Bell |
| Engine Start | FE 01 00 | u4 | Startup sequence |
| Engine Stop | FE 01 FF | u5 | Shutdown sequence |
| Smoke Increase | FE 00 18 | - | Smoke intensity up |
| Smoke Decrease | FE 00 19 | - | Smoke intensity down |
| Cab Chatter | FE 00 16 | - | PFA cab chatter |
| TowerCom | FE 00 17 | - | PFA TowerCom |

---

## Remote Control Guide

### Engine Control

**Startup:**
- **AUX 1** button (all remotes)
- **MASTER KEY → ENGINE START** (Cab-2/Cab-3)
- Sends: `FE 01 00` → MTH: `u4`

**Shutdown:**
- **Number 5** key (all remotes)
- **MASTER KEY → ENGINE STOP** (Cab-2/Cab-3)
- Sends: `FE 01 FF` → MTH: `u5`

### Smoke Control

**Cab-1L:**
- **Number 8**: Decrease smoke
- **Number 9**: Increase smoke

**Cab-2/Cab-3:**
- **SMOKE INCREASE** button
- **SMOKE DECREASE** button
- **SMOKE ON/OFF** buttons

### ProtoWhistle

- **AUX2**: Toggle protowhistle mode
- **Whistle button**: Adapts based on mode
  - OFF: Regular MTH whistle
  - ON: MTH protowhistle (quillable)

---

## Troubleshooting

### MCU Not Responding

1. Check LED blinks 3 times on startup
2. Open Serial Monitor at 115200 baud
3. Look for "MTH WTIU Handler Ready"
4. Test with: `CMD:2:15`

### Python Can't Connect to MCU

```bash
# Check arduino-router service
ps aux | grep arduino-router

# Check socket exists
ls -la /var/run/arduino-router.sock

# Test socket
python3 -c "import socket; s=socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect('/var/run/arduino-router.sock'); print('OK')"
```

### MTH WTIU Not Found

1. Check WTIU is powered on
2. Verify same WiFi network (2.4GHz)
3. Test mDNS: `avahi-browse -a` (on board)
4. Fallback: Set manual IP in Python script
   ```python
   self.mth_host = '192.168.x.x'  # Your WTIU IP
   ```

### Lionel Base 3 Not Detected

```bash
# Check FTDI adapter
ls -la /dev/ttyUSB*

# Test serial port
cat /dev/ttyUSB0
```

### Enable Debug Logging

Edit `lionel_mth_bridge_fixed.py`:
```python
logging.basicConfig(level=logging.DEBUG, ...)
```

---

## Files

### Core Files
- **`mcu_mth_handler.ino`** - MCU sketch (upload to Arduino)
- **`lionel_mth_bridge_fixed.py`** - Python bridge (run on MPU)

### Documentation
- **`README.md`** - This file
- **`INSTALL_DEPENDENCIES.md`** - Python package installation guide

### Configuration
- **`.gitignore`** - Git ignore rules
- **`.gitattributes`** - Git attributes

---

## What's Different from Original Design

### Original (Incorrect)
- MCU handled WiFi, mDNS, Speck encryption
- Used WiFiS3, ArduinoMDNS libraries (don't exist for STM32U585)
- MCU tried to connect directly to WTIU

### Current (Correct)
- **MPU handles WiFi** - Qualcomm chip has WiFi
- **MPU handles mDNS** - Python zeroconf library
- **MPU handles encryption** - Python crypto libraries
- **MCU just receives commands** - Via Serial1 from MPU
- **Proper architecture** - Uses Arduino UNO Q's dual-processor design

---

## Next Steps

1. ✅ Upload `mcu_mth_handler.ino` to MCU
2. ✅ Test MCU with Serial Monitor
3. ✅ Install Python dependencies on MPU
4. ✅ Deploy Python script to MPU
5. ✅ Test MPU-MCU communication
6. ⏳ Get SER2 hardware for Lionel Base 3
7. ⏳ Connect MTH WTIU to WiFi
8. ⏳ Test complete system!

---

## License

GNU General Public License v3.0

## Author

© Allen Nemetz 2026

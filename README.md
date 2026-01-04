# Lionel Base 3 → MTH WTIU Bridge

Author: Allen Nemetz

Credits:
- **Mark DiVecchio** for his immense work translating MTH commands to and from the MTH WTIU  
  http://www.silogic.com/trains/RTC_Running.html
- **Lionel LLC** for publishing TMCC and Legacy protocol specifications
- **O Gauge Railroading Forum** (https://www.ogrforum.com/) for the model railroad community

## Disclaimer
This software is provided "as-is" without warranty. The author assumes no liability for any damages 
resulting from the use or misuse of this software. Users are responsible for ensuring safe operation 
of their model railroad equipment.

## Copyright
Copyright (c) 2026 Allen Nemetz. All rights reserved.

## Overview
This project creates a bridge between Lionel Base 3 systems and MTH DCS systems using Arduino UNO Q's dual-processor architecture.

- **MPU (Linux Processor)**: Runs Python code to handle Lionel Base 3 TMCC communication via USB
- **MCU (Sub-processor)**: Runs C++ code to handle MTH WTIU wireless communication

## Features
- Control MTH trains using Lionel Cab-1L, Cab-2, or Cab-3 remotes
- **Auto-Reconnect**: Automatically detects and connects when SER2 is powered on
- **Power-Cycle Resilient**: Bridge runs continuously, handles SER2 power cycling
- **Smart Whistle**: Whistle button automatically uses regular whistle or protowhistle based on mode
- **Direct Engine Mapping**: Lionel engine 1-99 maps directly to MTH engine 1-99 (no offset)
- **Fine Speed Control**: Ultra-fine low-speed control (1,3,5,10 Smph for first 4 steps)
- **mDNS Discovery**: Automatically finds MTH WTIU on network (no hardcoded IP)
- **Speck Encryption**: Secure communication with MTH WTIU (same as RTCRemote)
- **Speed Control**: TMCC speed steps (0-31) convert to MTH speed (0-120 Smph)
- **AUX2 Button**: Toggles protowhistle mode on/off
- **Whistle Button**: 
  - When protowhistle OFF: Uses regular MTH whistle
  - When protowhistle ON: Uses MTH protowhistle (quillable)
- **LED Indicator**: Shows connection status to MTH WTIU

## Hardware Requirements
- **Lionel Base 3** 2208010
- **Lionel Cab-1L, Cab-2, or Cab-3 Remote** (Base 3 compatible)
- **Lionel LCS SER2** (TMCC to serial converter 6-81326)
- **FTDI USB Serial Cable** (USB-SER9 or similar)
- **Arduino UNO Q** (MPU running Python ABX00162)
- **MTH WTIU** (WiFi-enabled DCS controller 50-1039)

## Connection Diagram
```
Lionel Base 3 → SER2 Box → FTDI Cable → Arduino UNO Q → WiFi → MTH WTIU
```

## Installation

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install python3-serial -y
```

### 2. Configure WiFi Credentials
**IMPORTANT:** You must configure WiFi credentials in the Arduino sketch before uploading:

#### Method 1: Edit Arduino Sketch (Recommended)
1. Open `mcu_mth_handler.ino` in Arduino IDE
2. Find lines 47-48:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";        // <-- Your WiFi network name
   const char* password = "YOUR_WIFI_PASSWORD";  // <-- Your WiFi password
   ```
3. **Replace** `YOUR_WIFI_SSID` with your actual WiFi network name
4. **Replace** `YOUR_WIFI_PASSWORD` with your actual WiFi password
5. **Save** and upload to Arduino UNO Q

#### Method 2: Use WiFiManager (Advanced)
1. Uncomment lines 50-52 in `mcu_mth_handler.ino`:
   ```cpp
   // #include <WiFiManager.h>
   // WiFiManager wifiManager;
   ```
2. Follow WiFiManager setup instructions for web-based configuration

### 3. Test Hardware
```bash
# Test FTDI connection
python3 test_ftdi.py
```

### 4. Run Bridge
```bash
# Start the bridge
python3 main.py

# Or run directly
python3 lionel_mth_bridge.py
```

## WiFi Configuration

### 🔧 Required Setup
The Arduino UNO Q (MCU) needs WiFi credentials to communicate with your MTH WTIU. This must be configured **before** uploading the Arduino sketch.

### 📱 Step-by-Step Instructions

#### 1. Open Arduino Sketch
```bash
# Open in Arduino IDE or App Lab
mcu_mth_handler.ino
```

#### 2. Locate WiFi Section
Find lines 46-52 in the sketch:
```cpp
// WiFi configuration - UPDATE THESE VALUES
const char* ssid = "YOUR_WIFI_SSID";        // <-- Your WiFi network name
const char* password = "YOUR_WIFI_PASSWORD";  // <-- Your WiFi password

// Alternative: Use WiFiManager for configuration (uncomment to enable)
// #include <WiFiManager.h>
// WiFiManager wifiManager;
```

#### 3. Enter Your WiFi Credentials
**Example:**
```cpp
// If your WiFi network is "MyHomeWiFi" and password is "password123"
const char* ssid = "MyHomeWiFi";
const char* password = "password123";
```

#### 4. Upload to Arduino UNO Q
1. Connect Arduino UNO Q to your computer
2. Select "Arduino UNO Q" as board
3. Upload the modified sketch

### 🔍 Troubleshooting WiFi

#### Common Issues:
- **Wrong credentials** - Double-check SSID and password
- **Hidden networks** - Ensure your network is visible
- **5GHz networks** - Arduino UNO Q only supports 2.4GHz
- **Special characters** - Avoid spaces/special chars in passwords
- **mDNS library missing** - Install ArduinoMDNS from Library Manager
- **WTIU not advertising** - Ensure MTH WTIU is broadcasting mDNS services

#### Verification:
After uploading, open Serial Monitor (115200 baud). You should see:
```
=== MTH WTIU Handler Starting ===
Initializing WiFi...
WiFi connected
IP address: 192.168.x.x
mDNS responder started
Searching for MTH WTIU devices via mDNS...
Found 1 WTIU service(s):
  1: MTH-WTIU-12345 (192.168.0.100:8882)
✅ Connected to WTIU: 192.168.0.100:8882
=== MTH WTIU Handler Ready ===
```

### 🌐 Network Requirements
- **2.4GHz WiFi network** (required)
- **Same network as MTH WTIU** (both devices must be on same subnet)
- **mDNS/Bonjour enabled** (Arduino UNO Q has native support via Linux/Avahi)
- **No captive portal** (hotel/airport WiFi won't work)
- **Stable connection** - WiFi dropouts will interrupt train control
- **Router supports mDNS** - Most modern routers do, but some may need enabling

### 📱 Arduino UNO Q mDNS Support
- **Official ArduinoMDNS library** - Compatible with all Arduino boards
- **WiFiNINA support** - Works with Arduino UNO Q's WiFi module
- **Service discovery** - Finds WTIU devices automatically
- **Port rotation handling** - Adapts to WTIU port changes
- **No extra dependencies** - Single library installation

## Usage

### Auto-Reconnect Behavior
The bridge includes intelligent auto-reconnect capabilities:

- **🔄 SER2 Detection**: Automatically detects when SER2 is powered on
- **⏳ Wait Mode**: Runs continuously waiting for SER2 connection
- **🔌 Power-Cycle Handling**: Handles SER2 power cycling without stopping
- **📡 Connection Monitoring**: Continuously monitors connection health
- **🔄 Automatic Recovery**: Reconnects automatically if connection lost

### 1. **Connect Hardware**:
   - Connect Lionel Base 3 to SER2 box
   - Connect SER2 box to FTDI cable
   - Connect FTDI cable to Arduino UNO Q USB port

2. **Test Connection**:
   ```bash
   python3 test_ftdi.py
   ```
   Use your Lionel Base 3 remote - you should see TMCC packets.

3. **Start Bridge**:
   ```bash
   python3 main.py
   ```
   The bridge will monitor for TMCC packets and forward them to MTH devices.

## TMCC Command Mapping

| TMCC Command | MTH Command | Description |
|--------------|-------------|-------------|
| Forward (FE 00 00) | /control/direction/forward | Forward motion |
| Reverse (FE 00 1F) | /control/direction/reverse | Reverse motion |
| Speed (FE 03 XX) | /control/speed/XX | Speed control |
| Horn (FE 00 1C) | /control/function/horn | Horn/whistle |
| Bell (FE 00 1D) | /control/function/bell | Bell |
| Engine Start (FE 01 00) | /control/engine/start | Engine startup |
| Engine Stop (FE 01 FF) | /control/engine/stop | Engine shutdown |
| Smoke Increase (FE 00 18) | /control/smoke/increase | Smoke intensity up |
| Smoke Decrease (FE 00 19) | /control/smoke/decrease | Smoke intensity down |
| Smoke On (FE 00 1A) | /control/smoke/on | Smoke unit on |
| Smoke Off (FE 00 1B) | /control/smoke/off | Smoke unit off |
| Cab Chatter (FE 00 16) | /control/pfa/cab_chatter | PFA cab chatter on/off |
| TowerCom (FE 00 17) | /control/pfa/towercom | PFA TowerCom on/off |

### Engine Control

#### Engine Startup (All Cab Remotes):
- **AUX 1 button** on Cab-1L, Cab-2, and Cab-3 remotes
- **MASTER KEY → ENGINE START** on Cab-2/Cab-3 (additional method)
- **ENGINE START button** on Cab-2 (additional method)
- Sends TMCC packet: `FE 01 00`
- Translates to MTH: `/control/engine/start`

#### Engine Shutdown (All Cab Remotes):
- **Number 5 key** on Cab-1L, Cab-2, and Cab-3 keypads
- **MASTER KEY → ENGINE STOP** on Cab-2/Cab-3 (additional method)  
- **ENGINE STOP button** on Cab-2 (additional method)
- Sends TMCC packet: `FE 01 FF`
- Translates to MTH: `/control/engine/stop`

#### Remote-Specific Controls:
- **Cab-1L**: AUX 1 = Start, Number 5 = Stop (basic controls)
- **Cab-2**: AUX 1 + Number 5 + ENGINE START/STOP buttons + MASTER KEY wheel
- **Cab-3**: AUX 1 + Number 5 + MASTER KEY wheel (enhanced power commands)

### Smoke Control

#### Cab-1L Smoke Control:
- **Number 8 key**: Decrease smoke intensity (Max → Medium → Low → Off)
- **Number 9 key**: Increase smoke intensity (Off → Low → Medium → Max)
- Requires smoke to be set to maximum first for step control

#### Cab-2 Smoke Control:
- **SMOKE INCREASE button**: Increase smoke intensity
- **SMOKE DECREASE button**: Decrease smoke intensity  
- **SMOKE ON button**: Turn smoke unit on
- **SMOKE OFF button**: Turn smoke unit off

#### Cab-3 Smoke Control:
- **SMOKE INCREASE button**: Increase smoke intensity
- **SMOKE DECREASE button**: Decrease smoke intensity
- **SMOKE ON button**: Turn smoke unit on
- **SMOKE OFF button**: Turn smoke unit off

#### Universal Smoke Commands:
- **Smoke Increase**: `FE 00 18` → `/control/smoke/increase`
- **Smoke Decrease**: `FE 00 19` → `/control/smoke/decrease`
- **Smoke On**: `FE 00 1A` → `/control/smoke/on`
- **Smoke Off**: `FE 00 1B` → `/control/smoke/off`

### PFA (Proto-Sound Effects Animation) Control

#### Cab Chatter to PFA:
- **CAB CHATTER button** (Cab-2/Cab-3) or **AUX button** (Cab-1L)
- Toggles PFA cab chatter effects on/off
- **TMCC Packet**: `FE 00 16` → `/control/pfa/cab_chatter`

#### TowerCom to PFA:
- **TOWERCOM button** (Cab-2/Cab-3)
- Toggles PFA TowerCom announcements on/off
- **TMCC Packet**: `FE 00 17` → `/control/pfa/towercom`

#### PFA Control by Remote:
- **Cab-1L**: AUX buttons may trigger cab chatter (varies by setup)
- **Cab-2**: Dedicated CAB CHATTER and TOWERCOM buttons
- **Cab-3**: Dedicated CAB CHATTER and TOWERCOM buttons

#### Universal PFA Commands:
- **Cab Chatter Toggle**: `FE 00 16` → `/control/pfa/cab_chatter`
- **TowerCom Toggle**: `FE 00 17` → `/control/pfa/towercom`

#### High-Speed Testing (Steps 26-31):
- **~5.8 Smph per step** for testing up to 120 Smph

### ProtoWhistle Control

#### AUX2 Button:
- **Toggle**: Press AUX2 to toggle protowhistle mode
- **Smart Whistle**: Whistle button adapts based on protowhistle state
- **Pitch Control**: 4 pitch levels available when protowhistle enabled

## Troubleshooting

### 🔧 Quick Setup Checklist
1. **WiFi configured** in `mcu_mth_handler.ino` (lines 47-48)
2. **SER2 connected** to Lionel Base 3
3. **FTDI cable** connected to Arduino UNO Q
4. **MTH WTIU** on same WiFi network
5. **Python dependencies** installed: `pip install pyserial`

### MPU Issues
- Check USB connection to Lionel Base 3
- Check Python dependencies: `pip3 list | grep serial`
- Verify FTDI cable: `python test_ftdi.py`

### MCU Issues
- **WiFi not connecting**: Double-check SSID/password in sketch
- **Can't find WTIU**: Ensure both devices on same 2.4GHz network
- **ArduinoMDNS library missing**: Install via Arduino IDE Library Manager
- **WTIU port changes**: ArduinoMDNS automatically handles port rotation
- **Serial Monitor**: Should show "mDNS responder started" and WTIU discovery

### ProtoWhistle Issues
- Verify MTH engine supports protowhistle
- Check AUX2 button mapping in TMCC commands
- Ensure protowhistle is properly enabled/disabled

### Command Not Working
- Check TMCC command parsing in logs
- Verify engine number mapping (1-99)
- Check WTIU connection status
- Ensure MTH engine is powered on and addressed

### 📱 WiFi Configuration Problems
**Symptoms:**
- "WiFi connection failed" in Serial Monitor
- No IP address shown
- Can't find MTH WTIU

**Solutions:**
1. **Verify credentials** - Check SSID/password spelling
2. **Network compatibility** - Use 2.4GHz only
3. **Network security** - Avoid WPA3/Enterprise networks
4. **Signal strength** - Move closer to router
5. **Router settings** - Enable mDNS/bonjour services

### 🔍 Debug Mode
Enable debug logging by changing line 31 in `lionel_mth_bridge.py`:
```python
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
```

## License
GNU General Public License v3.0

## Author
© Allen Nemetz Copyright 2026

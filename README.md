# O Gauge Command Control Bridge

**Control MTH DCS trains using your Lionel Cab-1L, Cab-2, or Cab-3 remote**

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Beta Release](https://img.shields.io/badge/Status-Beta-orange.svg)]()

---

## What Is This?

This bridge translates Lionel TMCC and Legacy commands to MTH DCS commands, letting you control MTH Proto-Sound 2 and Proto-Sound 3 locomotives with your Lionel remote. Full support for both TMCC (32-step speed) and Legacy (200-step speed) protocols.

### Key Features

- **Dual Protocol Support** - TMCC and Legacy protocols fully supported
- **200-Step Speed Control** - Legacy's fine-grained speed control mapped to DCS sMPH
- **ProtoWhistle/Quilling Horn** - Legacy whistle slider controls MTH whistle pitch
- **Extended Startup/Shutdown** - Hold power button for full startup/shutdown sequences
- **PFA Announcements** - Passenger/Freight announcements via CAB3
- **Auto Engine Discovery** - Automatically discovers MTH engines on WTIU

---

## Hardware Required

| Component | Purpose |
|-----------|---------|
| **Lionel Base 3** (6-82972) | Receives commands from remote |
| **Lionel Remote** (Cab-1L, Cab-2, or Cab-3) | Your controller |
| **Lionel LCS SER2** (6-81326) | Serial output from Base 3 |
| **FTDI USB-Serial Adapter** | Connects SER2 to Arduino |
| **MTH WTIU** (50-1039) | WiFi interface to DCS track |
| **Arduino UNO Q** (ABX00162) | Runs the bridge software |
| **USB Hub with Power Delivery** | Powers the Arduino |

**Connection:**
```
Remote → Base 3 → SER2 → FTDI → Arduino UNO Q → WiFi → WTIU → Track
```

---

## Installation

### Step 1: Arduino UNO Q Setup

1. Download **Arduino App Lab** from arduino.cc
2. Connect Arduino via USB-C and open App Lab
3. Go to **Settings → Network** and connect to your WiFi

> **Important:** Arduino and WTIU must be on the same network subnet

### Step 2: Create Project in Arduino App Lab

1. Open **Arduino App Lab** and connect to your Arduino UNO Q
2. Click **New Project** and name it `lcs-to-mth-bridge`
3. App Lab will automatically create the folder structure:
   ```
   /home/arduino/ArduinoApps/lcs-to-mth-bridge/
   ├── python/    ← Python scripts go here
   └── sketch/    ← Arduino sketch goes here
   ```

4. Upload these files to the `python/` folder:
   - `main.py` (entry point for App Lab - replace the default one)
   - `lionel_mth_bridge.py` (main bridge script)
   - `bridge_config.json` (configuration file)
   - `install.sh` (installer script)
   - `lionel-mth-bridge.service` (systemd service file)

5. Upload to the `sketch/` folder:
   - `mcu_mth_handler.ino` (Arduino sketch for MCU communication)

6. In App Lab, click **Run** to flash the files to the Arduino Uno Q

### Step 3: Run the Installer

In App Lab, click the **Connect to the board's shell** button to open the board's terminal, then run:

```bash
cd /home/arduino/ArduinoApps/lcs-to-mth-bridge/python
chmod +x install.sh
./install.sh
```

The installer will:
- Install Python dependencies
- Create the configuration file
- Set up the systemd service
- Start the bridge

### Step 4: Connect Hardware

1. Disconnect the Arduino UNO Q from your computer
2. Connect the USB hub to the Arduino UNO Q via USB-C
3. Connect the power adapter to the USB hub's PD (Power Delivery) port
4. Connect the FTDI cable to the USB hub
5. Connect the FTDI cable's DB9 end to the SER2

### Step 5: Add Engines to MTH WTIU

**Before using the bridge**, you must add your MTH engines to the WTIU database using the MTH app:

1. Open the **MTH DCS app** on your phone/tablet
2. Connect to your WTIU
3. Go to **Add Engine** and follow the prompts to add each locomotive
4. Note the engine number shown in the app (e.g., "Engine 48")

> **Important:** The bridge can only control engines that are already in the WTIU database

### Step 6: Configuration (Optional)

The bridge auto-discovers MTH engines and maps them automatically. Most users won't need to change anything.

**Default config (`bridge_config.json`):**
```json
{
  "lionel_port": "/dev/ttyUSB0",
  "legacy_enabled": true,
  "mth_host": "auto",
  "mth_port": "auto"
}
```

**How engine mapping works:**
- Set your Lionel remote to the same engine number as shown in the MTH app
- Example: MTH app shows "Engine 48" → Use Lionel engine address 48
- The bridge handles the internal DCS addressing automatically

**Optional manual mapping** (only if needed):
```json
{
  "engine_mappings": {
    "10": 49
  }
}
```
This would map Lionel #10 to MTH engine 48 (use the MTH app number + 1 for the DCS value).

### Step 7: Verify It's Working

Check the service status:
```bash
sudo systemctl status lionel-mth-bridge
```

View live logs:
```bash
sudo journalctl -u lionel-mth-bridge -f
```

You should see:
```
✅ Connected to Lionel Base 3 on /dev/ttyUSB0
✅ Connected to MTH WTIU at 192.168.x.x
🎯 Monitoring Lionel Base 3 for TMCC packets...
```

---

## Verified Commands

### TMCC Mode

| Button/Control | MTH Action | Status |
|----------------|------------|--------|
| **Speed Knob** | Speed control (32-step → 0-120 sMPH) | ✅ Verified |
| **Direction** | Toggle forward/reverse | ✅ Verified |
| **Whistle** (hold) | Whistle on while held | ✅ Verified |
| **Bell** (press) | Toggle bell on/off | ✅ Verified |
| **AUX1** | Quick engine startup | ✅ Verified |
| **Keypad 2** | PFA announcements (start/advance) | ✅ Verified |
| **Keypad 5** | Quick engine shutdown | ✅ Verified |
| **Keypad 8** | Smoke off | ✅ Verified |
| **Keypad 9** | Smoke on | ✅ Verified |
| **Front Coupler** | Fire front coupler | ✅ Verified |
| **Rear Coupler** | Fire rear coupler | ✅ Verified |

### Legacy Mode

| Button/Control | MTH Action | Status |
|----------------|------------|--------|
| **Speed Knob** | 200-step speed → 0-120 sMPH (fine control) | ✅ Verified |
| **Direction** | Direct forward/reverse control | ✅ Verified |
| **Whistle Slider** | ProtoWhistle with 4-level pitch control | ✅ Verified |
| **Bell** (hold >0.5s) | Toggle bell on/off | ✅ Verified |
| **Bell** (quick press) | Single bell ring | ✅ Verified |
| **Power Button** (quick) | Quick startup | ✅ Verified |
| **Power Button** (hold) | Extended startup sequence | ✅ Verified |
| **Shutdown Button** (quick) | Quick shutdown | ✅ Verified |
| **Shutdown Button** (hold) | Extended shutdown sequence | ✅ Verified |
| **AUX1 Option 1** | Quick startup | ✅ Verified |
| **Keypad 1** | Volume up | ✅ Verified |
| **Keypad 4** | Volume down | ✅ Verified |
| **Keypad 5** | Quick shutdown | ✅ Verified |
| **Keypad 2** | PFA announcements (start/advance) | ✅ Verified |
| **Smoke Up** | Cycle smoke: off → low → med → high | ✅ Verified |
| **Smoke Down** | Cycle smoke: high → med → low → off | ✅ Verified |
| **Front Coupler** | Fire front coupler | ✅ Verified |
| **Rear Coupler** | Fire rear coupler | ✅ Verified |
| **Boost** | Increase speed | ✅ Verified |
| **Brake** | Decrease speed | ✅ Verified |

### Engines Tested

| Engine | Type | Status |
|--------|------|--------|
| **Chesapeake and Ohio Allegheny** | Steam (PS1->PS3 Upgrade) | ✅ Verified |
| **Marburger Dairy SW1200** | Diesel (PS3) | ✅ Verified |

---

## Coming Soon

- **Consist/Lashup Support** - Build and control multi-engine consists
- **Additional Device Control** - Control other devices through the MCU with additional apps (room lighting scenes, etc.)

---

## Service Commands

| Command | Description |
|---------|-------------|
| `sudo systemctl start lionel-mth-bridge` | Start the bridge |
| `sudo systemctl stop lionel-mth-bridge` | Stop the bridge |
| `sudo systemctl restart lionel-mth-bridge` | Restart after config changes |
| `sudo journalctl -u lionel-mth-bridge -f` | View live logs |

---

## Troubleshooting

**WTIU not connecting:**
- Verify WTIU is powered and on WiFi
- Check Arduino is on the same network subnet

**No response from train:**
- Verify engine is added to WTIU (use MTH app first)
- Check `engine_mappings` in config file
- View logs for error messages

**Commands not recognized:**
- Check log output for raw TMCC packets
- Verify remote is paired with Base 3

---

## Credits

- **Mark DiVecchio** - MTH WTIU protocol research ([silogic.com](http://www.silogic.com/trains/RTC_Running.html))
- **Lionel LLC** - TMCC protocol documentation

---

## License

GNU General Public License v3.0 - Copyright (c) 2026 Allen Nemetz

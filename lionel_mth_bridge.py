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
import json
import os
import queue
from queue import Queue, Empty
import struct

# PDI Protocol Constants (from pytrain-ogr)
PDI_SOP = 0xD1  # Start of Packet
PDI_EOP = 0xDF  # End of Packet
PDI_STF = 0xDE  # Stuff byte (escape)

# PDI Command Types (from PyTrain constants.py)
class PdiCommand:
    BASE_ENGINE = 0x20
    BASE_TRAIN = 0x21
    BASE_ACC = 0x22
    BASE_BASE = 0x23
    BASE_ROUTE = 0x24
    BASE_SWITCH = 0x25
    BASE_MEMORY = 0x26
    TMCC_TX = 0x27
    TMCC_RX = 0x28
    PING = 0x29

# MTH Lashup ID Range (from Mark's RTC)
# PC-controlled lashups use IDs 102-120 (101 is reserved for DCS remote)
MTH_LASHUP_MIN = 102
MTH_LASHUP_MAX = 120
MTH_LASHUP_DCS_NO = 102  # DCS engine number for lashups

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class Config:
    def __init__(self):
        self.config_file = "bridge_config.json"
        self.defaults = {
            "lionel_port": "/dev/ttyUSB0",
            "mth_host": "auto",
            "mth_port": "auto",
            "debug": False,
            "log_level": "INFO",
            "engine_mappings": {},  # Empty for auto-discovery
            "connection_settings": {
                "max_reconnect_attempts": 10,
                "connection_check_interval": 5,
                "debounce_delay": 0.5,
                "whistle_timeout": 0.3
            },
            "mth_settings": {
                "master_volume": 70,
                "volume_step": 5,
                "use_encryption": True,
                "simplified_handshake_first": True,
                "mdns_discovery": True,
                "fallback_hosts": ["192.168.0.31:33069", "192.168.0.100:33069", "192.168.0.102:33069"],
                "default_port": 33069,
                "auto_engine_mapping": True
            },
            "queue_settings": {
                "max_queue_size": 100,
                "processing_interval": 0.01
            }
        }
    
    def load(self):
        if os.path.exists(self.config_file):
            with open(self.config_file, 'r') as f:
                return {**self.defaults, **json.load(f)}
        return self.defaults
    
    def save(self, config):
        with open(self.config_file, 'w') as f:
            json.dump(config, f, indent=2)

class CommandQueue:
    def __init__(self, max_size=100):
        self.queue = Queue(maxsize=max_size)
        self.processor_thread = None
        self.running = False
        self.max_size = max_size
        self.processing_interval = 0.01  # 10ms between commands
        self.last_command_time = {}
        self.command_cooldown = 0.05  # 50ms cooldown for same command type
        
    def start(self, bridge):
        self.bridge = bridge
        self.running = True
        self.processor_thread = threading.Thread(target=self._process_queue, daemon=True)
        self.processor_thread.start()
        logger.info("🚀 Command queue started")
    
    def add_command(self, command):
        """Add command to queue with cooldown checking"""
        try:
            # Check for command cooldown to prevent flooding
            cmd_key = f"{command.get('type', 'unknown')}_{command.get('engine', 0)}"
            current_time = time.time()
            
            # Check if we should throttle this command
            if cmd_key in self.last_command_time:
                time_since_last = current_time - self.last_command_time[cmd_key]
                if time_since_last < self.command_cooldown:
                    logger.debug(f"🚦 Command throttled: {cmd_key} ({time_since_last:.3f}s ago)")
                    return False
            
            # Add to queue (will block if full)
            self.queue.put(command, timeout=0.1)
            self.last_command_time[cmd_key] = current_time
            logger.debug(f"📝 Queued command: {cmd_key}")
            return True
            
        except queue.Full:
            logger.warning("⚠️ Command queue full - dropping command")
            return False
        except Exception as e:
            logger.error(f"❌ Queue error: {e}")
            return False
    
    def stop(self):
        logger.info("🛑 Stopping command queue...")
        self.running = False
        if self.processor_thread and self.processor_thread.is_alive():
            self.processor_thread.join(timeout=1.0)
        
        # Clear remaining items
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
                self.queue.task_done()
            except:
                break
        
        logger.info("✅ Command queue stopped")
    
    def get_queue_size(self):
        return self.queue.qsize()
    
    def _process_queue(self):
        """Process commands from queue with rate limiting"""
        while self.running:
            try:
                command = self.queue.get(timeout=0.1)
                if command:
                    # Process command
                    success = self.bridge.send_to_mth(command)
                    if success:
                        logger.debug(f"✅ Processed: {command.get('type', 'unknown')}")
                    else:
                        logger.warning(f"❌ Failed to process: {command.get('type', 'unknown')}")
                    
                    self.queue.task_done()
                    
                    # Rate limiting - wait between commands
                    time.sleep(self.processing_interval)
                else:
                    continue
            except Empty:
                continue
            except Exception as e:
                logger.error(f"❌ Queue processing error: {e}")
                time.sleep(0.1)  # Prevent tight error loop

class LegacyProtocolParser:
    """Parser for Legacy (TMCC2) protocol with 200-step speed control"""
    
    def __init__(self, bridge):
        self.bridge = bridge
        self.legacy_engine_speeds = {}  # 0-199 scale
        self.legacy_directions = {}
        self.speed_high_bit = {}  # {engine: True/False} - tracks if next speed should add 128
        
    def parse_legacy_packet(self, packet):
        """Parse Legacy protocol packets (0xF8, 0xF9)
        
        NOTE: 0xFB is NOT a valid start byte - it's a continuation marker for multi-word commands.
        Multi-word commands (9 bytes) are handled separately in _parse_multiword_packet().
        """
        if len(packet) < 3:
            return None
            
        first_byte = packet[0]
        
        if first_byte == 0xF8:  # Engine commands
            return self.parse_legacy_engine_command(packet)
        elif first_byte == 0xF9:  # Train commands
            return self.parse_legacy_train_command(packet)
        # NOTE: 0xFB is handled as part of 9-byte multi-word sequences, not here
            
        return None
    
    def parse_legacy_engine_command(self, packet):
        """Parse Legacy Engine command (0xF8) - 200-step speed!"""
        if len(packet) != 3:
            return None
            
        # Combine bytes 1 and 2 (16 bits total)
        word = (packet[1] << 8) | packet[2]
        
        # Extract address (bits 15-9) and command (bits 8-0)
        address = (word >> 9) & 0x7F  # 7-bit address
        command = word & 0x1FF        # 9-bit command
        
        logger.info(f"🔍 Legacy Engine: addr={address}, cmd=0x{command:03x}")
        
        # Update current engine
        if address > 0:
            self.bridge.current_lionel_engine = address
            
        # Check bit 9 (page 8-9 spec)
        if (command >> 8) & 0x01 == 0:  # Bit 9 = 0: Speed/Momentum commands
            return self.parse_legacy_speed_command(address, command)
        else:  # Bit 9 = 1: Action commands
            return self.parse_legacy_action_command(address, command & 0xFF)
    
    def parse_legacy_speed_command(self, address, command):
        """Parse Legacy speed/momentum commands (bit 8 = 0)
        
        Per LCS-LEGACY-Protocol-Spec-v1.21:
        - 0x00-0xC7 (0-199): Set Absolute Speed Step 200 (D = 0...199)
        - 0xC8-0xCF: Set Momentum (D = 0...7)
        - 0xE0-0xE7: Brake Level (D = 0...7)  
        - 0xE8-0xEF: Boost Level (D = 0...7)
        - 0xF0-0xF7: Train Brake (D = 0...7)
        - 0xF8: Set Stall
        - 0xFB: Stop Immediate
        """
        cmd_value = command & 0xFF
        
        # Speed commands: 0x00-0xC7 (0-199)
        # Per LCS spec: 0 DDDDDDDDD = Set Absolute Speed Step 200 (D = 0...199)
        if cmd_value <= 199:
            return {
                'type': 'speed_legacy',
                'engine': address,
                'value': 'absolute',
                'speed': cmd_value,
                'scale': '200_step',
                'protocol': 'legacy'
            }
        
        # Set Momentum: 0xC8-0xCF (0 1100 1DDD)
        if 0xC8 <= cmd_value <= 0xCF:
            level = cmd_value & 0x07
            logger.info(f"🔧 Legacy Set Momentum: level {level}")
            return {'type': 'momentum', 'value': level, 'engine': address, 'protocol': 'legacy'}
        
        # Brake Level: 0xE0-0xE7 (0 1110 0DDD)
        if 0xE0 <= cmd_value <= 0xE7:
            level = cmd_value & 0x07
            logger.info(f"🔧 Legacy Brake Level: {level}")
            return {'type': 'brake_level', 'value': level, 'engine': address, 'protocol': 'legacy'}
        
        # Boost Level: 0xE8-0xEF (0 1110 1DDD)
        if 0xE8 <= cmd_value <= 0xEF:
            level = cmd_value & 0x07
            logger.info(f"🔧 Legacy Boost Level: {level}")
            return {'type': 'boost_level', 'value': level, 'engine': address, 'protocol': 'legacy'}
        
        # Train Brake: 0xF0-0xF7 (0 1111 0DDD)
        if 0xF0 <= cmd_value <= 0xF7:
            level = cmd_value & 0x07
            logger.info(f"🔧 Legacy Train Brake: {level}")
            return {'type': 'train_brake', 'value': level, 'engine': address, 'protocol': 'legacy'}
        
        # Set Stall: 0xF8 (0 1111 1000)
        if cmd_value == 0xF8:
            logger.info(f"🔧 Legacy Set Stall")
            return {'type': 'stall', 'engine': address, 'protocol': 'legacy'}
        
        # Stop Immediate: 0xFB (0 1111 1011)
        # This is sent before quick startup/shutdown - record timestamp for timing detection
        if cmd_value == 0xFB:
            logger.info(f"🛑 Legacy Stop Immediate for engine {address}")
            # Record timestamp for timing-based startup/shutdown detection
            self.bridge.last_stop_immediate_time[address] = time.time()
            return {'type': 'speed_legacy', 'engine': address, 'value': 'absolute', 'speed': 0, 'scale': '200_step', 'protocol': 'legacy'}
        
        # Unknown command in speed range
        logger.debug(f"🔧 Legacy unknown speed-range command: 0x{cmd_value:02x}")
        return None
    
    def parse_legacy_action_command(self, address, cmd_byte):
        """Parse Legacy action commands (TMCC2 Bit 9=1 commands from LCS spec)"""
        # Full Legacy command map based on LCS-LEGACY-Protocol-Spec-v1.21
        action_map = {
            # Direction Commands (DIRECT control - key Legacy advantage!)
            0x00: {'type': 'direction', 'value': 'forward'},      # 100000000 Forward Direction
            0x01: {'type': 'direction', 'value': 'toggle'},       # 100000001 Toggle Direction
            0x03: {'type': 'direction', 'value': 'reverse'},      # 100000011 Reverse Direction
            
            # Speed Commands
            0x04: {'type': 'speed', 'value': 'boost'},            # 100000100 Boost Speed
            0x07: {'type': 'speed', 'value': 'brake'},            # 100000111 Brake Speed
            
            # Coupler Commands
            0x05: {'type': 'coupler', 'value': 'front'},          # 100000101 Open Front Coupler
            0x06: {'type': 'coupler', 'value': 'rear'},           # 100000110 Open Rear Coupler
            
            # Aux1 Commands
            0x08: {'type': 'aux1', 'value': 'off'},               # 100001000 Aux1 Off
            0x09: {'type': 'aux1', 'value': 'option1'},           # 100001001 Aux1 Option 1 (Cab1 AUX1)
            0x0A: {'type': 'aux1', 'value': 'option2'},           # 100001010 Aux1 Option 2
            0x0B: {'type': 'aux1', 'value': 'on'},                # 100001011 Aux1 On
            
            # Aux2 Commands
            0x0C: {'type': 'aux2', 'value': 'off'},               # 100001100 Aux2 Off
            0x0D: {'type': 'aux2', 'value': 'option1'},           # 100001101 Aux2 Option 1 (Cab1 AUX2)
            0x0E: {'type': 'aux2', 'value': 'option2'},           # 100001110 Aux2 Option 2
            0x0F: {'type': 'aux2', 'value': 'on'},                # 100001111 Aux2 On
            
            # Numeric Commands (0x10-0x19 = Numeric 0-9)
            0x10: {'type': 'numeric', 'value': 0},                # 100010000 Numeric 0
            0x11: {'type': 'numeric', 'value': 1},                # 100010001 Numeric 1
            0x12: {'type': 'numeric', 'value': 2},                # 100010010 Numeric 2
            0x13: {'type': 'numeric', 'value': 3},                # 100010011 Numeric 3
            0x14: {'type': 'numeric', 'value': 4},                # 100010100 Numeric 4
            0x15: {'type': 'numeric', 'value': 5},                # 100010101 Numeric 5
            0x16: {'type': 'numeric', 'value': 6},                # 100010110 Numeric 6
            0x17: {'type': 'numeric', 'value': 7},                # 100010111 Numeric 7
            0x18: {'type': 'numeric', 'value': 8},                # 100011000 Numeric 8
            0x19: {'type': 'numeric', 'value': 9},                # 100011001 Numeric 9
            
            # Sound Commands
            0x1C: {'type': 'horn', 'value': 'on'},                # 100011100 Blow Horn 1
            0x1D: {'type': 'bell', 'value': 'toggle'},            # 100011101 Ring Bell
            0x1E: {'type': 'letoff', 'value': 'sound'},           # 100011110 Let-Off Sound
            0x1F: {'type': 'horn', 'value': 'secondary'},         # 100011111 Blow Horn 2
            
            # Consist/Lashup Assignment Commands
            0x20: {'type': 'consist', 'value': 'single_fwd'},     # 100100000 Single Unit Forward
            0x21: {'type': 'consist', 'value': 'single_rev'},     # 100100001 Single Unit Reverse
            0x22: {'type': 'consist', 'value': 'head_fwd'},       # 100100010 Head End Forward
            0x23: {'type': 'consist', 'value': 'head_rev'},       # 100100011 Head End Reverse
            0x24: {'type': 'consist', 'value': 'middle_fwd'},     # 100100100 Middle Unit Forward
            0x25: {'type': 'consist', 'value': 'middle_rev'},     # 100100101 Middle Unit Reverse
            0x26: {'type': 'consist', 'value': 'rear_fwd'},       # 100100110 Rear End Forward
            0x27: {'type': 'consist', 'value': 'rear_rev'},       # 100100111 Rear End Reverse
            
            # Momentum Commands
            0x28: {'type': 'momentum', 'value': 'low'},           # 100101000 Set Momentum Low
            0x29: {'type': 'momentum', 'value': 'medium'},        # 100101001 Set Momentum Medium
            0x2A: {'type': 'momentum', 'value': 'high'},          # 100101010 Set Momentum High
            0x2B: {'type': 'address', 'value': 'set'},            # 100101011 Set Engine/Train Address
            0x2C: {'type': 'consist', 'value': 'clear'},          # 100101100 Clear Consist (Lash-Up)
            0x2D: {'type': 'sound', 'value': 'refuel'},           # 100101101 Locomotive Re-Fueling Sound
            
            # Assign to Train (triggers PDI query for lashup contents)
            0x30: {'type': 'consist', 'value': 'assign_to_train'},  # 100110000 Assign to Train
            
            # Diesel Run Level (0x68-0x6F = levels 0-7)
            0x68: {'type': 'diesel_level', 'value': 0},           # 110100000 Diesel Run Level 0
            0x69: {'type': 'diesel_level', 'value': 1},           # 110100001 Diesel Run Level 1
            0x6A: {'type': 'diesel_level', 'value': 2},           # 110100010 Diesel Run Level 2
            0x6B: {'type': 'diesel_level', 'value': 3},           # 110100011 Diesel Run Level 3
            0x6C: {'type': 'diesel_level', 'value': 4},           # 110100100 Diesel Run Level 4
            0x6D: {'type': 'diesel_level', 'value': 5},           # 110100101 Diesel Run Level 5
            0x6E: {'type': 'diesel_level', 'value': 6},           # 110100110 Diesel Run Level 6
            0x6F: {'type': 'diesel_level', 'value': 7},           # 110100111 Diesel Run Level 7
            
            # RailSounds Triggers
            0x50: {'type': 'rs_trigger', 'value': 'water_injector'},  # 110101000 Water Injector
            0x51: {'type': 'rs_trigger', 'value': 'aux_air_horn'},    # 110101001 Aux Air Horn
            0x53: {'type': 'system', 'value': 'halt'},                # 110101011 System HALT
            
            # Bell Slider Position (0x54-0x57)
            0x54: {'type': 'bell_slider', 'value': 0},            # Bell Slider Position 0
            0x55: {'type': 'bell_slider', 'value': 1},            # Bell Slider Position 1
            0x56: {'type': 'bell_slider', 'value': 2},            # Bell Slider Position 2
            0x57: {'type': 'bell_slider', 'value': 3},            # Bell Slider Position 3
            
            # Engine Labor (0x70-0x7F)
            0x70: {'type': 'labor', 'value': 0},                  # Engine Labor 0
            0x71: {'type': 'labor', 'value': 1},                  # Engine Labor 1
            0x72: {'type': 'labor', 'value': 2},                  # Engine Labor 2
            0x73: {'type': 'labor', 'value': 3},                  # Engine Labor 3
            0x74: {'type': 'labor', 'value': 4},                  # Engine Labor 4
            0x75: {'type': 'labor', 'value': 5},                  # Engine Labor 5
            0x76: {'type': 'labor', 'value': 6},                  # Engine Labor 6
            0x77: {'type': 'labor', 'value': 7},                  # Engine Labor 7
            
            # Quilling Horn (0x78-0x7F)
            0x78: {'type': 'quilling_horn', 'value': 0},          # Quilling Horn Intensity 0
            0x79: {'type': 'quilling_horn', 'value': 1},          # Quilling Horn Intensity 1
            0x7A: {'type': 'quilling_horn', 'value': 2},          # Quilling Horn Intensity 2
            0x7B: {'type': 'quilling_horn', 'value': 3},          # Quilling Horn Intensity 3
            0x7C: {'type': 'quilling_horn', 'value': 4},          # Quilling Horn Intensity 4
            0x7D: {'type': 'quilling_horn', 'value': 5},          # Quilling Horn Intensity 5
            0x7E: {'type': 'quilling_horn', 'value': 6},          # Quilling Horn Intensity 6
            0x7F: {'type': 'quilling_horn', 'value': 7},          # Quilling Horn Intensity 7
            
            # Startup/Shutdown per LCS Legacy Protocol Spec:
            # 0xFB (1FB) = Start Up Sequence 1 (Delayed Prime Mover) = Extended
            # 0xFC (1FC) = Start Up Sequence 2 (Immediate Start Up) = Quick
            # 0xFD (1FD) = Shut Down Sequence 1 (Delay w/ Announcement) = Extended
            # 0xFE (1FE) = Shut Down Sequence 2 (Immediate Shut Down) = Quick
            0xFB: {'type': 'engine', 'value': 'startup_extended'},  # Delayed Prime Mover
            0xFC: {'type': 'engine', 'value': 'startup'},           # Immediate Start Up
            0xFD: {'type': 'engine', 'value': 'shutdown_extended'}, # Delay w/ Announcement
            0xFE: {'type': 'engine', 'value': 'shutdown'},          # Immediate Shut Down
            0xFF: {'type': 'engine', 'value': 'stop_immediate'},    # Stop Immediate
            
            # CAB3 Quilling Horn (0xE0-0xEF = intensity 0-15)
            0xE0: {'type': 'quilling_horn', 'value': 0},
            0xE1: {'type': 'quilling_horn', 'value': 1},
            0xE2: {'type': 'quilling_horn', 'value': 2},
            0xE3: {'type': 'quilling_horn', 'value': 3},
            0xE4: {'type': 'quilling_horn', 'value': 4},
            0xE5: {'type': 'quilling_horn', 'value': 5},
            0xE6: {'type': 'quilling_horn', 'value': 6},
            0xE7: {'type': 'quilling_horn', 'value': 7},
            0xE8: {'type': 'quilling_horn', 'value': 8},
            0xE9: {'type': 'quilling_horn', 'value': 9},
            0xEA: {'type': 'quilling_horn', 'value': 10},
            0xEB: {'type': 'quilling_horn', 'value': 11},
            0xEC: {'type': 'quilling_horn', 'value': 12},
            0xED: {'type': 'quilling_horn', 'value': 13},
            0xEE: {'type': 'quilling_horn', 'value': 14},
            0xEF: {'type': 'quilling_horn', 'value': 15},
        }
        
        # Handle relative speed commands (0x40-0x4A)
        if 0x40 <= cmd_byte <= 0x4A:
            speed_change = cmd_byte - 0x45  # 0x45 = no change, below = decrease, above = increase
            return {
                'type': 'speed',
                'value': speed_change,
                'relative': True,
                'engine': address,
                'protocol': 'legacy'
            }
        
        # Handle absolute speed 32-step (0xB0-0xCF = binary 1011DDDDD, speed 0-31)
        # Per LCS spec: "Set Absolute Speed 32 (D = 0...31)" = 1011DDDDD
        # Range is 0xB0 (D=0) to 0xCF (D=31) - NOT 0xDF!
        if 0xB0 <= cmd_byte <= 0xCF:
            speed = cmd_byte - 0xB0
            logger.info(f"🔧 Legacy 32-step speed: {speed}/31 (ignoring - use 200-step instead)")
            # Don't send 32-step speed - Legacy should use 200-step
            # Just log it and return None to ignore
            return None
        
        # Handle train assignment (0x30-0x3F = assign to train 0-15)
        if 0x30 <= cmd_byte <= 0x3F:
            train_addr = cmd_byte & 0x0F
            return {
                'type': 'train_assign',
                'value': train_addr,
                'engine': address,
                'protocol': 'legacy'
            }
        
        # Check for Parameter Index pattern (0x7C, 0x7D for multi-word commands)
        # These are NOT quilling horn - they're the start of multi-word command sequences
        # 0x7C = index 0x0C (Effects/Smoke), 0x7D = index 0x0D (Lighting)
        if cmd_byte in [0x7C, 0x7D]:
            index = cmd_byte & 0x0F
            logger.info(f"🔧 Parameter Index detected: 0x{cmd_byte:02x} (index=0x{index:02x}) - ignoring as multi-word setup")
            return {'type': 'multiword_index', 'index': index, 'engine': address, 'protocol': 'legacy'}
        
        if cmd_byte in action_map:
            cmd = action_map[cmd_byte].copy()
            cmd['engine'] = address
            cmd['protocol'] = 'legacy'
            return cmd
            
        return None
    
    def parse_multiword_command(self, packet):
        """Parse Legacy multi-word command (0xFB)
        
        Multi-word commands are 9 bytes (3 words):
        Word 1: 0xF8/0xF9, Address, Parameter Index (0x0C = Effects, 0x0D = Lighting)
        Word 2: 0xFB, Address+E/T, Parameter Data
        Word 3: 0xFB, Address+E/T, Checksum
        
        For Effects (index 0x0C):
        - 0x00 = Smoke Off
        - 0x01 = Smoke Low
        - 0x02 = Smoke Medium
        - 0x03 = Smoke High
        
        The 0xFB packet contains the Parameter Data in byte 2 (lower 4 bits)
        """
        if len(packet) < 3:
            return None
            
        # Extract address from byte 1 (bits 6-1 are address, bit 0 is E/T flag)
        address = (packet[1] >> 1) & 0x7F
        # Parameter Data is in byte 2
        param_data = packet[2] & 0x0F
        
        logger.info(f"🔧 Multi-word 0xFB: addr={address}, param_data=0x{param_data:02x}")
        
        # Check if this is a smoke command (param_data 0x00-0x03)
        # We detect smoke commands by the data value pattern
        if param_data <= 0x03:
            # This looks like a smoke level command
            smoke_levels = {0x00: 'off', 0x01: 'low', 0x02: 'med', 0x03: 'high'}
            smoke_value = smoke_levels.get(param_data, 'off')
            logger.info(f"💨 Smoke command detected: {smoke_value} for engine {address}")
            return {'type': 'smoke_direct', 'value': smoke_value, 'engine': address, 'protocol': 'legacy'}
        
        return None

    def parse_legacy_train_command(self, packet):
        """Parse Legacy Train command (0xF9) - train/lashup commands"""
        if len(packet) != 3:
            return None
        
        word = (packet[1] << 8) | packet[2]
        train_id = (word >> 9) & 0x7F
        command = word & 0x1FF
        
        logger.info(f"🚂 Train Command: TR={train_id}, cmd=0x{command:03x}")
        
        # TMCC2 consist/lashup commands (from PyTrain tmcc2_constants.py)
        # 0x12C = ASSIGN_CLEAR_CONSIST - clears the lashup
        if command == 0x12C:
            logger.info(f"🗑️ Lashup CLEAR command detected for TR{train_id}")
            return {
                'type': 'consist',
                'value': 'clear',
                'train_id': train_id,
                'protocol': 'legacy_train'
            }
        
        # 0x130 = ASSIGN_TO_TRAIN - engine assigned to train (lashup creation)
        # This triggers a PDI query to get the full lashup contents
        if command == 0x130:
            logger.info(f"🔗 Lashup ASSIGN command detected for TR{train_id}")
            return {
                'type': 'consist',
                'value': 'assign',
                'train_id': train_id,
                'protocol': 'legacy_train'
            }
        
        return {
            'type': 'train_command',
            'train_id': train_id,
            'command': command,
            'protocol': 'legacy_train'
        }

class LegacySpeedManager:
    """Manage 200-step Legacy speed with fine-grained control"""
    def __init__(self):
        self.legacy_speeds = {}  # {engine: 0-199}
        self.legacy_directions = {}
        self.last_speed_update = {}
        self.speed_resolution = 200  # Legacy has 200 steps!
        
    def set_legacy_speed(self, engine, legacy_speed):
        """Set Legacy speed (0-199) and convert to DCS (0-120)"""
        # Clamp to Legacy range
        legacy_speed = max(0, min(199, legacy_speed))
        old_speed = self.legacy_speeds.get(engine, 0)
        
        if legacy_speed != old_speed:
            self.legacy_speeds[engine] = legacy_speed
            
            # Convert Legacy 0-199 to DCS 0-120 with better precision
            dcs_speed = self.convert_legacy_to_dcs(legacy_speed)
            
            logger.info(f"🎯 Legacy Speed: Engine {engine}: {old_speed} → {legacy_speed}/199 = {dcs_speed}/120 DCS")
            return dcs_speed
            
        return None
    
    def convert_legacy_to_dcs(self, legacy_speed):
        """Convert Legacy 0-199 to DCS 0-120 sMPH with optimized mapping
        
        Legacy: 0-199 (200 steps) - finer resolution
        DCS:    0-120 sMPH (121 steps) - scale miles per hour
        
        Mapping strategy:
        - Linear mapping with rounding for best precision
        - Every ~1.65 Legacy steps = 1 DCS sMPH step
        - Preserves full range: Legacy 0 = DCS 0, Legacy 199 = DCS 120
        """
        if legacy_speed <= 0:
            return 0
        if legacy_speed >= 199:
            return 120
            
        # Direct linear mapping with proper rounding
        # Formula: dcs = round(legacy * 120 / 199)
        dcs_speed = round(legacy_speed * 120.0 / 199.0)
        
        # Ensure within bounds
        return max(0, min(120, int(dcs_speed)))
    
    def get_current_speed(self, engine):
        """Get current speed in both Legacy and DCS scales"""
        legacy = self.legacy_speeds.get(engine, 0)
        dcs = self.convert_legacy_to_dcs(legacy)
        return {'legacy': legacy, 'dcs': dcs}
    
    def handle_relative_adjustment(self, engine, change):
        """Handle relative speed adjustments in Legacy mode"""
        current = self.legacy_speeds.get(engine, 0)
        new_speed = current + change
        
        # Scale change based on current speed
        # Larger jumps at higher speeds, smaller at low speeds
        if abs(change) > 0:
            if current < 20:  # Very low speed
                effective_change = change
            elif current < 100:  # Medium speed
                effective_change = change * 2
            else:  # High speed
                effective_change = change * 3
                
            new_speed = current + effective_change
            
        return self.set_legacy_speed(engine, new_speed)

class ConsistComponent:
    """Represents an engine within a Lionel lashup (from pytrain-ogr)"""
    
    # Flag bit positions
    UNIT_TYPE_MASK = 0x03      # Bits 0-1: unit type
    DIRECTION_BIT = 0x04      # Bit 2: direction (0=fwd, 1=rev)
    TRAIN_LINK_BIT = 0x08     # Bit 3: train-link
    HORN_MASK_BIT = 0x10      # Bit 4: horn mask
    DIALOG_MASK_BIT = 0x20    # Bit 5: dialog mask
    TMCC2_BIT = 0x40          # Bit 6: TMCC2 capable
    ACCESSORY_BIT = 0x80      # Bit 7: accessory
    
    # Unit types
    SINGLE = 0
    HEAD = 1
    MIDDLE = 2
    TAIL = 3
    
    def __init__(self, tmcc_id: int, flags: int = 0):
        self.tmcc_id = tmcc_id
        self.flags = flags
    
    @classmethod
    def from_bytes(cls, data: bytes) -> list:
        """Parse 32-byte consist block into list of ConsistComponents"""
        components = []
        for i in range(0, min(32, len(data)), 2):
            if i + 1 < len(data):
                if data[i] != 0xFF and data[i + 1] != 0xFF:
                    components.insert(0, cls(tmcc_id=data[i + 1], flags=data[i]))
        return components
    
    @property
    def is_reversed(self) -> bool:
        return bool(self.flags & self.DIRECTION_BIT)
    
    @property
    def unit_type(self) -> int:
        return self.flags & self.UNIT_TYPE_MASK
    
    def __repr__(self):
        dir_str = "REV" if self.is_reversed else "FWD"
        type_names = {0: "SINGLE", 1: "HEAD", 2: "MIDDLE", 3: "TAIL"}
        return f"ConsistComponent(id={self.tmcc_id}, {type_names.get(self.unit_type, '?')}, {dir_str})"


class LashupManager:
    """Manages Lionel TR to MTH lashup ID mapping with persistent storage"""
    
    LASHUP_FILE = "lashup_mappings.json"
    
    def __init__(self, bridge):
        self.bridge = bridge
        self.tr_to_mth = {}           # {lionel_tr_id: mth_lashup_id}
        self.mth_to_tr = {}           # {mth_lashup_id: lionel_tr_id}
        self.lashup_engines = {}      # {lionel_tr_id: [engine_ids]}
        self.mth_engines_in_lashup = {}  # {lionel_tr_id: [mth_engine_ids]}
        self.engine_list_strings = {}  # {lionel_tr_id: engine_list_hex_string}
        self.lashup_created_on_wtiu = {}  # {lionel_tr_id: bool} - True if U command succeeded
        self.available_mth_ids = list(range(MTH_LASHUP_MIN, MTH_LASHUP_MAX + 1))
        self._load_mappings()
    
    def _load_mappings(self):
        """Load persistent lashup mappings from file"""
        try:
            if os.path.exists(self.LASHUP_FILE):
                with open(self.LASHUP_FILE, 'r') as f:
                    data = json.load(f)
                    self.tr_to_mth = {int(k): v for k, v in data.get('tr_to_mth', {}).items()}
                    self.mth_to_tr = {int(k): v for k, v in data.get('mth_to_tr', {}).items()}
                    self.lashup_engines = {int(k): v for k, v in data.get('lashup_engines', {}).items()}
                    self.mth_engines_in_lashup = {int(k): v for k, v in data.get('mth_engines_in_lashup', {}).items()}
                    # Update available IDs
                    used_ids = set(self.tr_to_mth.values())
                    self.available_mth_ids = [i for i in range(MTH_LASHUP_MIN, MTH_LASHUP_MAX + 1) if i not in used_ids]
                    logger.info(f"🔗 Loaded {len(self.tr_to_mth)} lashup mappings from disk")
        except Exception as e:
            logger.warning(f"⚠️ Could not load lashup mappings: {e}")
    
    def _save_mappings(self):
        """Save lashup mappings to persistent storage"""
        try:
            data = {
                'tr_to_mth': self.tr_to_mth,
                'mth_to_tr': self.mth_to_tr,
                'lashup_engines': self.lashup_engines,
                'mth_engines_in_lashup': self.mth_engines_in_lashup
            }
            with open(self.LASHUP_FILE, 'w') as f:
                json.dump(data, f, indent=2)
            logger.info(f"💾 Saved {len(self.tr_to_mth)} lashup mappings to disk")
        except Exception as e:
            logger.error(f"❌ Could not save lashup mappings: {e}")
    
    def get_mth_lashup_id(self, tr_id: int, force_new: bool = False) -> int:
        """Get or allocate MTH lashup ID for a Lionel TR ID
        
        Args:
            tr_id: Lionel TR ID
            force_new: If True, allocate a new lashup ID even if one exists
                      (used to bypass WTIU cache when recreating lashup)
        """
        if tr_id in self.tr_to_mth and not force_new:
            return self.tr_to_mth[tr_id]
        
        # If forcing new, release the old ID first
        if tr_id in self.tr_to_mth and force_new:
            old_id = self.tr_to_mth[tr_id]
            del self.mth_to_tr[old_id]
            del self.tr_to_mth[tr_id]
            # Track recycled IDs - they go to the end of the queue
            # WTIU cache should be stale by the time we cycle back to them
            if not hasattr(self, '_recycled_ids'):
                self._recycled_ids = []
            self._recycled_ids.append(old_id)
            logger.info(f"🔗 Released MTH lashup {old_id} for TR{tr_id} (recycled to end of queue)")
        
        if not self.available_mth_ids:
            # Try to use recycled IDs if available
            if hasattr(self, '_recycled_ids') and self._recycled_ids:
                mth_id = self._recycled_ids.pop(0)
                logger.info(f"🔗 Reusing recycled MTH lashup ID {mth_id}")
            else:
                logger.error("❌ No available MTH lashup IDs (102-120 all in use)")
                return None
        else:
            mth_id = self.available_mth_ids.pop(0)
        
        self.tr_to_mth[tr_id] = mth_id
        self.mth_to_tr[mth_id] = tr_id
        self._save_mappings()
        logger.info(f"🔗 Allocated MTH lashup {mth_id} for Lionel TR{tr_id}")
        return mth_id
    
    def has_mth_engines(self, engine_ids: list) -> bool:
        """Check if any engines in the list are MTH engines"""
        logger.info(f"🔍 has_mth_engines checking {engine_ids}")
        logger.info(f"🔍 engine_mappings: {self.bridge.engine_mappings}")
        logger.info(f"🔍 discovered_mth_engines: {self.bridge.discovered_mth_engines}")
        for eng_id in engine_ids:
            if str(eng_id) in self.bridge.engine_mappings:
                logger.info(f"🔍 Found {eng_id} in engine_mappings")
                return True
            if str(eng_id) in self.bridge.discovered_mth_engines:
                logger.info(f"🔍 Found {eng_id} in discovered_mth_engines")
                return True
        logger.info(f"🔍 No MTH engines found for {engine_ids}")
        return False
    
    def get_mth_engine_ids(self, engine_ids: list) -> list:
        """Get list of MTH engine IDs from Lionel engine IDs"""
        mth_ids = []
        for eng_id in engine_ids:
            if str(eng_id) in self.bridge.engine_mappings:
                mth_ids.append(self.bridge.engine_mappings[str(eng_id)])
            elif str(eng_id) in self.bridge.discovered_mth_engines:
                mth_ids.append(self.bridge.discovered_mth_engines[str(eng_id)])
        return mth_ids
    
    def update_lashup(self, tr_id: int, components: list):
        """Update lashup with consist components from Base 3
        
        Supports mixed Lionel/MTH lashups:
        - Only MTH engines are sent to WTIU for MTH lashup creation
        - Lionel engines are ignored (Base 3 handles them directly)
        - Lashup mapping is created so TR commands get forwarded to MTH engines
        """
        engine_ids = [c.tmcc_id for c in components]
        
        # Check if ANY engines in the consist are MTH
        if not self.has_mth_engines(engine_ids):
            logger.info(f"🚂 TR{tr_id} has no MTH engines, skipping MTH lashup creation")
            return False
        
        # Get only the MTH engine IDs (Lionel-only engines will be filtered out)
        mth_engine_ids = self.get_mth_engine_ids(engine_ids)
        if not mth_engine_ids:
            logger.warning(f"⚠️ No MTH engine mappings found for TR{tr_id}")
            return False
        
        # Log mixed lashup info
        lionel_only_engines = [e for e in engine_ids if str(e) not in self.bridge.engine_mappings 
                              and str(e) not in self.bridge.discovered_mth_engines]
        if lionel_only_engines:
            logger.info(f"🚂 TR{tr_id} is MIXED lashup: MTH engines {mth_engine_ids}, Lionel-only engines {lionel_only_engines}")
            logger.info(f"🚂 Base 3 will handle Lionel engines, WTIU will handle MTH engines")
        
        self.lashup_engines[tr_id] = engine_ids
        self.mth_engines_in_lashup[tr_id] = mth_engine_ids
        
        # Always force a new lashup ID to bypass WTIU cache
        # The WTIU caches lashup configurations and doesn't update them
        # when we send a new U command with the same lashup ID
        mth_lashup_id = self.get_mth_lashup_id(tr_id, force_new=True)
        if mth_lashup_id is None:
            return False
        
        # Build MTH engine list with direction flags (only MTH engines included)
        engine_list = self._build_mth_engine_list(components)
        self.engine_list_strings[tr_id] = engine_list  # Store for regular commands
        
        if not engine_list:
            logger.warning(f"⚠️ No MTH engines to include in lashup for TR{tr_id}")
            return False
        
        # DON'T send U command here - wait until startup command like Mark's RTC
        # The U command will be sent when we receive a startup command (u4/u6)
        # This matches Mark's RTC behavior where lashup is created on startup, not when building consist
        self.lashup_created_on_wtiu[tr_id] = False  # Mark as not yet created on WTIU
        
        self._save_mappings()
        if lionel_only_engines:
            logger.info(f"📋 Registered MIXED MTH lashup {mth_lashup_id} for TR{tr_id}: MTH engines {mth_engine_ids} (U command will be sent on startup)")
        else:
            logger.info(f"📋 Registered MTH lashup {mth_lashup_id} for TR{tr_id}: {engine_list} (U command will be sent on startup)")
        
        return True
    
    def _build_mth_engine_list(self, components: list) -> str:
        """Build MTH engine list string from consist components
        
        From Mark's LashUp_Selection.cpp (DCS V6 format):
        
        LashUpCommand[EngIndex++] = 0x2C;           // comma prefix
        for each engine:
            iEng++;                                  // convert to DCS Engine Number (Lionel + 1)
            if (val < 0) iEng |= 0x80;              // reverse bit
            sprintf(&LashUpCommand[EngIndex], "%02X", iEng);  // 2 ASCII hex chars
        LashUpCommand[EngIndex++] = 0xFF;           // terminator
        
        Result format: ",<hex><hex>...ÿ" where each engine is 2 ASCII hex digits
        Example: ",0D13ÿ" = comma, engine 13 (0x0D), engine 19 (0x13), 0xFF terminator
        
        U command uses LashUpEngineList + 1 (skips comma)
        Other lashup commands prepend "|" and use full list with comma
        """
        # Start with comma (0x2C) - will be skipped for U command
        engine_list = chr(0x2C)
        
        for comp in components:
            mth_ids = self.get_mth_engine_ids([comp.tmcc_id])
            if mth_ids:
                # mth_ids[0] is already the MTH engine number (Lionel + 1)
                # Per Mark's code: iEng++ converts Lionel to DCS engine number
                # But our discovered_mth_engines already stores DCS engine numbers
                dcs_id = mth_ids[0]
                if comp.is_reversed:
                    dcs_id |= 0x80  # Set high bit for reverse
                # sprintf("%02X", iEng) - 2 ASCII hex characters
                engine_list += f"{dcs_id:02X}"
                logger.info(f"🔗 Engine {comp.tmcc_id} -> DCS {dcs_id & 0x7F} {'REV' if comp.is_reversed else 'FWD'} (0x{dcs_id:02X})")
        
        # Add 0xFF terminator (required per Mark's code)
        engine_list += chr(0xFF)
        
        logger.info(f"🔗 Built engine list: {repr(engine_list)} = {' '.join(f'{ord(c):02X}' for c in engine_list)}")
        
        return engine_list
    
    def clear_lashup(self, tr_id: int) -> list:
        """Clear a lashup and free the MTH ID
        
        Returns:
            List of MTH engine IDs that were in the lashup (for sending m4 commands)
        """
        if tr_id not in self.tr_to_mth:
            return []
        
        mth_id = self.tr_to_mth[tr_id]
        
        # Get the list of MTH engines before clearing (for m4 commands)
        mth_engines = self.mth_engines_in_lashup.get(tr_id, []).copy()
        
        logger.info(f"🗑️ Clearing lashup: TR{tr_id} -> MTH {mth_id}, engines: {mth_engines}")
        
        # Free the MTH ID
        del self.tr_to_mth[tr_id]
        del self.mth_to_tr[mth_id]
        if tr_id in self.lashup_engines:
            del self.lashup_engines[tr_id]
        if tr_id in self.mth_engines_in_lashup:
            del self.mth_engines_in_lashup[tr_id]
        if tr_id in self.engine_list_strings:
            del self.engine_list_strings[tr_id]
        
        # Return ID to available pool
        self.available_mth_ids.append(mth_id)
        self.available_mth_ids.sort()
        
        self._save_mappings()
        
        return mth_engines
    
    def get_mth_id_for_tr(self, tr_id: int) -> int:
        """Get MTH lashup ID for a Lionel TR ID (if exists)"""
        return self.tr_to_mth.get(tr_id)
    
    def get_engine_list_for_tr(self, tr_id: int) -> str:
        """Get the engine list hex string for a Lionel TR ID"""
        return self.engine_list_strings.get(tr_id, "")


class PdiClient:
    """PDI client for querying Base 3 train data via SER2 serial port only"""
    
    def __init__(self, bridge):
        self.bridge = bridge
        self.pdi_lock = Lock()
    
    @staticmethod
    def _calculate_checksum_and_stuff(data: bytes) -> tuple:
        """Calculate PDI checksum and apply byte stuffing (PyTrain's exact method)
        
        Returns: (stuffed_data, checksum_byte)
        """
        byte_stream = bytearray()
        check_sum = 0
        
        for b in data:
            check_sum += b
            if b in (PDI_SOP, PDI_STF, PDI_EOP):
                # Add stuff byte and account for it in checksum
                check_sum += PDI_STF
                byte_stream.append(PDI_STF)
            byte_stream.append(b)
        
        # Two's complement checksum
        byte_sum = check_sum
        check_sum = 0xFF & (0 - check_sum)
        
        # If checksum itself needs stuffing
        if check_sum in (PDI_SOP, PDI_STF, PDI_EOP):
            byte_stream.append(PDI_STF)
            byte_sum += PDI_STF
            check_sum = 0xFF & (0 - byte_sum)
        
        return bytes(byte_stream), bytes([check_sum])
    
    @staticmethod
    def _unstuff_bytes(data: bytes) -> bytes:
        """Remove PDI byte stuffing"""
        result = bytearray()
        i = 0
        while i < len(data):
            if data[i] == PDI_STF and i + 1 < len(data):
                result.append(data[i + 1] ^ 0x80)
                i += 2
            else:
                result.append(data[i])
                i += 1
        return bytes(result)
    
    @staticmethod
    def _verify_checksum(data: bytes) -> bool:
        """Verify PDI checksum (two's complement - sum of all bytes should be 0)"""
        return (sum(data) & 0xFF) == 0
    
    def build_train_read_request(self, train_id: int) -> bytes:
        """Build PDI request to read train data from Base 3"""
        # PDI BASE_TRAIN read request
        # Format: SOP, stuffed(command + train_id + action), checksum, EOP
        # Action 0x03 = CONFIG (read)
        ACTION_CONFIG = 0x03
        payload = bytes([PdiCommand.BASE_TRAIN, train_id, ACTION_CONFIG])
        stuffed, checksum = self._calculate_checksum_and_stuff(payload)
        return bytes([PDI_SOP]) + stuffed + checksum + bytes([PDI_EOP])
    
    def query_train_data_ser2(self, train_id: int, timeout: float = 3.0) -> dict:
        """Query Base 3 for train data via SER2 serial port
        
        Returns dict with:
        - consist_flags: byte at offset 0x6F
        - consist_components: list of ConsistComponent from offset 0x70
        """
        with self.pdi_lock:
            if not self.bridge.lionel_serial or not self.bridge.lionel_serial.is_open:
                logger.error("❌ Cannot query PDI: serial not connected")
                return None
            
            try:
                # Build request
                request = self.build_train_read_request(train_id)
                logger.info(f"📡 PDI SER2: Querying train {train_id}: {request.hex()}")
                
                with self.bridge.lionel_lock:
                    # Flush input buffer
                    self.bridge.lionel_serial.reset_input_buffer()
                    
                    # Send PDI request via SER2
                    self.bridge.lionel_serial.write(request)
                    self.bridge.lionel_serial.flush()
                    
                    # Wait for response
                    start_time = time.time()
                    response_data = bytearray()
                    in_packet = False
                    
                    while time.time() - start_time < timeout:
                        if self.bridge.lionel_serial.in_waiting > 0:
                            raw_bytes = self.bridge.lionel_serial.read(self.bridge.lionel_serial.in_waiting)
                            logger.info(f"📡 PDI SER2: Received {len(raw_bytes)} bytes: {raw_bytes.hex()}")
                            
                            # Also process consist commands that may be in this data
                            self.bridge._process_consist_commands(raw_bytes)
                            
                            for b in raw_bytes:
                                if b == PDI_SOP:
                                    in_packet = True
                                    response_data = bytearray()
                                elif b == PDI_EOP and in_packet:
                                    # Complete packet received
                                    logger.info(f"📡 PDI SER2: Complete packet: {response_data.hex()}")
                                    return self._parse_train_response(response_data)
                                elif in_packet:
                                    response_data.append(b)
                        else:
                            time.sleep(0.01)
                    
                    if response_data:
                        logger.warning(f"⚠️ PDI SER2: Incomplete packet: {response_data.hex()}")
                    else:
                        logger.warning(f"⚠️ PDI SER2: Timeout - no PDI response for train {train_id}")
                    return None
                    
            except Exception as e:
                logger.error(f"❌ PDI SER2 query error: {e}")
                return None
    
    def query_train_data(self, train_id: int, timeout: float = 3.0) -> dict:
        """Query Base 3 for train data via SER2 only
        
        Note: Consist detection now primarily uses TRAIN_ADDRESS and TRAIN_UNIT
        commands parsed directly from SER2 TMCC traffic. This PDI query is a
        fallback for cases where those commands are missed.
        
        Returns dict with:
        - consist_flags: byte at offset 0x6F
        - consist_components: list of ConsistComponent from offset 0x70
        """
        # Use SER2 only - no WiFi dependency
        result = self.query_train_data_ser2(train_id, timeout)
        if result:
            return result
        
        # SER2 query failed - consist detection will rely on TRAIN_ADDRESS/TRAIN_UNIT commands
        # parsed directly from the TMCC traffic on SER2
        return None
    
    def _extract_train_packet(self, data: bytes, train_id: int) -> bytes:
        """Extract the BASE_TRAIN response packet from concatenated PDI packets
        
        Response format: D1 21 <train_id> 02 <data...> <checksum> DF
        Action 0x02 = read response (vs 0x03 = read request)
        """
        logger.info(f"📡 PDI: Looking for train {train_id} (0x{train_id:02x}) in {len(data)} bytes")
        i = 0
        while i < len(data) - 4:
            # Look for SOP + BASE_TRAIN + train_id + action 0x02 (read response)
            if data[i] == PDI_SOP:
                logger.debug(f"📡 PDI: Found SOP at {i}, next bytes: {data[i:i+5].hex()}")
            if (data[i] == PDI_SOP and 
                data[i+1] == PdiCommand.BASE_TRAIN and 
                data[i+2] == train_id and 
                data[i+3] == 0x02):
                logger.info(f"📡 PDI: Found train packet header at offset {i}")
                # Found the start of our packet - find the EOP
                for j in range(i+4, len(data)):
                    if data[j] == PDI_EOP:
                        # Check if preceded by stuff byte
                        if j > 0 and data[j-1] == PDI_STF:
                            continue  # This EOP is stuffed, keep looking
                        # Found complete packet
                        packet = data[i+1:j]  # Exclude SOP and EOP
                        logger.info(f"📡 PDI: Extracted train packet: {len(packet)} bytes")
                        return packet
            i += 1
        logger.warning(f"📡 PDI: No matching packet found for train {train_id}")
        return None
    
    def _parse_train_response(self, data: bytes) -> dict:
        """Parse PDI train response data"""
        try:
            # Unstuff the data
            unstuffed = self._unstuff_bytes(data)
            
            if len(unstuffed) < 3:
                logger.warning(f"⚠️ PDI response too short: {len(unstuffed)} bytes")
                return None
            
            # Verify checksum (two's complement - sum should be 0)
            if not self._verify_checksum(unstuffed):
                logger.warning(f"⚠️ PDI checksum failed for response: {unstuffed.hex()}")
                return None
            
            # Skip header: cmd(1) + train_id(1) + action(1) = 3 bytes, remove checksum
            record_data = unstuffed[3:-1]
            
            logger.info(f"📡 PDI: Train record {len(record_data)} bytes")
            
            result = {
                'raw_data': record_data,
                'consist_flags': None,
                'consist_components': []
            }
            
            # PyTrain offsets are from full packet, adjust by -3 for header
            # consist_flags at [69] -> offset 66 in record_data
            # consist_components at [70:102] -> offset 67:99 in record_data
            if len(record_data) > 66:
                result['consist_flags'] = record_data[66]
                logger.info(f"📡 PDI: Consist flags = 0x{result['consist_flags']:02x}")
            
            # Extract consist components (32 bytes = 16 engine slots)
            if len(record_data) >= 99:
                consist_block = record_data[67:99]
                logger.info(f"📡 PDI: Consist block: {consist_block.hex()}")
                result['consist_components'] = ConsistComponent.from_bytes(consist_block)
                logger.info(f"🔗 PDI: Found {len(result['consist_components'])} consist components")
                for comp in result['consist_components']:
                    logger.info(f"   🚂 Engine {comp.tmcc_id}: flags=0x{comp.flags:02x}")
            else:
                logger.info(f"📡 PDI: Record has {len(record_data)} bytes (need 99+ for consist)")
            
            return result
            
        except Exception as e:
            logger.error(f"❌ PDI parse error: {e}")
            return None


class LionelMTHBridge:
    def __init__(self):
        # Load configuration
        self.config = Config()
        self.settings = self.config.load()
        
        # Apply configuration settings
        self.lionel_port = self.settings.get('lionel_port', '/dev/ttyUSB0')
        self.mcu_port = self.settings.get('mcu_port', '/dev/ttymxc3')
        self.mth_host = self.settings.get('mth_host', 'auto')
        self.mth_port = self.settings.get('mth_port', 'auto')
        self.engine_mappings = self.settings.get('engine_mappings', {})
        
        # MTH discovery settings
        mth_settings = self.settings.get('mth_settings', {})
        self.mdns_discovery = mth_settings.get('mdns_discovery', True)
        self.fallback_hosts = mth_settings.get('fallback_hosts', ['192.168.0.31:33069', '192.168.0.100:33069', '192.168.0.102:33069'])
        self.default_mth_port = mth_settings.get('default_port', 33069)
        
        # Engine mapping settings
        self.engine_mappings = self.settings.get('engine_mappings', {})
        self.auto_engine_mapping = mth_settings.get('auto_engine_mapping', True)
        self.discovered_mth_engines = {}  # {lionel_addr: mth_engine}
        self.available_mth_engines = []  # List of available MTH engine numbers
        self.engine_names = {}  # {mth_engine: name} - engine names from WTIU
        self._load_engine_mappings()  # Load persisted mappings
        
        self.mth_devices = ['192.168.0.100', '192.168.0.102']
        self.lionel_serial = None
        self.mcu_serial = None
        self.mcu_connected = False
        self.mth_connected = False
        self.mth_socket = None
        # MCU connection monitoring
        self.mcu_last_heartbeat = time.time()
        self.mcu_heartbeat_interval = 10  # seconds
        self.mcu_last_ack = {}  # Track ACK responses per command type
        self.mcu_response_thread = None  # MCU response monitoring thread
        self.running = False
        self.auto_reconnect = True
        self.connection_check_interval = 5  # seconds
        self.max_reconnect_attempts = 10
        
        # Initialize command queue with configuration
        queue_settings = self.settings.get('queue_settings', {})
        max_queue_size = queue_settings.get('max_queue_size', 100)
        self.command_queue = CommandQueue(max_size=max_queue_size)
        
        # Apply configuration settings to other attributes
        connection_settings = self.settings.get('connection_settings', {})
        self.connection_check_interval = connection_settings.get('connection_check_interval', 5)
        self.max_reconnect_attempts = connection_settings.get('max_reconnect_attempts', 10)
        self.debounce_delay = connection_settings.get('debounce_delay', 0.5)
        self.whistle_timeout = connection_settings.get('whistle_timeout', 0.3)
        
        mth_settings = self.settings.get('mth_settings', {})
        self.master_volume = mth_settings.get('master_volume', 70)
        self.volume_step = mth_settings.get('volume_step', 5)
        self.use_encryption = mth_settings.get('use_encryption', True)
        self.simplified_handshake_first = mth_settings.get('simplified_handshake_first', True)
        
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
        
        # Bell and ProtoWhistle state tracking per engine
        self.bell_states = {}  # {engine: True/False} - continuous bell ringing
        self.bell_button_press_time = {}  # {engine: timestamp} - when bell button was first pressed
        self.bell_hold_triggered = {}  # {engine: True/False} - whether hold action was triggered
        self.protowhistle_states = {}  # {engine: True/False}
        self.quilling_intensity = {}  # {engine: last_intensity} for pitch tracking
        self.protowhistle_capable = {}  # {engine: True/False/None} - None = unknown, try once
        
        # Smoke state tracking per engine (Legacy cycles: off -> low -> med -> high)
        # MTH commands: abE=off, ab12=min, ab11=med, ab10=max, abF=on
        self.smoke_states = {}  # {engine: 0=off, 1=low, 2=med, 3=high}
        
        # Startup/shutdown timing: 0xFB (stop immediate) before 0xFC/0xFD = quick, alone = extended
        self.last_stop_immediate_time = {}  # {engine: timestamp} - when 0xFB was received
        
        # Extended startup/shutdown debounce - send once and ignore repeats so sequence completes
        self.last_extended_startup_time = {}  # {engine: timestamp}
        self.last_extended_shutdown_time = {}  # {engine: timestamp}
        self.extended_command_cooldown = 20  # seconds to ignore repeated extended commands
        
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
        
        # PFA state tracking (per engine)
        # States: 0=off, 1-4=announcement step (direction toggles to advance)
        self.pfa_state = {}  # {engine: current_step (0=off, 1-4=running)}
        self.pfa_direction = {}  # {engine: last_direction ('d0' or 'd1')}
        
        # WTIU session key (from H5 response)
        self.wtiu_session_key = None
        
        # WTIU TIU number (discovered from x command)
        self.wtiu_tiu_number = None
        
        # Legacy protocol support
        self.legacy_parser = LegacyProtocolParser(self)
        self.legacy_speed_manager = LegacySpeedManager()
        self.protocol_mode = 'auto'  # 'tmcc1', 'legacy', or 'auto'
        self.legacy_enabled = True
        
        # Track which engines support Legacy
        self.legacy_capable_engines = set()
        
        # Lashup management (PDI client and TR-to-MTH mapping)
        self.pdi_client = PdiClient(self)
        self.lashup_manager = LashupManager(self)
        self.pending_train_queries = set()
        self.queried_trains = set()  # TR IDs we've already queried (no need to re-query)  # Train IDs to query after lashup commands
        
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
    
    def parse_packet(self, packet):
        """Enhanced packet parser that handles both TMCC1 and Legacy"""
        if len(packet) < 3:
            return None
            
        first_byte = packet[0]
        
        # Legacy protocol packets
        # NOTE: 0xFB is NOT a valid start byte - it's a continuation marker for multi-word commands
        # 0xFB packets are handled separately as part of 9-byte multi-word sequences
        if first_byte in [0xF8, 0xF9] and self.legacy_enabled:
            logger.info(f"🔧 Legacy packet detected: 0x{first_byte:02x}")
            command = self.legacy_parser.parse_legacy_packet(packet)
            if command:
                command['protocol'] = 'legacy'
            return command
            
        # TMCC1 protocol packets
        elif first_byte == 0xFE:
            command = self.parse_tmcc_packet(packet)
            if command:
                command['protocol'] = 'tmcc1'
            return command
            
        return None
    
    def parse_tmcc_packet(self, packet):
        """Parse TMCC packet and convert to MTH command"""
        if len(packet) != 3 or packet[0] != 0xFE:
            return None
        
        # Extract TMCC packet fields correctly
        # packet[1] = bits 15-8, packet[2] = bits 7-0
        # Bit 15-14: Command type (00=Engine, 01=Train, 10=Switch, 11=Accessory/Group)
        # Bit 13-7: Address (A), Bit 6-5: Command (C), Bit 4-0: Data (D)
        
        # Extract command type (bits 15-14) from packet[1]
        cmd_type = (packet[1] >> 6) & 0x03
        
        # Ignore Switch (10) and Accessory/Group (11) commands - they're not for engines
        if cmd_type == 0x02:  # Switch command
            logger.info(f"🔀 Switch command detected - ignoring (not for MTH engines)")
            return None
        elif cmd_type == 0x03:  # Accessory/Group command
            logger.info(f"🎛️ Accessory/Group command detected - ignoring (not for MTH engines)")
            return None
        
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
            elif data_field == 0x1D:  # Bell (11101) - Toggle on first press, debounce repeats
                current_time = time.time()
                engine = self.current_lionel_engine
                
                # Debounce - only toggle if >0.5s since last bell command
                last_bell_time = self.bell_button_press_time.get(engine, 0)
                if current_time - last_bell_time > 0.5:
                    # Toggle bell state
                    self.bell_button_press_time[engine] = current_time
                    current_state = self.bell_states.get(engine, False)
                    new_state = not current_state
                    self.bell_states[engine] = new_state
                    
                    if new_state:
                        logger.info(f"🔔 Bell ON (toggle) for engine {engine}")
                        return {'type': 'bell', 'value': 'on', 'engine': engine}
                    else:
                        logger.info(f"🔔 Bell OFF (toggle) for engine {engine}")
                        return {'type': 'bell', 'value': 'off', 'engine': engine}
                
                # Debounced - ignore repeat packets
                return None
            elif data_field == 0x1E:  # (11110) - Horn 2 / Secondary whistle
                logger.info(f"🔧 DEBUG: Horn 2 / Secondary whistle detected")
                return {'type': 'function', 'value': 'horn2'}
            elif data_field == 0x1F:  # (11111) - Absolute speed 31 (max speed in TMCC1)
                logger.info(f"🔧 DEBUG: Absolute speed 31 (max) detected")
                return {'type': 'speed', 'value': 31, 'absolute': True}
            
            # Direction commands
            elif data_field in [0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6]:
                return {'type': 'direction', 'value': 'forward'}
            elif data_field in [0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE]:
                return {'type': 'direction', 'value': 'reverse'}
                
        elif cmd_field == 0x02:  # Relative speed commands (binary 10 - bit 6 set)
            if 0x00 <= data_field <= 0x1F:  # Relative speed D (0-31)
                current_time = time.time()
                last_time = self.last_command_time.get(f"{self.current_lionel_engine}_speed", 0)
                
                if current_time - last_time > 0.3:  # Debounce for speed
                    # Map data_field to speed change
                    speed_map = {0x0A: 5, 0x09: 4, 0x08: 3, 0x07: 2, 0x06: 1, 
                                 0x05: 0, 0x04: -1, 0x03: -2, 0x02: -3, 0x01: -4, 0x00: -5}
                    speed_change = speed_map.get(data_field, 0)
                    
                    self.last_command_time[f"{self.current_lionel_engine}_speed"] = current_time
                    logger.info(f"🔧 DEBUG: Relative speed change: {speed_change}")
                    return {'type': 'speed', 'value': speed_change}
                else:
                    logger.info(f"🔧 DEBUG: Speed command debounced")
                    return None
        
        return None
    
    def process_legacy_speed_command(self, command):
        """Process Legacy 200-step speed commands"""
        engine = command.get('engine', self.current_lionel_engine)
        
        if command.get('scale') == '200_step':
            # The speed is already correctly parsed as 0-199 by parse_legacy_speed_command
            # with high bit tracking, so use it directly
            legacy_speed = command.get('speed', 0)
            
            dcs_speed = self.legacy_speed_manager.set_legacy_speed(engine, legacy_speed)
            
            # Sync TMCC1 tracker (0-31) for Cab 1L dial consistency
            if not hasattr(self, '_engine_tmcc_speed'):
                self._engine_tmcc_speed = {}
            self._engine_tmcc_speed[engine] = int(legacy_speed * 31 / 199)
            
            if dcs_speed is not None:
                return {
                    'type': 'speed',
                    'engine': engine,
                    'dcs_speed': dcs_speed,
                    'legacy_speed': legacy_speed,
                    'protocol': 'legacy',
                    'resolution': '200_step'
                }
                
        return None
    
    def send_to_mth_with_legacy(self, command):
        """Send command with Legacy protocol enhancements"""
        if not command:
            return False
            
        protocol = command.get('protocol', 'tmcc1')
        engine = command.get('engine', self.current_lionel_engine)
        
        # Handle Legacy-specific commands
        if protocol == 'legacy':
            # Legacy 200-step speed commands
            if command.get('type') == 'speed_legacy':
                processed = self.process_legacy_speed_command(command)
                if processed:
                    # Send as DCS command
                    dcs_speed = processed['dcs_speed']
                    mth_cmd = f"s{dcs_speed}"
                    
                    logger.info(f"🎯 Legacy→DCS: {processed['legacy_speed']}/199 → {dcs_speed}/120")
                    return self.send_wtiu_command(mth_cmd)
                    
            # Legacy DIRECT direction commands (key advantage over TMCC1!)
            elif command.get('type') == 'direction':
                direction = command.get('value')
                if direction in ['forward', 'reverse']:
                    # Check if direction actually changed - suppress repeated commands
                    current_dir = self.engine_directions.get(engine, None)
                    if current_dir == direction:
                        # Direction hasn't changed - suppress to avoid drift sound
                        logger.debug(f"🔇 Suppressing repeated direction {direction} for engine {engine}")
                        return True  # Return success but don't send command
                    
                    # Legacy provides DIRECT direction control
                    # Update direction state immediately
                    self.legacy_speed_manager.legacy_directions[engine] = direction
                    self.engine_directions[engine] = direction
                    
                    # Send to MTH
                    mth_cmd = 'd0' if direction == 'forward' else 'd1'
                    logger.info(f"🎯 Legacy DIRECT direction: Engine {engine} → {direction} ({mth_cmd})")
                    return self.send_wtiu_command(mth_cmd)
                    
                elif direction == 'toggle':
                    # Legacy also supports toggle (fallback)
                    current_dir = self.engine_directions.get(engine, 'forward')
                    new_dir = 'reverse' if current_dir == 'forward' else 'forward'
                    self.legacy_speed_manager.legacy_directions[engine] = new_dir
                    self.engine_directions[engine] = new_dir
                    
                    mth_cmd = 'd0' if new_dir == 'forward' else 'd1'
                    logger.info(f"🎯 Legacy TOGGLE direction: Engine {engine} {current_dir} → {new_dir} ({mth_cmd})")
                    return self.send_wtiu_command(mth_cmd)
            
            # Legacy boost/brake with finer control
            elif command.get('type') == 'speed' and command.get('value') in ['boost', 'brake']:
                current_speed = self.legacy_speed_manager.get_current_speed(engine)
                legacy_speed = current_speed['legacy']
                
                if command['value'] == 'boost':
                    new_legacy = min(199, legacy_speed + 15)  # Larger boost in Legacy
                else:  # brake
                    new_legacy = max(0, legacy_speed - 15)   # Larger brake in Legacy
                    
                dcs_speed = self.legacy_speed_manager.set_legacy_speed(engine, new_legacy)
                mth_cmd = f"s{dcs_speed}"
                return self.send_wtiu_command(mth_cmd)
            
            # Legacy coupler commands
            elif command.get('type') == 'coupler':
                coupler = command.get('value')
                if coupler == 'front':
                    mth_cmd = 'c0'  # Front coupler
                elif coupler == 'rear':
                    mth_cmd = 'c1'  # Rear coupler
                else:
                    return False
                logger.info(f"🎯 Legacy coupler: Engine {engine} → {coupler} ({mth_cmd})")
                return self.send_wtiu_command(mth_cmd)
            
            # Legacy momentum commands
            elif command.get('type') == 'momentum':
                value = command.get('value')
                logger.info(f"🎯 Legacy momentum: Engine {engine} → {value}")
                if value == 'low':
                    return self.send_wtiu_command('Da4') and self.send_wtiu_command('Dd4')
                elif value == 'medium':
                    return self.send_wtiu_command('Da12') and self.send_wtiu_command('Dd12')
                elif value == 'high':
                    return self.send_wtiu_command('Da25') and self.send_wtiu_command('Dd25')
                return True
            
            # Legacy horn commands
            elif command.get('type') == 'horn':
                value = command.get('value')
                if value in ['on', 'primary']:
                    return self.send_wtiu_command('w2')
                elif value == 'secondary':
                    return self.send_wtiu_command('n243')
                elif value == 'off':
                    return self.send_wtiu_command('bFFFD')
                return True
            
            # Legacy bell commands - track state for proper toggle
            elif command.get('type') == 'bell':
                value = command.get('value')
                engine = command.get('engine', self.current_lionel_engine)
                
                if value == 'toggle':
                    # Toggle bell state
                    current_state = self.bell_states.get(engine, False)
                    if current_state:
                        self.bell_states[engine] = False
                        logger.info(f"🔔 Bell OFF (toggle) for engine {engine}")
                        return self.send_wtiu_command('bFFFB')
                    else:
                        self.bell_states[engine] = True
                        logger.info(f"🔔 Bell ON (toggle) for engine {engine}")
                        return self.send_wtiu_command('w4')
                elif value == 'on':
                    self.bell_states[engine] = True
                    return self.send_wtiu_command('w4')
                elif value == 'off':
                    self.bell_states[engine] = False
                    return self.send_wtiu_command('bFFFB')
                return True
            
            # Legacy diesel run level (g1-g8)
            elif command.get('type') == 'diesel_level':
                level = min(8, max(1, command.get('value', 0) + 1))
                return self.send_wtiu_command(f'g{level}')
            
            # Legacy labor/rev commands (r14-r17)
            elif command.get('type') == 'labor':
                level = command.get('value', 0)
                mth_cmd = 'r17' if level <= 2 else ('r15' if level <= 4 else 'r16')
                return self.send_wtiu_command(mth_cmd)
            
            # Legacy quilling horn -> MTH ProtoWhistle with pitch mapping
            # Falls back to regular whistle for PS2 engines that don't support ProtoWhistle
            elif command.get('type') == 'quilling_horn':
                intensity = command.get('value', 0)
                engine = command.get('engine', self.current_lionel_engine)
                
                # Check if engine supports ProtoWhistle (PS3+ only, not PS2)
                # None = unknown (try ProtoWhistle), True = supports, False = doesn't support
                supports_protowhistle = self.protowhistle_capable.get(engine, None)
                
                if intensity > 0:
                    if supports_protowhistle is False:
                        # Engine doesn't support ProtoWhistle - use regular whistle
                        logger.info(f"🎺 Engine {engine} (PS2) - using regular whistle")
                        return self.send_wtiu_command('w2')
                    
                    # Try ProtoWhistle (PS3+ engines)
                    if not self.protowhistle_states.get(engine, False):
                        self.protowhistle_states[engine] = True
                        # Set low pitch FIRST before enabling ProtoWhistle to avoid loud initial blast
                        self.send_wtiu_command('ab26')  # Start at low pitch
                        self.quilling_intensity[engine] = 'ab26'
                        result = self.send_wtiu_command('ab20')  # Enable ProtoWhistle
                        if result:
                            if supports_protowhistle is None:
                                self.protowhistle_capable[engine] = True
                            logger.info(f"🎺 ProtoWhistle ON for engine {engine} (starting at low pitch)")
                        else:
                            # ProtoWhistle failed - mark as not capable, use regular whistle
                            self.protowhistle_capable[engine] = False
                            self.protowhistle_states[engine] = False
                            logger.info(f"🎺 Engine {engine} doesn't support ProtoWhistle - falling back to regular whistle")
                            return self.send_wtiu_command('w2')
                    
                    # Map Legacy intensity (1-15) to MTH pitch (only if ProtoWhistle capable)
                    # Send pitch BEFORE whistle to ensure it takes effect
                    if self.protowhistle_capable.get(engine, True):
                        # Calculate pitch level (4 levels from 16 intensity values)
                        if intensity <= 3:
                            pitch_cmd = 'ab26'  # Low pitch
                        elif intensity <= 7:
                            pitch_cmd = 'ab27'  # Mid-low pitch
                        elif intensity <= 11:
                            pitch_cmd = 'ab28'  # Mid-high pitch
                        else:
                            pitch_cmd = 'ab29'  # High pitch
                        
                        # Only send pitch if it changed
                        last_pitch = self.quilling_intensity.get(engine, None)
                        if pitch_cmd != last_pitch:
                            self.quilling_intensity[engine] = pitch_cmd
                            self.send_wtiu_command(pitch_cmd)
                            logger.info(f"🎺 Quilling intensity {intensity} → {pitch_cmd}")
                    
                    # Send whistle command
                    return self.send_wtiu_command('w2')
                else:
                    # Quilling horn released - turn off whistle
                    if self.protowhistle_states.get(engine, False):
                        self.protowhistle_states[engine] = False
                        
                        # Ramp pitch down: send pitch + whistle to hear the change
                        last_pitch = self.quilling_intensity.get(engine, None)
                        if last_pitch and last_pitch != 'ab26':
                            # Ramp down through pitch levels, blowing whistle at each step
                            if last_pitch == 'ab29':
                                self.send_wtiu_command('ab28')
                                time.sleep(0.05)
                                self.send_wtiu_command('w2')  # Keep blowing
                                time.sleep(0.08)
                            if last_pitch in ['ab29', 'ab28']:
                                self.send_wtiu_command('ab27')
                                time.sleep(0.05)
                                self.send_wtiu_command('w2')  # Keep blowing
                                time.sleep(0.08)
                            self.send_wtiu_command('ab26')  # Low pitch
                            time.sleep(0.05)
                            self.send_wtiu_command('w2')  # Keep blowing at low pitch
                            time.sleep(0.15)  # Let low pitch sound longer
                            logger.info(f"🎺 Ramping pitch down for engine {engine}")
                        
                        # Clear pitch state so next use starts fresh
                        self.quilling_intensity[engine] = None
                        self.send_wtiu_command('bFFFD')  # Whistle off
                        self.send_wtiu_command('ab21')  # Disable ProtoWhistle
                        logger.info(f"🎺 ProtoWhistle OFF for engine {engine}")
                    else:
                        # Regular whistle (PS2 or unknown engine) - always turn off
                        logger.info(f"🎺 Whistle OFF for engine {engine}")
                        self.send_wtiu_command('bFFFD')
                    return True
            
            # Legacy engine startup/shutdown
            elif command.get('type') == 'engine':
                value = command.get('value')
                current_time = time.time()
                
                if value == 'startup':
                    # If extended startup is in progress, ignore quick startup
                    last_ext_start = self.last_extended_startup_time.get(engine, 0)
                    if current_time - last_ext_start < self.extended_command_cooldown:
                        logger.debug(f"🚂 Quick Startup ignored (extended in progress) for engine {engine}")
                        return True
                    # Debounce quick startup to prevent flooding WTIU
                    if not hasattr(self, '_startup_debounce'):
                        self._startup_debounce = {}
                    last_startup = self._startup_debounce.get(engine, 0)
                    if current_time - last_startup < 2.0:  # 2 second debounce
                        logger.debug(f"🚂 Quick Startup ignored (debounced) for engine {engine}")
                        return True
                    self._startup_debounce[engine] = current_time
                    logger.info(f"🚂 Quick Startup for engine {engine}")
                    return self.send_wtiu_command('u4')  # Quick startup
                elif value == 'shutdown':
                    # If extended shutdown is in progress, ignore quick shutdown
                    last_ext_shut = self.last_extended_shutdown_time.get(engine, 0)
                    if current_time - last_ext_shut < self.extended_command_cooldown:
                        logger.debug(f"🚂 Quick Shutdown ignored (extended in progress) for engine {engine}")
                        return True
                    # Debounce quick shutdown to prevent flooding WTIU
                    if not hasattr(self, '_shutdown_debounce'):
                        self._shutdown_debounce = {}
                    last_shutdown = self._shutdown_debounce.get(engine, 0)
                    if current_time - last_shutdown < 2.0:  # 2 second debounce
                        logger.debug(f"🚂 Quick Shutdown ignored (debounced) for engine {engine}")
                        return True
                    self._shutdown_debounce[engine] = current_time
                    logger.info(f"🚂 Quick Shutdown for engine {engine}")
                    return self.send_wtiu_command('u5')  # Quick shutdown
                elif value == 'startup_extended':
                    # Debounce: only send once, ignore repeats for cooldown period
                    last_ext_start = self.last_extended_startup_time.get(engine, 0)
                    if current_time - last_ext_start < self.extended_command_cooldown:
                        logger.debug(f"🚂 Extended Startup ignored (cooldown) for engine {engine}")
                        return True  # Ignore repeated command
                    self.last_extended_startup_time[engine] = current_time
                    logger.info(f"🚂 Extended Startup for engine {engine}")
                    return self.send_wtiu_command('u6')  # Extended Startup
                elif value == 'shutdown_extended':
                    # Debounce: only send once, ignore repeats for cooldown period
                    last_ext_shut = self.last_extended_shutdown_time.get(engine, 0)
                    if current_time - last_ext_shut < self.extended_command_cooldown:
                        logger.debug(f"🚂 Extended Shutdown ignored (cooldown) for engine {engine}")
                        return True  # Ignore repeated command
                    self.last_extended_shutdown_time[engine] = current_time
                    logger.info(f"🚂 Extended Shutdown for engine {engine}")
                    return self.send_wtiu_command('u7')  # Extended Shutdown
                elif value == 'startup_timed':
                    # Check if 0xFB (stop immediate) was received recently
                    last_stop = self.last_stop_immediate_time.get(engine, 0)
                    if current_time - last_stop < 0.5:  # Within 500ms = quick press
                        logger.info(f"🚂 Quick Startup (timed) for engine {engine}")
                        return self.send_wtiu_command('u4')  # Quick startup
                    else:
                        logger.info(f"🚂 Extended Startup (timed) for engine {engine}")
                        return self.send_wtiu_command('u6')  # Extended Startup
                elif value == 'shutdown_timed':
                    # Check if 0xFB (stop immediate) was received recently
                    last_stop = self.last_stop_immediate_time.get(engine, 0)
                    if current_time - last_stop < 0.5:  # Within 500ms = quick press
                        logger.info(f"🚂 Quick Shutdown (timed) for engine {engine}")
                        return self.send_wtiu_command('u5')  # Quick shutdown
                    else:
                        logger.info(f"🚂 Extended Shutdown (timed) for engine {engine}")
                        return self.send_wtiu_command('u7')  # Extended Shutdown
                elif value == 'stop_immediate':
                    # Record timestamp for timing-based startup/shutdown detection
                    self.last_stop_immediate_time[engine] = current_time
                    logger.info(f"🛑 Stop Immediate recorded for engine {engine}")
                    return self.send_wtiu_command('s0')
                elif value == 'reset':
                    return self.send_wtiu_command('F0')
                return True
            
            # Legacy smoke_direct commands from multi-word 0xFB packets (CAB3 smoke buttons)
            # These set the smoke level directly, not cycling
            elif command.get('type') == 'smoke_direct':
                value = command.get('value')
                engine = command.get('engine', self.current_lionel_engine)
                
                # Check if this is a TR ID (lashup) - forward to lashup instead
                # Cab 3 sends 0xFB packets with engine address, but if that address matches a TR ID, route to lashup
                mth_lashup_id = self.lashup_manager.get_mth_id_for_tr(engine)
                logger.debug(f"💨 smoke_direct: engine={engine}, mth_lashup_id={mth_lashup_id}, tr_to_mth={self.lashup_manager.tr_to_mth}")
                if mth_lashup_id:
                    # This is a lashup - send smoke command to lashup
                    if value == 'off':
                        logger.info(f"💨 TR{engine} smoke OFF -> MTH lashup {mth_lashup_id}")
                        return self.send_lashup_command(mth_lashup_id, "abE", engine)
                    elif value == 'low':
                        logger.info(f"💨 TR{engine} smoke LOW -> MTH lashup {mth_lashup_id}")
                        self.send_lashup_command(mth_lashup_id, "abF", engine)  # Turn on first
                        return self.send_lashup_command(mth_lashup_id, "ab12", engine)
                    elif value == 'med':
                        logger.info(f"💨 TR{engine} smoke MED -> MTH lashup {mth_lashup_id}")
                        self.send_lashup_command(mth_lashup_id, "abF", engine)  # Turn on first
                        return self.send_lashup_command(mth_lashup_id, "ab11", engine)
                    elif value == 'high':
                        logger.info(f"💨 TR{engine} smoke HIGH -> MTH lashup {mth_lashup_id}")
                        self.send_lashup_command(mth_lashup_id, "abF", engine)  # Turn on first
                        return self.send_lashup_command(mth_lashup_id, "ab10", engine)
                    return True
                
                # Single engine smoke control
                if value == 'off':
                    self.smoke_states[engine] = 0
                    logger.info(f"💨 Smoke OFF (direct) for engine {engine}")
                    return self.send_wtiu_command('abE')
                elif value == 'low':
                    self.smoke_states[engine] = 1
                    logger.info(f"💨 Smoke LOW (direct) for engine {engine}")
                    self.send_wtiu_command('abF')  # Turn on first
                    return self.send_wtiu_command('ab12')
                elif value == 'med':
                    self.smoke_states[engine] = 2
                    logger.info(f"💨 Smoke MED (direct) for engine {engine}")
                    self.send_wtiu_command('abF')  # Turn on first
                    return self.send_wtiu_command('ab11')
                elif value == 'high':
                    self.smoke_states[engine] = 3
                    logger.info(f"💨 Smoke HIGH (direct) for engine {engine}")
                    self.send_wtiu_command('abF')  # Turn on first
                    return self.send_wtiu_command('ab10')
                return True
            
            # Legacy smoke commands - track state for cycling behavior
            # Legacy cycles: Smoke ON button = off->low->med->high, Smoke OFF button = high->med->low->off
            # MTH: abE=off, ab12=min, ab11=med, ab10=max
            elif command.get('type') == 'smoke':
                value = command.get('value')
                engine = command.get('engine', self.current_lionel_engine)
                current_state = self.smoke_states.get(engine, 0)  # 0=off, 1=low, 2=med, 3=high
                
                if value == 'on' or value == 'up':
                    # Cycle up: off->low->med->high
                    new_state = min(3, current_state + 1)
                    self.smoke_states[engine] = new_state
                    
                    if new_state == 1:
                        logger.info(f"💨 Smoke LOW for engine {engine}")
                        self.send_wtiu_command('abF')  # Turn on first
                        return self.send_wtiu_command('ab12')  # Min
                    elif new_state == 2:
                        logger.info(f"💨 Smoke MED for engine {engine}")
                        return self.send_wtiu_command('ab11')  # Med
                    elif new_state == 3:
                        logger.info(f"💨 Smoke HIGH for engine {engine}")
                        return self.send_wtiu_command('ab10')  # Max
                    return True
                    
                elif value == 'off' or value == 'down':
                    # Cycle down: high->med->low->off
                    new_state = max(0, current_state - 1)
                    self.smoke_states[engine] = new_state
                    
                    if new_state == 2:
                        logger.info(f"💨 Smoke MED for engine {engine}")
                        return self.send_wtiu_command('ab11')  # Med
                    elif new_state == 1:
                        logger.info(f"💨 Smoke LOW for engine {engine}")
                        return self.send_wtiu_command('ab12')  # Min
                    elif new_state == 0:
                        logger.info(f"💨 Smoke OFF for engine {engine}")
                        return self.send_wtiu_command('abE')  # Off
                    return True
                return True
            
            # Legacy aux1 commands - option1 = startup, option2 = shutdown
            elif command.get('type') == 'aux1':
                value = command.get('value')
                if value == 'on':
                    return self.send_wtiu_command('ab3')
                elif value == 'off':
                    return self.send_wtiu_command('ab2')
                elif value == 'option1':
                    return self.send_wtiu_command('u4')  # Startup
                elif value == 'option2':
                    return self.send_wtiu_command('u5')  # Shutdown
                return True
            
            # Legacy aux2 commands - option1 (0x0D) is Cab 1L AUX2 = Headlight Toggle
            elif command.get('type') == 'aux2':
                value = command.get('value')
                engine = command.get('engine', self.current_lionel_engine)
                if value == 'on':
                    return self.send_wtiu_command('ab1')  # Headlight ON
                elif value == 'off':
                    return self.send_wtiu_command('ab0')  # Headlight OFF
                elif value == 'option1':
                    # Cab 1L AUX2 button = Headlight Toggle - track state
                    if not hasattr(self, '_engine_headlight_state'):
                        self._engine_headlight_state = {}
                    current_state = self._engine_headlight_state.get(engine, True)  # Default ON
                    new_state = not current_state
                    self._engine_headlight_state[engine] = new_state
                    mth_cmd = "ab1" if new_state else "ab0"
                    logger.info(f"💡 Engine {engine} headlight {'ON' if new_state else 'OFF'}: {mth_cmd}")
                    return self.send_wtiu_command(mth_cmd)
                elif value == 'option2':
                    return self.send_wtiu_command('abD')  # Beacon
                return True
            
            # Legacy let-off sound
            elif command.get('type') == 'letoff':
                return self.send_wtiu_command('n30')
            
            # Legacy refuel sound
            elif command.get('type') == 'sound' and command.get('value') == 'refuel':
                return self.send_wtiu_command('n55')
            
            # Legacy numeric -> special mappings for CAB3
            elif command.get('type') == 'numeric':
                num = command.get('value', 0)
                current_time = time.time()
                
                # CAB3 uses Numeric 1 for volume up (with debouncing)
                if num == 1:
                    last_vol_time = self.last_command_time.get('volume', 0)
                    if current_time - last_vol_time > 0.3:  # 300ms debounce
                        self.last_command_time['volume'] = current_time
                        self.master_volume = min(100, self.master_volume + self.volume_step)
                        logger.info(f" Legacy Numeric 1 → Volume Up: {self.master_volume}")
                        return self.send_wtiu_command(f'v0{self.master_volume}')  # v0 = master volume
                    return True  # Debounced, ignore
                    
                # CAB3 uses Numeric 4 for volume down (with debouncing)
                elif num == 4:
                    last_vol_time = self.last_command_time.get('volume', 0)
                    if current_time - last_vol_time > 0.3:  # 300ms debounce
                        self.last_command_time['volume'] = current_time
                        self.master_volume = max(0, self.master_volume - self.volume_step)
                        logger.info(f" Legacy Numeric 4 → Volume Down: {self.master_volume}")
                        return self.send_wtiu_command(f'v0{self.master_volume}')  # v0 = master volume
                    return True  # Debounced, ignore
                    
                # CAB3 uses Numeric 5 for shutdown - debounce to prevent flooding WTIU
                elif num == 5:
                    engine = command.get('engine', self.current_lionel_engine)
                    if not hasattr(self, '_shutdown_debounce'):
                        self._shutdown_debounce = {}
                    last_shutdown = self._shutdown_debounce.get(engine, 0)
                    if current_time - last_shutdown < 2.0:  # 2 second debounce
                        logger.debug(f" Legacy Numeric 5 → Shutdown (debounced)")
                        return True
                    self._shutdown_debounce[engine] = current_time
                    logger.info(f" Legacy Numeric 5 → Shutdown")
                    return self.send_wtiu_command('u5')
                    
                # CAB3 uses Numeric 2 for PFA announcements (WTIU WiFi mode)
                # First press: u1 to start, subsequent presses: m24 to advance
                # After 60 seconds of inactivity: send u0 to end, then next press starts fresh with u1
                elif num == 2:
                    engine = command.get('engine', self.current_lionel_engine)
                    last_pfa_time = self.pfa_direction.get(engine, 0)  # Timestamp of last press
                    pfa_active = self.pfa_state.get(engine, False)  # True if PFA is running
                    
                    # If PFA was active but 60+ seconds since last press, end it first
                    if pfa_active and (current_time - last_pfa_time > 60):
                        logger.info(f" PFA Timeout: Engine {engine} → u0 (inactive for 60s)")
                        self.send_wtiu_command('u0')
                        self.pfa_state[engine] = False
                        pfa_active = False
                    
                    self.pfa_direction[engine] = current_time  # Update timestamp
                    
                    if not pfa_active:
                        # Start new PFA sequence
                        self.pfa_state[engine] = True
                        logger.info(f" PFA Started: Engine {engine} → u1")
                        return self.send_wtiu_command('u1')
                    else:
                        # Advance to next announcement
                        logger.info(f" PFA Advance: Engine {engine} → m24")
                        return self.send_wtiu_command('m24')
                    
                # Other numerics -> idle sounds
                elif num in [3, 6, 7, 8, 9]:
                    return self.send_wtiu_command(f'i{num}')
                return True
            
            # Legacy relative speed - use TMCC1 32-step scale for Cab 1L dial
            elif command.get('type') == 'speed' and command.get('relative'):
                change = command.get('value', 0)
                engine = command.get('engine', self.current_lionel_engine)
                # Track in TMCC1 steps (0-31) for Cab 1L dial behavior
                if not hasattr(self, '_engine_tmcc_speed'):
                    self._engine_tmcc_speed = {}
                current_tmcc = self._engine_tmcc_speed.get(engine, 0)
                new_tmcc = max(0, min(31, current_tmcc + change))
                self._engine_tmcc_speed[engine] = new_tmcc
                # Convert TMCC1 (0-31) to MTH (0-120): each step = ~3.87 sMPH
                dcs_speed = int(new_tmcc * 120 / 31)
                logger.info(f"🎚️ Engine {engine} dial: TMCC {current_tmcc}+{change}={new_tmcc} -> MTH {dcs_speed}")
                return self.send_wtiu_command(f's{dcs_speed}')
            
            # Legacy absolute 32-step speed
            elif command.get('type') == 'speed' and command.get('absolute') and command.get('scale') == '32_step':
                speed = command.get('value', 0)
                dcs_speed = int(speed * 120 / 31)
                return self.send_wtiu_command(f's{dcs_speed}')
            
            # Legacy RailSounds triggers
            elif command.get('type') == 'rs_trigger':
                value = command.get('value')
                if value == 'water_injector':
                    return self.send_wtiu_command('w800')
                elif value == 'aux_air_horn':
                    return self.send_wtiu_command('n243')
                return True
            
            # Legacy system halt
            elif command.get('type') == 'system' and command.get('value') == 'halt':
                return self.send_wtiu_command('o0')
                
        # Fall back to original TMCC1 handling
        return self.send_to_mth(command)
    
    def enable_legacy_mode(self, engine=None):
        """Enable Legacy mode for specific engine or all"""
        if engine:
            self.legacy_capable_engines.add(engine)
            logger.info(f"✅ Legacy mode enabled for engine {engine}")
        else:
            self.legacy_enabled = True
            logger.info("✅ Legacy mode enabled globally")
    
    def get_speed_status(self, engine=None):
        """Get detailed speed status for debugging"""
        if engine is None:
            engine = self.current_lionel_engine
            
        tmcc1_speed = self.engine_speeds.get(engine, 0)
        legacy_speed = self.legacy_speed_manager.legacy_speeds.get(engine, 0)
        legacy_dcs = self.legacy_speed_manager.convert_legacy_to_dcs(legacy_speed)
        
        return {
            'engine': engine,
            'tmcc1_speed': f"{tmcc1_speed}/31",
            'legacy_speed': f"{legacy_speed}/199",
            'dcs_speed': legacy_dcs,
            'direction': self.engine_directions.get(engine, 'forward'),
            'supports_legacy': engine in self.legacy_capable_engines
        }
    
    def send_to_mcu(self, command):
        """Send command to Arduino MCU with proper 3-part format"""
        if not self.mcu_connected:
            logger.debug("MCU not connected - command not sent")
            return False
            
        try:
            with self.mcu_lock:
                # Get command type code
                cmd_type_code = self.mcu_command_types.get(command['type'], 0)
                
                # Get engine number (default to current Lionel engine)
                engine_num = self.current_lionel_engine
                
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
                
                # FIXED: Include engine number in command
                cmd_string = f"CMD:{cmd_type_code}:{engine_num}:{cmd_value}\n"
                
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
    
    def read_mcu_responses(self):
        """Read responses from MCU"""
        try:
            if hasattr(self, 'mcu_socket') and self.mcu_socket:
                # Read from socket
                self.mcu_socket.settimeout(0.1)
                try:
                    response = self.mcu_socket.recv(256).decode().strip()
                    if response:
                        self._process_mcu_response(response)
                except socket.timeout:
                    pass
                except Exception as e:
                    logger.debug(f"MCU socket read error: {e}")
                    
            elif hasattr(self, 'mcu_serial') and self.mcu_serial:
                # Read from serial
                if self.mcu_serial.in_waiting > 0:
                    response = self.mcu_serial.readline().decode().strip()
                    if response:
                        self._process_mcu_response(response)
                        
        except Exception as e:
            logger.debug(f"MCU response read error: {e}")
    
    def _process_mcu_response(self, response):
        """Process MCU response"""
        logger.info(f"MCU: {response}")
        
        if response.startswith("ACK:"):
            parts = response.split(":")
            if len(parts) >= 3:
                cmd_type = parts[1]
                engine_num = parts[2]
                logger.info(f"✅ Command acknowledged: type={cmd_type}, engine={engine_num}")
                
                # Update last ACK time for heartbeat monitoring
                self.mcu_last_heartbeat = time.time()
                
                # Track ACK per command type
                self.mcu_last_ack[cmd_type] = time.time()
                
        elif response == "HEARTBEAT":
            logger.debug("🫀 MCU heartbeat received")
            self.mcu_last_heartbeat = time.time()
            
        elif response == "RESET":
            logger.info("🔄 MCU reset notification")
            
        elif response == "TIMEOUT":
            logger.warning("⚠️ MCU timeout detected")
            
        elif response.startswith("STATUS:"):
            logger.info(f"📊 MCU Status: {response}")
            
        elif response.startswith("ERROR:"):
            logger.error(f"❌ MCU Error: {response}")
    
    def monitor_mcu_heartbeat(self):
        """Monitor MCU heartbeat"""
        logger.info("🫀 Starting MCU heartbeat monitor...")
        
        while self.running:
            try:
                # Check for missed heartbeat
                if time.time() - self.mcu_last_heartbeat > 7:  # Slightly longer than 5s interval
                    logger.warning("⚠️ MCU heartbeat missed")
                    
                    # Attempt reconnect
                    logger.info("🔄 Attempting MCU reconnect...")
                    if self.connect_mcu():
                        logger.info("✅ MCU reconnected successfully")
                    else:
                        logger.error("❌ MCU reconnect failed")
                    
                    self.mcu_last_heartbeat = time.time()
                
                # Read any pending responses
                self.read_mcu_responses()
                
                time.sleep(1)  # Check every second
                
            except Exception as e:
                logger.error(f"MCU heartbeat monitor error: {e}")
                time.sleep(1)  # Prevent tight error loop
    
    def start_mcu_monitoring(self):
        """Start MCU response and heartbeat monitoring"""
        if self.mcu_response_thread and self.mcu_response_thread.is_alive():
            return  # Already running
            
        self.mcu_response_thread = threading.Thread(target=self.monitor_mcu_heartbeat, daemon=True)
        self.mcu_response_thread.start()
        logger.info("🫀 MCU monitoring started")
    
    def simplified_handshake(self):
        """Try a simplified handshake without complex encryption"""
        try:
            # Send H5
            self.mth_socket.send(b"H5\r\n")
            h5_response = self.mth_socket.recv(256).decode('latin-1')
            logger.info(f"H5 response: {h5_response.strip()}")
            
            # Extract challenge
            if "H5" in h5_response:
                # Try sending H6 with empty or simple response first
                self.mth_socket.send(b"H600000000\r\n")
                h6_response = self.mth_socket.recv(256).decode('latin-1')
                logger.info(f"H6 response: {h6_response.strip()}")
                
                # If that doesn't work, try Mark's exact approach
                if "okay" not in h6_response.lower():
                    logger.info("⚠️ Simplified H6 failed, trying Mark's approach...")
                    return False
                else:
                    logger.info("✅ Simplified handshake successful!")
                    return True
            else:
                logger.warning("⚠️ No H5 in response")
                return False
                
        except Exception as e:
            logger.error(f"Simplified handshake failed: {e}")
            return False
    
    def safe_send_mth(self, command):
        """Send command with retry logic"""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                if not self.mth_connected:
                    self.reconnect_mth()
                
                self.send_wtiu_command(command)
                return True
                
            except (socket.error, ConnectionError) as e:
                logger.warning(f"Attempt {attempt + 1} failed: {e}")
                self.mth_connected = False
                time.sleep(0.5 * (2 ** attempt))  # Exponential backoff: 0.5s, 1s, 2s
                
                if attempt < max_retries - 1:
                    continue
                else:
                    logger.error(f"Failed after {max_retries} attempts")
                    return False
    
    def reconnect_mth(self):
        """Reconnect to MTH WTIU"""
        logger.info("🔄 Reconnecting to MTH WTIU...")
        self.mth_connected = False
        if self.mth_socket:
            self.mth_socket.close()
        return self.connect_mth()
    
    def discover_mth_engines(self):
        """Discover available MTH engines from WTIU using I0 command"""
        if not self.mth_connected or not self.mth_socket:
            logger.warning("⚠️ Not connected to WTIU - cannot discover engines")
            return False
        
        try:
            logger.info("🔍 Discovering MTH engines via I0 command...")
            
            # Don't clear discovered_mth_engines until we have new data
            # This prevents race conditions with PDI queries
            new_available_engines = []
            new_discovered_mappings = {}
            
            # CRITICAL: Flush socket buffer before I0 to avoid contaminated responses
            # Logs showed I0 getting leftover lashup command responses like "|u5,068Bÿ okay"
            with self.mth_lock:
                self.mth_socket.setblocking(False)
                try:
                    while True:
                        stale = self.mth_socket.recv(512)
                        if not stale:
                            break
                        logger.debug(f"📥 Flushed stale data before I0: {stale[:50]}")
                except (socket.error, BlockingIOError):
                    pass  # No more data to flush
                self.mth_socket.setblocking(True)
                
                # Send I0 command to get engine roster (100-bit bitmap)
                # IMPORTANT: I0 is expensive - Mark's code uses I_WAIT=10500ms (10.5 seconds)
                # because the WTIU must poll all 100 possible engines over DCS track signal
                self.mth_socket.settimeout(12.0)  # Match Mark's I_WAIT plus buffer
                self.mth_socket.send(b"I0\r\n")
                
                # Wait a moment for WTIU to process, then read response
                time.sleep(0.1)
                response = self.mth_socket.recv(512).decode('latin-1')
            logger.info(f"🔍 I0 response: {response.strip()[:100]}...")
            
            # Parse I0 response - WTIU returns hex bytes representing engine bitmap
            # Format: I0:HH,HH,HH,... okay (13 bytes, engine 1 = bit 0 of byte 0)
            if "I0" in response and "okay" in response.lower():
                try:
                    # Extract hex data between "I0:" and " okay"
                    # Response format: "I0:00,00,00,00,00,00,00,00,00,00,00,04,20 okay"
                    import re
                    hex_match = re.search(r'I0[:\s]*([\dA-Fa-f,]+)', response)
                    if hex_match:
                        hex_part = hex_match.group(1).strip()
                        hex_bytes = [h.strip() for h in hex_part.split(",") if h.strip()]
                        
                        logger.info(f"🔍 I0 hex bytes: {hex_bytes}")
                        
                        # Bitmap is reversed: rightmost bit of last byte = engine 1
                        # So we read from the end backwards
                        num_bytes = len(hex_bytes)
                        for byte_idx, hex_byte in enumerate(hex_bytes):
                            try:
                                byte_val = int(hex_byte, 16)
                                for bit in range(8):
                                    if byte_val & (1 << bit):
                                        # Engine number calculated from the END of the array
                                        # Last byte (index num_bytes-1) bit 0 = engine 1
                                        reverse_byte_idx = num_bytes - 1 - byte_idx
                                        engine_num = reverse_byte_idx * 8 + bit + 1
                                        if 1 <= engine_num <= 99:
                                            new_available_engines.append(engine_num)
                                            logger.info(f"🚂 Found engine {engine_num} (byte {byte_idx}, bit {bit}, reverse_idx {reverse_byte_idx})")
                            except ValueError:
                                continue
                except Exception as e:
                    logger.warning(f"⚠️ Failed to parse I0 response: {e}")
            
            self.mth_socket.settimeout(5.0)
            
            if new_available_engines:
                logger.info(f"✅ Found {len(new_available_engines)} MTH engines: {new_available_engines}")
                
                # Merge new engines with existing - update mapping if engine changed
                for mth_engine in new_available_engines:
                    lionel_addr = mth_engine - 1
                    if lionel_addr > 0:
                        # Skip if manually configured
                        if str(lionel_addr) in self.engine_mappings:
                            logger.debug(f"🔗 Lionel #{lionel_addr} has manual mapping, skipping")
                            continue
                        
                        # Check if already mapped to same engine
                        existing = self.discovered_mth_engines.get(str(lionel_addr))
                        if existing == mth_engine:
                            logger.debug(f"🔗 Lionel #{lionel_addr} already mapped to MTH #{mth_engine}")
                        elif existing is not None:
                            # Different engine at same address - overwrite
                            old_name = self.engine_names.get(str(existing), "Unknown")
                            self.discovered_mth_engines[str(lionel_addr)] = mth_engine
                            logger.info(f"🔗 Updated Lionel #{lionel_addr}: MTH #{existing} ({old_name}) → MTH #{mth_engine}")
                        else:
                            # New mapping
                            self.discovered_mth_engines[str(lionel_addr)] = mth_engine
                            logger.info(f"🔗 Auto-mapped Lionel #{lionel_addr} → MTH #{mth_engine}")
                
                # Merge available engines list (don't replace, add new ones)
                for eng in new_available_engines:
                    if eng not in self.available_mth_engines:
                        self.available_mth_engines.append(eng)
                
                # Query capabilities for each engine (also gets engine names)
                for dcs_engine in self.available_mth_engines:
                    self.query_engine_capabilities(dcs_engine)
                
                # Save mappings to disk
                self._save_engine_mappings()
                
                return True
            else:
                logger.warning("⚠️ No MTH engines found via I0, trying fallback...")
                for mth_engine in [6, 11]:
                    try:
                        self.mth_socket.settimeout(1.0)
                        self.mth_socket.send(f"y{mth_engine}\r\n".encode())
                        resp = self.mth_socket.recv(256).decode('latin-1')
                        if "okay" in resp.lower():
                            lionel_addr = mth_engine - 1
                            # Merge with existing - add to available list
                            if mth_engine not in self.available_mth_engines:
                                self.available_mth_engines.append(mth_engine)
                            # Update mapping if not manually configured
                            if lionel_addr > 0 and str(lionel_addr) not in self.engine_mappings:
                                existing = self.discovered_mth_engines.get(str(lionel_addr))
                                if existing != mth_engine:
                                    self.discovered_mth_engines[str(lionel_addr)] = mth_engine
                                    if existing:
                                        logger.info(f"🔗 Updated Lionel #{lionel_addr}: MTH #{existing} → MTH #{mth_engine}")
                                    else:
                                        logger.info(f"🔗 Auto-mapped Lionel #{lionel_addr} → MTH #{mth_engine}")
                            logger.info(f"🚂 Found engine {mth_engine} via fallback")
                    except:
                        continue
                self.mth_socket.settimeout(5.0)
                
                # Save mappings after fallback discovery
                if self.discovered_mth_engines:
                    self._save_engine_mappings()
                
                return len(self.available_mth_engines) > 0
                
        except Exception as e:
            logger.error(f"❌ Engine discovery failed: {e}")
            return False
    
    def query_engine_capabilities(self, dcs_engine):
        """Query engine capabilities - get engine name, type, and ProtoWhistle support"""
        try:
            # First select the engine
            self.mth_socket.settimeout(2.0)
            self.mth_socket.send(f"y{dcs_engine}\r\n".encode())
            self.mth_socket.recv(256)  # Discard response
            
            # Query engine info - need full response for capability bytes
            cmd = f"I{dcs_engine}\r\n"
            self.mth_socket.send(cmd.encode())
            response = self.mth_socket.recv(4096).decode('latin-1')
            logger.info(f"🔍 I{dcs_engine} response ({len(response)} bytes): {response.strip()[:200]}...")
            
            # Parse response: Ixx:YY;EngineName;HH,HH,...;01 okay
            # YY = engine type: 0x00/0x10=Steam, 0x05/0x85=Diesel, 0x25=Gas/Electric
            # Capability bytes: first 32 are FF (unused), then 32 bytes of actual data
            # Byte 20 of capability data (index 52) bit 3 (0x08) = ProtoWhistle
            if f"I{dcs_engine}" in response:
                parts = response.split(";")
                if len(parts) >= 2:
                    # Extract engine type from first part (Ixx:YY)
                    header = parts[0]
                    engine_name = parts[1].strip() if len(parts) > 1 else "Unknown"
                    
                    # Parse engine type
                    engine_type = 0
                    if ":" in header:
                        type_hex = header.split(":")[1].strip()
                        try:
                            engine_type = int(type_hex, 16)
                        except:
                            pass
                    
                    # Determine if steam (0x00, 0x10, 0x90) or diesel (0x05, 0x85)
                    is_steam = (engine_type & 0x0F) == 0x00
                    is_diesel = (engine_type & 0x0F) == 0x05
                    
                    # Parse capability bytes to detect ProtoWhistle
                    has_protowhistle = False
                    hex_data = None
                    for part in parts[2:]:
                        if "," in part:
                            hex_data = part.strip()
                            break
                    
                    if hex_data:
                        hex_bytes = hex_data.split(",")
                        # First 32 bytes are FF (unused), capability data starts at index 32
                        # ProtoWhistle flag is at byte 19 of capability data (index 51), bit 3 (0x08)
                        # NOT byte 20 as Mark's code suggests - actual testing shows byte 19
                        byte19_idx = 32 + 19  # = 51
                        if len(hex_bytes) > byte19_idx:
                            try:
                                byte19 = int(hex_bytes[byte19_idx].strip(), 16)
                                has_protowhistle = bool(byte19 & 0x08)
                                logger.info(f"🔍 Byte19 (idx {byte19_idx}): 0x{byte19:02X}, ProtoWhistle={has_protowhistle}")
                            except:
                                pass
                        else:
                            logger.info(f"🔍 Only {len(hex_bytes)} capability bytes, need {byte19_idx+1}")
                    
                    lionel_engine = dcs_engine - 1
                    
                    self.engine_capabilities[dcs_engine] = {
                        'name': engine_name,
                        'type': engine_type,
                        'is_steam': is_steam,
                        'is_diesel': is_diesel,
                        'protowhistle': has_protowhistle
                    }
                    
                    # Store engine name for display
                    self.engine_names[str(dcs_engine)] = engine_name
                    
                    self.protowhistle_capable[lionel_engine] = has_protowhistle
                    
                    logger.info(f"🚂 Engine {dcs_engine} ({engine_name}): type=0x{engine_type:02X}, steam={is_steam}, diesel={is_diesel}, ProtoWhistle={has_protowhistle}")
            
            self.mth_socket.settimeout(5.0)
            
        except Exception as e:
            logger.debug(f"⚠️ Failed to query capabilities for engine {dcs_engine}: {e}")
    
    ENGINE_MAPPINGS_FILE = "engine_mappings.json"
    
    def _load_engine_mappings(self):
        """Load persisted engine mappings from file"""
        try:
            if os.path.exists(self.ENGINE_MAPPINGS_FILE):
                with open(self.ENGINE_MAPPINGS_FILE, 'r') as f:
                    data = json.load(f)
                    self.discovered_mth_engines = data.get('discovered_mth_engines', {})
                    self.available_mth_engines = data.get('available_mth_engines', [])
                    self.engine_names = data.get('engine_names', {})
                    logger.info(f"🔗 Loaded {len(self.discovered_mth_engines)} engine mappings from disk")
                    for lionel, mth in self.discovered_mth_engines.items():
                        name = self.engine_names.get(str(mth), "Unknown")
                        logger.info(f"   Lionel #{lionel} → MTH #{mth} ({name})")
        except Exception as e:
            logger.warning(f"⚠️ Could not load engine mappings: {e}")
    
    def _save_engine_mappings(self):
        """Save engine mappings to persistent storage"""
        try:
            data = {
                'discovered_mth_engines': self.discovered_mth_engines,
                'available_mth_engines': self.available_mth_engines,
                'engine_names': self.engine_names
            }
            with open(self.ENGINE_MAPPINGS_FILE, 'w') as f:
                json.dump(data, f, indent=2)
            logger.info(f"💾 Saved {len(self.discovered_mth_engines)} engine mappings to disk")
        except Exception as e:
            logger.error(f"❌ Could not save engine mappings: {e}")
    
    def get_mth_engine(self, lionel_address):
        """Get MTH engine number for Lionel address
        
        DCS engine = Lionel engine + 1
        (MTH human engine number = DCS engine number - 1)
        """
        # First check manual mappings (override)
        if str(lionel_address) in self.engine_mappings:
            return self.engine_mappings[str(lionel_address)]
        
        # Then check discovered mappings
        if str(lionel_address) in self.discovered_mth_engines:
            return self.discovered_mth_engines[str(lionel_address)]
        
        # Default: DCS engine = Lionel engine + 1
        return lionel_address + 1
    
    def create_auto_engine_mapping(self):
        """Create automatic mapping from Lionel addresses to MTH engines"""
        if not self.available_mth_engines:
            return
        
        logger.info("🔧 Creating automatic engine mapping...")
        
        # Map Lionel addresses 1-99 to available MTH engines
        lionel_start = 1
        mth_index = 0
        
        for lionel_addr in range(lionel_start, 100):  # Lionel addresses 1-99
            if mth_index < len(self.available_mth_engines):
                mth_engine = self.available_mth_engines[mth_index]
                
                # Only map if not already manually configured
                if str(lionel_addr) not in self.engine_mappings:
                    self.discovered_mth_engines[str(lionel_addr)] = mth_engine
                    logger.debug(f"🔗 Auto-mapped Lionel #{lionel_addr} → MTH #{mth_engine}")
                
                mth_index += 1
    
    def _create_auto_mapping(self):
        """Create automatic mapping from Lionel addresses to MTH engines
        
        MTH DCS engine numbers are Lionel TMCC address + 1
        e.g., Lionel address 5 = MTH engine 6, Lionel address 10 = MTH engine 11
        """
        if not self.available_mth_engines:
            return
        
        logger.info("🔧 Creating automatic engine mapping...")
        
        # Map each discovered MTH engine to its corresponding Lionel address
        # MTH engine number = Lionel address + 1
        for mth_engine in self.available_mth_engines:
            lionel_addr = mth_engine - 1  # MTH 6 = Lionel 5, MTH 11 = Lionel 10
            
            # Only map if not already manually configured
            if str(lionel_addr) not in self.engine_mappings:
                self.discovered_mth_engines[str(lionel_addr)] = mth_engine
                logger.info(f"🔗 Auto-mapped Lionel #{lionel_addr} → MTH #{mth_engine}")
    
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
        
        # Use mDNS-first approach with fallback
        mth_host = None
        mth_port = None
        
        # Try mDNS discovery first if enabled
        if self.mdns_discovery and self.mth_host == 'auto':
            logger.info("🔍 Attempting MTH WTIU mDNS discovery...")
            if self.discover_wtiu_mdns():
                logger.info("✅ mDNS discovery successful")
                # Use discovered WTIU
                if hasattr(self, 'discovered_wtiu'):
                    mth_host = self.discovered_wtiu['host']
                    mth_port = self.discovered_wtiu['port']
                    logger.info(f"🎯 Using discovered WTIU: {mth_host}:{mth_port}")
            else:
                logger.info("⚠️ mDNS discovery failed, trying fallback hosts")
        
        # If mDNS failed or disabled, try fallback hosts
        if not mth_host:
            if self.mth_host != 'auto':
                # Use specific host from config (might be host:port or just host)
                if ':' in self.mth_host:
                    mth_host, mth_port = self.mth_host.split(':', 1)
                else:
                    mth_host = self.mth_host
                    mth_port = self.default_mth_port
                logger.info(f"📍 Using configured host: {mth_host}:{mth_port}")
            else:
                # Try fallback hosts in order (host:port format)
                for host_port in self.fallback_hosts:
                    if ':' in host_port:
                        mth_host, mth_port = host_port.split(':', 1)
                    else:
                        mth_host = host_port
                        mth_port = self.default_mth_port
                    logger.info(f"🔄 Trying fallback host: {mth_host}:{mth_port}")
                    break
                else:
                    logger.error("❌ No hosts available for connection")
                    return False
        
        # Convert port to integer
        if isinstance(mth_port, str):
            mth_port = int(mth_port)
        
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
            
            # Try simplified handshake first
            if self.simplified_handshake():
                logger.info("✅ Simplified handshake successful!")
                return True
            else:
                logger.warning("⚠️ Simplified handshake failed, trying Mark's approach...")

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
                    h5_response = self.mth_socket.recv(256).decode('latin-1')
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
                                    x ^= k  # x ^= k
                                    y = self.rol16(y, 2)  # ROL 2
                                    y ^= x  # y ^= x
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
                            h6_response = self.mth_socket.recv(256).decode('latin-1')
                            logger.info(f"🔍 WTIU H6 response: {h6_response.strip()}")
                    else:
                        logger.warning("⚠️ Failed to encrypt H6 key properly")
                        h6_response = ""

                    # Check for success - accept H6 response (working ESP8266 approach)
                    if "H6" in h6_response:
                        if "okay" in h6_response.lower():
                            logger.info("✅ WTIU H5/H6 handshake successful (with okay)!")
                        else:
                            logger.info("✅ WTIU H5/H6 handshake successful (without okay)!")
                            logger.warning("⚠️ H6 response missing 'okay', but proceeding anyway...")

                        # Send x and ! commands like ESP8266 code
                        logger.info("🔐 Getting TIU info (like ESP8266 code)...")

                        # Send x command to get TIU number
                        self.mth_socket.send(b"x\r\n")
                        x_response = self.mth_socket.recv(256).decode('latin-1')
                        logger.info(f"🔍 WTIU x response: {x_response.strip()}")

                        # Send ! command to get version info
                        self.mth_socket.send(b"!\r\n")
                        exclamation_response = self.mth_socket.recv(256).decode('latin-1')
                        logger.info(f"🔍 WTIU ! response: {exclamation_response.strip()}")

                        # Accept any response from x and ! commands (WTIU is responding)
                        logger.info("✅ WTIU full handshake successful!")
                        logger.info(f"🔍 x response: '{x_response.strip()}'")
                        logger.info(f"🔍 ! response: '{exclamation_response.strip()}'")

                        # Send 'y' command to establish PC connection
                        logger.info("🔐 Establishing PC connection with 'y' command...")
                        y_command = f"y11\r\n"  # Engine number 11 (Lionel Engine #10)
                        self.mth_socket.send(y_command.encode())
                        y_response = self.mth_socket.recv(256).decode('latin-1')
                        logger.info(f"🔍 WTIU y response: {y_response.strip()}")

                        # Test if connection is working
                        logger.info("🔐 Testing connection with simple command...")
                        test_command = "y11\r\n"
                        self.mth_socket.send(test_command.encode())
                        test_response = self.mth_socket.recv(256).decode('latin-1')
                        logger.info(f"🔍 Test response: {test_response.strip()}")

                        if "PC connection not available" not in test_response:
                            logger.info("✅ WTIU PC connection established successfully!")
                            
                            # Discover available MTH engines
                            self.discover_mth_engines()
                            
                            return True  # Success!
                        else:
                            logger.warning("⚠️ WTIU still reports PC connection not available")
                            continue  # Retry
                    else:
                        logger.warning(f"⚠️ H6 response missing 'H6': '{h6_response.strip()}'")
                        continue  # Retry

                except Exception as handshake_error:
                    logger.warning(f"⚠️ WTIU handshake failed: {handshake_error}")
                    if attempt < max_handshake_attempts - 1:
                        time.sleep(1)  # Wait before retry
                    else:
                        break

            logger.error("❌ All handshake attempts failed")
            return False

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
            x_response = self.mth_socket.recv(256).decode('latin-1')
            logger.info(f"🔍 x command response: {x_response.strip()}")
            
            # Parse TIU number
            match = re.search(r'x(\d)(\d)', x_response)
            if match:
                self.wtiu_tiu_number = int(match.group(1))  # 0-4
                logger.info(f"✅ Found TIU number: {self.wtiu_tiu_number + 1}")
            
            # 2. Get version
            self.mth_socket.send(b"!\r\n")
            version_response = self.mth_socket.recv(256).decode('latin-1')
            logger.info(f"🔍 ! command response: {version_response.strip()}")
            
            # 3. Send y command with engine number (like ESP8266 Sendy)
            # Map Lionel Engine #10 to WTIU Engine #11 (DCS #12)
            # Map Lionel Engine #11 to WTIU Engine #12 (DCS #13) 
            # Default to Engine #11 for Lionel Engine #10
            self.mth_socket.send(b"y12\r\n")
            y_response = self.mth_socket.recv(256).decode('latin-1')
            logger.info(f"🔍 y command response: {y_response.strip()}")
            
            logger.info("✅ WTIU setup complete - ready for commands!")
            return True
            
        except Exception as e:
            logger.error(f"❌ PC connection failed: {e}")
            return False
    
    def send_wtiu_command(self, command, engine=None):
        """Send command to WTIU in exact ESP8266 format"""
        try:
            # Select engine if specified and different from last selected
            if engine is None:
                engine = self.current_lionel_engine
            mth_engine = self.get_mth_engine(engine)
            if mth_engine and mth_engine != getattr(self, '_last_selected_engine', None):
                select_cmd = f"y{mth_engine}\r\n".encode()
                self.mth_socket.send(select_cmd)
                self._last_selected_engine = mth_engine
                logger.info(f"🎯 Selected MTH engine {mth_engine}")
                time.sleep(0.05)  # Brief delay after engine selection
            
            # Send command directly
            full_command = f"{command}\r\n".encode()
            self.mth_socket.send(full_command)
            logger.info(f"🚂 Sent to WTIU: {command}")
            
            # Wait for response with timeout
            self.mth_socket.settimeout(2.0)
            try:
                response = self.mth_socket.recv(256).decode('latin-1')
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
    
    def create_mth_lashup(self, mth_lashup_id: int, engine_list: str, max_retries: int = 20, retry_interval: float = 2.0) -> bool:
        """Create MTH lashup on WTIU with retry logic
        
        Args:
            mth_lashup_id: MTH lashup ID (102-120)
            engine_list: ASCII hex string of DCS engine numbers (e.g., "0F8A")
            max_retries: Maximum number of retry attempts (default 20)
            retry_interval: Seconds between retry attempts (default 2.0)
        
        Returns:
            True if lashup created successfully
            
        Note: Like Mark's RTC, the U command is sent on startup, not when building the consist.
        Uses retry logic because track communication can be intermittent.
        """
        if not self.mth_connected or not self.mth_socket:
            logger.warning("⚠️ Cannot create MTH lashup: WTIU not connected")
            return False
        
        # Pause periodic engine discovery during lashup creation to prevent I0 spam
        self._lashup_creation_in_progress = True
        
        try:
            for attempt in range(max_retries):
                try:
                    with self.mth_lock:
                        # Flush any pending data in the socket buffer
                        self.mth_socket.setblocking(False)
                        try:
                            while True:
                                stale = self.mth_socket.recv(256)
                                if not stale:
                                    break
                                logger.debug(f"📥 Flushed stale data: {stale}")
                        except (socket.error, BlockingIOError):
                            pass  # No more data to flush
                        self.mth_socket.setblocking(True)
                        
                        # NOTE: Removed I0 roster pre-check - Mark's code doesn't do this
                        # The I0 roster is unreliable and was blocking lashup creation
                        # Just send the U command directly like Mark's RTC does
                        
                        logger.info(f"🔗 Creating MTH lashup (attempt {attempt + 1}/{max_retries})")
                        
                        # Small delay after buffer flush before sending command
                        time.sleep(0.2)
                        
                        # From Mark's Comm_Thread.cpp:
                        # - For lashup commands (Consist >= LASHUPMIN):
                        #   sCommand_String = sCommand_String + String((char *)ptrLashUpEngineList);
                        #   if (FirstChar != 'U') sCommand_String = "|" + sCommand_String;
                        # - U command uses LashUpEngineList + 1 (skips the comma)
                        # - U command does NOT get the "|" prefix
                        #
                        # From Train_Control.cpp line 8356:
                        # Send_String_FIFO(LUNo, "U", TIU, RemoteNo, 11, VERY_LONG_WAIT, RECORD_OK, LashUpEngineList + 1, &respCode);
                        # "LashUp is always sent to the TIU as engine number 101 (DCS EngineNo #102 0x66)"
                        
                        # Skip the leading comma (0x2C) for U command
                        # engine_list format: ",<hex><hex>...ÿ" -> we want "<hex><hex>...ÿ"
                        engine_list_no_comma = engine_list[1:] if engine_list.startswith(chr(0x2C)) else engine_list
                        
                        # Build U command: "U" + engine_list_no_comma
                        lashup_cmd = f"U{engine_list_no_comma}"
                        
                        # Use latin-1 encoding to preserve raw 0xFF byte
                        full_cmd = f"{lashup_cmd}\r\n".encode('latin-1')
                        logger.info(f"🔗 Sending U command: {' '.join(f'{b:02X}' for b in full_cmd)} ({len(full_cmd)} bytes)")
                        self.mth_socket.send(full_cmd)
                        
                        # Lashup creation takes a long time - Mark's RTC uses VERY_LONG_WAIT (2 seconds)
                        # Use longer timeout to give WTIU time to communicate with all engines
                        self.mth_socket.settimeout(5.0)
                        try:
                            response = self.mth_socket.recv(256).decode('latin-1')
                            logger.info(f"📥 Lashup creation response: {response.strip()}")
                            
                            # Check for U command success - response must start with U and contain okay
                            # Other commands like |u4 or |s0 may also return okay but aren't U command responses
                            response_stripped = response.strip()
                            is_u_response = response_stripped.startswith('U') and "okay" in response.lower()
                            
                            if is_u_response:
                                # After creating lashup, send 'y' command to select head engine
                                # Mark's RTC: "apparently the WTIU requires a 'yX' command after the 'U' command"
                                if len(engine_list) >= 2:
                                    head_engine_dcs = int(engine_list[:2], 16) & 0x7F  # Remove reverse bit
                                    select_head_cmd = f"y{head_engine_dcs}\r\n"
                                    logger.info(f"🔗 Selecting head engine {head_engine_dcs} for lashup")
                                    self.mth_socket.send(select_head_cmd.encode())
                                    time.sleep(0.1)
                                    try:
                                        head_response = self.mth_socket.recv(256).decode('latin-1')
                                        logger.info(f"📥 Head engine selection response: {head_response.strip()}")
                                    except socket.timeout:
                                        pass
                                
                                logger.info(f"✅ MTH lashup {mth_lashup_id} created successfully on attempt {attempt + 1}")
                                return True
                            elif "timeout" in response.lower() or "error" in response.lower():
                                # DCS timeout or error - retry
                                logger.warning(f"⚠️ Lashup creation failed (attempt {attempt + 1}/{max_retries}): {response.strip()}")
                                if attempt < max_retries - 1:
                                    # Wait before retry - give WTIU time to recover
                                    time.sleep(retry_interval)
                                    continue
                            else:
                                logger.warning(f"⚠️ Unexpected lashup response: {response}")
                                return False
                                
                        except socket.timeout:
                            logger.warning(f"⚠️ Lashup creation timeout (attempt {attempt + 1}/{max_retries})")
                            if attempt < max_retries - 1:
                                time.sleep(retry_interval)
                                continue
                            # On final attempt, assume success (some commands don't return response)
                            return True
                            
                except Exception as e:
                    logger.error(f"❌ MTH lashup creation error (attempt {attempt + 1}/{max_retries}): {e}")
                    if attempt < max_retries - 1:
                        time.sleep(retry_interval)
                        continue
                    return False
            
            logger.error(f"❌ MTH lashup {mth_lashup_id} creation failed after {max_retries} attempts")
            return False
        finally:
            # Re-enable periodic engine discovery
            self._lashup_creation_in_progress = False
    
    def handle_lashup_command(self, command: dict) -> bool:
        """Handle lashup-related commands from Lionel
        
        Detects consist assignment commands and triggers PDI query to Base 3.
        Returns True if this was a lashup command that was handled.
        """
        cmd_type = command.get('type')
        cmd_value = command.get('value')
        engine = command.get('engine', 0)
        
        if cmd_type == 'consist':
            train_id = command.get('train_id', 0)
            logger.info(f"🔗 Lashup command detected: {cmd_value} for TR{train_id}")
            
            if cmd_value == 'clear' and train_id > 0:
                # Clear command received - break up MTH lashup properly
                if self.lashup_manager.get_mth_id_for_tr(train_id):
                    # Get engines before clearing, then send m4 to each to break up lashup
                    mth_engines = self.lashup_manager.clear_lashup(train_id)
                    logger.info(f"🗑️ Breaking up MTH lashup for TR{train_id}")
                    
                    # Send m4 (remove from lashup) then F0 (feature reset) to each engine
                    # Per Mark's RTC: after m4, send F0 to reset engine features
                    for mth_id in mth_engines:
                        lionel_id = mth_id - 1  # Convert MTH ID back to Lionel
                        logger.info(f"🔗 Sending m4 (break lashup) to engine {lionel_id} (MTH {mth_id})")
                        self.send_wtiu_command("m4", engine=lionel_id)
                        time.sleep(0.1)  # Brief delay between commands
                        logger.info(f"🔗 Sending F0 (feature reset) to engine {lionel_id} (MTH {mth_id})")
                        self.send_wtiu_command("F0", engine=lionel_id)
                        time.sleep(0.1)  # Brief delay between commands
                # Remove from queried set so we can query again after rebuild
                self.queried_trains.discard(train_id)
                # Cancel any pending query
                self.pending_train_queries.discard(train_id)
                return True
            
            if cmd_value == 'assign' and train_id > 0:
                # Assign command from train parser
                # Note: Consist detection now uses TRAIN_ADDRESS commands from TMCC traffic
                # No need for PDI query - the consist is detected automatically
                logger.info(f"🔗 Lashup assign detected for TR{train_id} (using TRAIN_ADDRESS detection)")
                return True
            
            # Position assignment commands (single/head/middle/rear) or assign_to_train
            # These come from engine commands (0xF8) with the engine being assigned
            engine = command.get('engine', 0)
            if cmd_value in ('single_fwd', 'single_rev', 'head_fwd', 'head_rev', 
                            'middle_fwd', 'middle_rev', 'rear_fwd', 'rear_rev', 
                            'assign_to_train') and engine > 0:
                # Engine is being assigned to a train - we need to find which train
                # The train ID comes in a subsequent ASSIGN_TO_TRAIN command with the train number
                # For now, just log it - the actual PDI query will be triggered by ASSIGN_TO_TRAIN
                logger.info(f"🔗 Engine {engine} position assignment: {cmd_value}")
                return True
            
            return True
        
        if cmd_type == 'train_command':
            # Train commands - forward to MTH lashup if mapping exists
            train_id = command.get('train_id', 0)
            cmd_code = command.get('command', 0)
            
            logger.debug(f"🔍 train_command: TR{train_id} cmd=0x{cmd_code:03x}")
            
            if train_id > 0:
                mth_id = self.lashup_manager.get_mth_id_for_tr(train_id)
                logger.debug(f"🔍 TR{train_id} -> mth_id={mth_id}")
                if mth_id:
                    # Forward command to MTH lashup
                    logger.info(f"🔗 Forwarding TR{train_id} cmd 0x{cmd_code:03x} to MTH lashup {mth_id}")
                    self.forward_train_command_to_mth(train_id, mth_id, command)
                elif train_id not in self.queried_trains:
                    # New TR ID detected - consist will be detected via TRAIN_ADDRESS commands
                    # No PDI query needed - TRAIN_ADDRESS detection handles this
                    if cmd_code != 0x12C:
                        logger.info(f"🔗 New TR{train_id} detected (using TRAIN_ADDRESS detection)")
            return True
        
        return False
    
    def forward_train_command_to_mth(self, train_id: int, mth_id: int, command: dict):
        """Forward a train command to the corresponding MTH lashup
        
        Args:
            train_id: Lionel TR ID
            mth_id: MTH lashup ID (102-120)
            command: Parsed command dict with 'command' field containing the raw command code
        """
        cmd_code = command.get('command', 0)
        
        # Track absolute speed for each lashup (shared between Cab 1L and Cab 3)
        if not hasattr(self, '_lashup_current_speed'):
            self._lashup_current_speed = {}
        
        # Map TMCC2 train commands to MTH lashup commands
        # Speed commands: 0x000-0x0C7 (0-199) - Cab 3 sends these as absolute speed
        if cmd_code <= 0x0C7:
            legacy_speed = cmd_code
            # Convert Legacy 200-step to DCS 120-step
            dcs_speed = int(legacy_speed * 120 / 199)
            # Also sync TMCC1 tracker (0-31) for Cab 1L dial consistency
            # TMCC1 = Legacy * 31 / 199
            tmcc_speed = int(legacy_speed * 31 / 199)
            if not hasattr(self, '_lashup_tmcc_speed'):
                self._lashup_tmcc_speed = {}
            self._lashup_tmcc_speed[train_id] = tmcc_speed
            self._lashup_current_speed[train_id] = dcs_speed
            mth_cmd = f"s{dcs_speed}"
            logger.info(f"🚂 TR{train_id} speed {legacy_speed} (TMCC {tmcc_speed}) -> MTH lashup {mth_id} speed {dcs_speed}")
            self.send_lashup_command(mth_id, mth_cmd, train_id)
            return
        
        # Direction commands
        # MTH only accepts d0 (forward) or d1 (reverse), not toggle
        # Track direction state and convert toggle to explicit direction
        if cmd_code == 0x100:  # Forward
            logger.info(f"🚂 TR{train_id} forward -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "d0", train_id)
            # Track direction state
            if not hasattr(self, '_lashup_direction_states'):
                self._lashup_direction_states = {}
            self._lashup_direction_states[train_id] = 0  # 0 = forward
            return
        if cmd_code == 0x101:  # Toggle direction (Cab 1L sends this)
            # Track direction state and send explicit forward/reverse
            # Add 500ms debounce to prevent rapid toggling
            if not hasattr(self, '_lashup_direction_states'):
                self._lashup_direction_states = {}
            if not hasattr(self, '_lashup_direction_debounce'):
                self._lashup_direction_debounce = {}
            
            import time
            now = time.time()
            dir_key = f"dir_{train_id}"
            last_toggle = self._lashup_direction_debounce.get(dir_key, 0)
            if now - last_toggle < 0.5:
                logger.debug(f"🚂 TR{train_id} direction toggle ignored (debounce)")
                return
            self._lashup_direction_debounce[dir_key] = now
            
            current_dir = self._lashup_direction_states.get(train_id, 0)  # Default forward
            new_dir = 1 - current_dir  # Toggle: 0->1, 1->0
            self._lashup_direction_states[train_id] = new_dir
            
            if new_dir == 0:
                logger.info(f"🚂 TR{train_id} toggle -> forward -> MTH lashup {mth_id}")
                self.send_lashup_command(mth_id, "d0", train_id)
            else:
                logger.info(f"🚂 TR{train_id} toggle -> reverse -> MTH lashup {mth_id}")
                self.send_lashup_command(mth_id, "d1", train_id)
            return
        if cmd_code == 0x103:  # Reverse
            logger.info(f"🚂 TR{train_id} reverse -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "d1", train_id)
            # Track direction state
            if not hasattr(self, '_lashup_direction_states'):
                self._lashup_direction_states = {}
            self._lashup_direction_states[train_id] = 1  # 1 = reverse
            return
        
        # Boost/Brake
        if cmd_code == 0x104:  # Boost
            logger.info(f"🚂 TR{train_id} boost -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "s+5", train_id)  # Relative speed increase
            return
        if cmd_code == 0x107:  # Brake
            logger.info(f"🚂 TR{train_id} brake -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "s-5", train_id)  # Relative speed decrease
            return
        
        # Relative speed commands (0x130-0x13F) - Cab 1L speed dial sends these
        # From LCS Legacy Protocol Spec:
        # D = 0x0 => -5 (max decrease)
        # D = 0x1 => -4
        # D = 0x2 => -3
        # D = 0x3 => -2
        # D = 0x4 => -1
        # D = 0x5 => 0 (neutral, no change)
        # D = 0x6 => +1
        # D = 0x7 => +2
        # D = 0x8 => +3
        # D = 0x9 => +4
        # D = 0xA => +5 (max increase)
        if 0x130 <= cmd_code <= 0x13F:
            rel_value = cmd_code & 0x0F  # 0-15
            
            # Throttle the commands - send every 100ms to match Cab 1L command rate
            # Cab 1L sends speed dial commands approximately every 100ms when dial is turned
            if not hasattr(self, '_lashup_speed_throttle'):
                self._lashup_speed_throttle = {}
            import time
            now = time.time()
            throttle_key = f"speed_{train_id}"
            last_send = self._lashup_speed_throttle.get(throttle_key, 0)
            if now - last_send < 0.1:
                return  # Silently throttle
            self._lashup_speed_throttle[throttle_key] = now
            
            # Track speed in TMCC1 units (0-31) for Cab 1L dial behavior
            # Cab 1L is designed for TMCC1's 32-step system
            # Convert TMCC1 steps to MTH: MTH = TMCC_step * 120 / 31
            if not hasattr(self, '_lashup_tmcc_speed'):
                self._lashup_tmcc_speed = {}
            
            current_tmcc_speed = self._lashup_tmcc_speed.get(train_id, 0)
            
            # Calculate speed delta: rel_value 5 = neutral, <5 = decrease, >5 = increase
            # Delta ranges from -5 (rel_value=0) to +5 (rel_value=10)
            speed_delta = rel_value - 5
            
            if speed_delta == 0:
                return  # Neutral, no change
            
            # Apply delta to TMCC1 speed (0-31 range)
            new_tmcc_speed = max(0, min(31, current_tmcc_speed + speed_delta))
            
            if new_tmcc_speed != current_tmcc_speed:
                self._lashup_tmcc_speed[train_id] = new_tmcc_speed
                
                # Convert TMCC1 speed (0-31) to MTH DCS speed (0-120)
                # Each TMCC step = ~3.87 MTH sMPH
                new_mth_speed = int(new_tmcc_speed * 120 / 31)
                self._lashup_current_speed[train_id] = new_mth_speed
                
                direction = "up" if speed_delta > 0 else "down"
                logger.info(f"🎚️ TR{train_id} dial {direction}: TMCC {current_tmcc_speed}+{speed_delta}={new_tmcc_speed} -> MTH {new_mth_speed}")
                self.send_lashup_command(mth_id, f"s{new_mth_speed}", train_id)
            return
        
        # Horn/Bell - from Mark's RTC: WHISTLE_ON="w2", WHISTLE_OFF="bFFFD", BELL_ON="w4", BELL_OFF="bFFFB"
        # Cab 1L doesn't send horn OFF - it just stops sending horn ON
        # We use a timeout to detect when whistle is released
        if cmd_code == 0x11C:  # Horn ON
            whistle_key = f"whistle_{train_id}"
            if not hasattr(self, '_lashup_whistle_timers'):
                self._lashup_whistle_timers = {}
            
            # Cancel any existing timer
            if whistle_key in self._lashup_whistle_timers:
                self._lashup_whistle_timers[whistle_key].cancel()
            
            # Check if whistle is already on - don't spam w2 commands
            if not hasattr(self, '_lashup_whistle_states'):
                self._lashup_whistle_states = {}
            
            if not self._lashup_whistle_states.get(whistle_key, False):
                logger.info(f"🚂 TR{train_id} horn ON -> MTH lashup {mth_id}")
                self.send_lashup_command(mth_id, "w2", train_id)
                self._lashup_whistle_states[whistle_key] = True
            
            # Start timer to turn off whistle after 300ms of no horn commands
            def whistle_off():
                if self._lashup_whistle_states.get(whistle_key, False):
                    logger.info(f"🚂 TR{train_id} horn OFF (timeout) -> MTH lashup {mth_id}")
                    self.send_lashup_command(mth_id, "bFFFD", train_id)
                    self._lashup_whistle_states[whistle_key] = False
            
            timer = threading.Timer(0.3, whistle_off)
            timer.daemon = True
            timer.start()
            self._lashup_whistle_timers[whistle_key] = timer
            return
        if cmd_code == 0x11F:  # Horn OFF (Legacy uses 0x11F for horn release)
            logger.info(f"🚂 TR{train_id} horn OFF -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "bFFFD", train_id)  # Whistle off from Mark's RTC
            # Clear whistle state
            if hasattr(self, '_lashup_whistle_states'):
                self._lashup_whistle_states[f"whistle_{train_id}"] = False
            return
        if cmd_code == 0x11D:  # Bell toggle - track state and send on/off with 1s debounce
            bell_key = f"bell_{train_id}"
            if not hasattr(self, '_lashup_bell_states'):
                self._lashup_bell_states = {}
            if not hasattr(self, '_lashup_bell_debounce'):
                self._lashup_bell_debounce = {}
            
            # Check debounce - ignore if less than 2 seconds since last toggle
            # Cab 1L sends rapid repeated 0x11D commands when bell button is pressed
            import time
            now = time.time()
            last_toggle = self._lashup_bell_debounce.get(bell_key, 0)
            if now - last_toggle < 2.0:
                logger.debug(f"🚂 TR{train_id} bell toggle ignored (debounce)")
                return
            self._lashup_bell_debounce[bell_key] = now
            
            # Toggle bell state
            current_state = self._lashup_bell_states.get(bell_key, False)
            new_state = not current_state
            self._lashup_bell_states[bell_key] = new_state
            
            if new_state:
                logger.info(f"🚂 TR{train_id} bell ON -> MTH lashup {mth_id}")
                self.send_lashup_command(mth_id, "w4", train_id)  # Bell ON
            else:
                logger.info(f"🚂 TR{train_id} bell OFF -> MTH lashup {mth_id}")
                self.send_lashup_command(mth_id, "bFFFB", train_id)  # Bell OFF
            return
        
        # Keypad buttons (from Cab 1L testing)
        # Volume control - MTH uses absolute volume: v<type><value> where type=0 (master), value=0-100
        # Track volume state and adjust by 10% per button press
        if not hasattr(self, '_lashup_volume'):
            self._lashup_volume = {}
        
        # 0x111 = Button 1 (Volume Up)
        if cmd_code == 0x111:
            current_vol = self._lashup_volume.get(train_id, 50)  # Default 50%
            new_vol = min(100, current_vol + 10)
            self._lashup_volume[train_id] = new_vol
            logger.info(f"🚂 TR{train_id} volume up -> MTH lashup {mth_id} vol {new_vol}%")
            self.send_lashup_command(mth_id, f"v0{new_vol:03d}", train_id)  # Master volume
            return
        # 0x114 = Button 4 (Volume Down)
        if cmd_code == 0x114:
            current_vol = self._lashup_volume.get(train_id, 50)  # Default 50%
            new_vol = max(0, current_vol - 10)
            self._lashup_volume[train_id] = new_vol
            logger.info(f"🚂 TR{train_id} volume down -> MTH lashup {mth_id} vol {new_vol}%")
            self.send_lashup_command(mth_id, f"v0{new_vol:03d}", train_id)  # Master volume
            return
        # 0x115 = Button 5 (Quick Shutdown) - debounce to prevent flooding
        if cmd_code == 0x115:
            import time
            if not hasattr(self, '_lashup_shutdown_debounce'):
                self._lashup_shutdown_debounce = {}
            last_shutdown = self._lashup_shutdown_debounce.get(train_id, 0)
            if time.time() - last_shutdown < 2.0:  # 2 second debounce
                return
            self._lashup_shutdown_debounce[train_id] = time.time()
            logger.info(f"🚂 TR{train_id} quick shutdown (btn 5) -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "u5", train_id)
            return
        # 0x109 = AUX1 (Quick Startup) - debounce to prevent flooding
        if cmd_code == 0x109:
            import time
            if not hasattr(self, '_lashup_startup_debounce'):
                self._lashup_startup_debounce = {}
            last_startup = self._lashup_startup_debounce.get(train_id, 0)
            if time.time() - last_startup < 2.0:  # 2 second debounce
                return
            self._lashup_startup_debounce[train_id] = time.time()
            logger.info(f"🚂 TR{train_id} quick startup (AUX1) -> MTH lashup {mth_id}")
            # Send U command to create lashup on WTIU (like Mark's RTC does on startup)
            self._ensure_lashup_created_on_wtiu(train_id, mth_id)
            self.send_lashup_command(mth_id, "u4", train_id)
            return
        # 0x105 = Rear Coupler (Lionel) -> c0 (MTH front coupler fires rear on consist)
        if cmd_code == 0x105:
            logger.info(f"🚂 TR{train_id} rear coupler -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "c0", train_id)  # Swapped: c0 fires rear on consist
            return
        # 0x106 = Front Coupler (Lionel) -> c1 (MTH rear coupler fires front on consist)
        if cmd_code == 0x106:
            logger.info(f"🚂 TR{train_id} front coupler -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "c1", train_id)  # Swapped: c1 fires front on consist
            return
        # 0x10D = AUX2 (Headlight Toggle) - MTH uses ab1 (ON) / ab0 (OFF)
        if cmd_code == 0x10D:
            # Debounce: 500ms cooldown to prevent rapid toggling
            import time
            current_time = time.time()
            if not hasattr(self, '_lashup_headlight_debounce'):
                self._lashup_headlight_debounce = {}
            last_time = self._lashup_headlight_debounce.get(train_id, 0)
            if current_time - last_time < 0.5:  # 500ms debounce
                return  # Ignore repeated command
            self._lashup_headlight_debounce[train_id] = current_time
            
            # Track headlight state per lashup
            if not hasattr(self, '_lashup_headlight_state'):
                self._lashup_headlight_state = {}
            current_state = self._lashup_headlight_state.get(train_id, True)  # Default ON
            new_state = not current_state
            self._lashup_headlight_state[train_id] = new_state
            mth_cmd = "ab1" if new_state else "ab0"
            logger.info(f"🚂 TR{train_id} headlight {'ON' if new_state else 'OFF'} (AUX2) -> MTH lashup {mth_id}: {mth_cmd}")
            self.send_lashup_command(mth_id, mth_cmd, train_id)
            return
        # 0x110 = Button 0 (Engine Reset)
        if cmd_code == 0x110:
            logger.info(f"🚂 TR{train_id} engine reset (btn 0) -> MTH lashup {mth_id}")
            # No direct MTH equivalent for engine reset in lashup mode
            return
        # 0x118 = Button 8 (Smoke Down)
        if cmd_code == 0x118:
            logger.info(f"🚂 TR{train_id} smoke down (btn 8) -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "abE", train_id)  # Smoke down/off
            return
        # 0x119 = Button 9 (Smoke Up)
        if cmd_code == 0x119:
            logger.info(f"🚂 TR{train_id} smoke up (btn 9) -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "abF", train_id)  # Smoke up/on
            return
        
        # Volume control (0x1B0-0x1BF) - master volume (from speed dial)
        if 0x1B0 <= cmd_code <= 0x1BF:
            vol_level = cmd_code & 0x0F
            # DCS volume is 0-8, Legacy is 0-15, so scale
            dcs_vol = min(8, vol_level // 2)
            logger.info(f"🚂 TR{train_id} volume {vol_level} -> MTH lashup {mth_id} vol {dcs_vol}")
            self.send_lashup_command(mth_id, f"v{dcs_vol:02d}00", train_id)  # Master volume
            return
        
        # Startup/Shutdown sequences (from menu/extended) - use same debounce as quick startup
        if cmd_code == 0x1FB:  # Startup seq 1 (extended startup from menu)
            import time
            if not hasattr(self, '_lashup_startup_debounce'):
                self._lashup_startup_debounce = {}
            last_startup = self._lashup_startup_debounce.get(train_id, 0)
            if time.time() - last_startup < 2.0:  # 2 second debounce
                return
            self._lashup_startup_debounce[train_id] = time.time()
            logger.info(f"🚂 TR{train_id} startup seq 1 -> MTH lashup {mth_id}")
            # Send U command to create lashup on WTIU (like Mark's RTC does on startup)
            self._ensure_lashup_created_on_wtiu(train_id, mth_id)
            self.send_lashup_command(mth_id, "u4", train_id)
            return
        if cmd_code == 0x1FC:  # Startup seq 2 (extended startup)
            import time
            if not hasattr(self, '_lashup_startup_debounce'):
                self._lashup_startup_debounce = {}
            last_startup = self._lashup_startup_debounce.get(train_id, 0)
            if time.time() - last_startup < 2.0:  # 2 second debounce
                return
            self._lashup_startup_debounce[train_id] = time.time()
            logger.info(f"🚂 TR{train_id} extended startup -> MTH lashup {mth_id}")
            # Send U command to create lashup on WTIU (like Mark's RTC does on startup)
            self._ensure_lashup_created_on_wtiu(train_id, mth_id)
            self.send_lashup_command(mth_id, "u6", train_id)
            return
        if cmd_code == 0x1FD:  # Shutdown seq 1 (from menu)
            import time
            if not hasattr(self, '_lashup_shutdown_debounce'):
                self._lashup_shutdown_debounce = {}
            last_shutdown = self._lashup_shutdown_debounce.get(train_id, 0)
            if time.time() - last_shutdown < 2.0:  # 2 second debounce
                return
            self._lashup_shutdown_debounce[train_id] = time.time()
            logger.info(f"🚂 TR{train_id} shutdown seq 1 -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "u5", train_id)
            return
        if cmd_code == 0x1FE:  # Shutdown seq 2 (extended shutdown)
            import time
            if not hasattr(self, '_lashup_shutdown_debounce'):
                self._lashup_shutdown_debounce = {}
            last_shutdown = self._lashup_shutdown_debounce.get(train_id, 0)
            if time.time() - last_shutdown < 2.0:  # 2 second debounce
                return
            self._lashup_shutdown_debounce[train_id] = time.time()
            logger.info(f"🚂 TR{train_id} extended shutdown -> MTH lashup {mth_id}")
            self.send_lashup_command(mth_id, "u7", train_id)
            return
        
        # Quilling horn (0x1E0-0x1EF) - Map to regular horn for lashups
        # 0x1E0 = off, 0x1E1-0x1EF = varying intensity
        # Cab 3 uses quilling horn for whistle, so we need to support it
        if 0x1E0 <= cmd_code <= 0x1EF:
            whistle_key = f"whistle_{train_id}"
            if not hasattr(self, '_lashup_whistle_timers'):
                self._lashup_whistle_timers = {}
            if not hasattr(self, '_lashup_whistle_states'):
                self._lashup_whistle_states = {}
            
            # Cancel any existing timer
            if whistle_key in self._lashup_whistle_timers:
                self._lashup_whistle_timers[whistle_key].cancel()
            
            if cmd_code == 0x1E0:
                # Quilling horn off - turn off whistle
                if self._lashup_whistle_states.get(whistle_key, False):
                    logger.info(f"🚂 TR{train_id} horn OFF -> MTH lashup {mth_id}")
                    self.send_lashup_command(mth_id, "bFFFD", train_id)
                    self._lashup_whistle_states[whistle_key] = False
            else:
                # Quilling horn on (any level) - send simple w2 (MTH doesn't support quilling in lashup)
                if not self._lashup_whistle_states.get(whistle_key, False):
                    logger.info(f"🚂 TR{train_id} horn ON -> MTH lashup {mth_id}")
                    self.send_lashup_command(mth_id, "w2", train_id)
                    self._lashup_whistle_states[whistle_key] = True
                
                # Start timer to turn off whistle after 300ms of no horn commands
                def whistle_off():
                    if self._lashup_whistle_states.get(whistle_key, False):
                        logger.info(f"🚂 TR{train_id} horn OFF (timeout) -> MTH lashup {mth_id}")
                        self.send_lashup_command(mth_id, "bFFFD", train_id)
                        self._lashup_whistle_states[whistle_key] = False
                
                timer = threading.Timer(0.3, whistle_off)
                timer.daemon = True
                timer.start()
                self._lashup_whistle_timers[whistle_key] = timer
            return
        
        logger.debug(f"🚂 TR{train_id} unhandled command 0x{cmd_code:03x}")
    
    def _ensure_lashup_created_on_wtiu(self, train_id: int, mth_id: int):
        """Start U command in background thread so it doesn't block other commands
        
        Like Mark's RTC, this is called on startup command (u4/u6).
        Runs asynchronously so commands can still flow while U is retrying.
        """
        # Check if already created or in progress
        if self.lashup_manager.lashup_created_on_wtiu.get(train_id, False):
            logger.info(f"🔗 Lashup for TR{train_id} already created on WTIU")
            return True
        
        # Check if already trying to create
        if not hasattr(self, '_lashup_u_cmd_in_progress'):
            self._lashup_u_cmd_in_progress = set()
        
        if train_id in self._lashup_u_cmd_in_progress:
            logger.debug(f"🔗 Lashup creation already in progress for TR{train_id}")
            return False
        
        # Get engine list for this lashup
        engine_list = self.lashup_manager.get_engine_list_for_tr(train_id)
        if not engine_list:
            logger.warning(f"⚠️ No engine list for TR{train_id}, cannot create lashup on WTIU")
            return False
        
        # Start U command in background thread
        self._lashup_u_cmd_in_progress.add(train_id)
        threading.Thread(
            target=self._create_lashup_async,
            args=(train_id, mth_id, engine_list),
            daemon=True
        ).start()
        
        return False  # Not created yet, but in progress
    
    def _create_lashup_async(self, train_id: int, mth_id: int, engine_list: str):
        """Background thread to create lashup on WTIU with retries"""
        logger.info(f"🔗 Creating MTH lashup on WTIU for TR{train_id} (background thread)")
        
        try:
            # Send U command with 20 retries, 2 second intervals
            success = self.create_mth_lashup(mth_id, engine_list, max_retries=20, retry_interval=2.0)
            
            if success:
                self.lashup_manager.lashup_created_on_wtiu[train_id] = True
                logger.info(f"✅ MTH lashup {mth_id} created on WTIU for TR{train_id}")
            else:
                # Mark as attempted but failed - won't retry until lashup is cleared
                self.lashup_manager.lashup_created_on_wtiu[train_id] = False
                logger.warning(f"⚠️ U command failed for TR{train_id}, horn/bell may go to all engines")
        finally:
            # Remove from in-progress set
            if hasattr(self, '_lashup_u_cmd_in_progress') and isinstance(self._lashup_u_cmd_in_progress, set):
                self._lashup_u_cmd_in_progress.discard(train_id)
    
    def send_lashup_command(self, mth_id: int, mth_cmd: str, train_id: int = None):
        """Send a command to an MTH lashup
        
        Args:
            mth_id: MTH lashup ID (102-120) - for tracking only
            mth_cmd: MTH command string (e.g., "s60", "d0", "w2")
            train_id: Lionel TR ID (to get engine list)
            
        Note: All lashups use DCS engine 102 (0x66) regardless of lashup ID.
        Commands to lashups need '|' prefix and engine list appended.
        Format: |<command>,<engine_list>  (e.g., |s60,0B86)
        """
        if not self.mth_connected or not self.mth_socket:
            logger.warning("⚠️ Cannot send lashup command: WTIU not connected")
            return False
        
        try:
            with self.mth_lock:
                # All lashups use DCS engine 102 (0x66) - from Mark's RTC code
                select_cmd = f"y{MTH_LASHUP_DCS_NO}\r\n"  # Always 102
                self.mth_socket.send(select_cmd.encode())
                time.sleep(0.05)
                
                # Get engine list for this lashup (already includes comma prefix)
                engine_list = ""
                if train_id:
                    engine_list = self.lashup_manager.get_engine_list_for_tr(train_id)
                
                # Build command with | prefix and engine list (from Mark's RTC)
                # Format: |<command><engine_list> where engine_list starts with comma and ends with 0xFF
                # Example: |u4,0B06\xff (comma is part of engine_list)
                if engine_list:
                    full_cmd = f"|{mth_cmd}{engine_list}\r\n"
                else:
                    full_cmd = f"|{mth_cmd}\r\n"
                
                # Use latin-1 encoding to preserve raw 0xFF byte (UTF-8 would encode it as 2 bytes)
                self.mth_socket.send(full_cmd.encode('latin-1'))
                logger.info(f"🚂 Sent to MTH lashup {mth_id}: {full_cmd.strip()}")
                
                # Get response
                self.mth_socket.settimeout(1.0)
                try:
                    response = self.mth_socket.recv(256).decode('latin-1')
                    if "okay" not in response.lower():
                        logger.warning(f"📥 Lashup response (no okay): {response.strip()}")
                except socket.timeout:
                    logger.debug(f"📥 Lashup command timeout (normal)")
                
                return True
                
        except Exception as e:
            logger.error(f"❌ Lashup command error: {e}")
            return False
    
    def _delayed_train_query(self, train_id: int, delay: float = 8.0):
        """Query Base 3 for train data after a delay with retries
        
        Base 3 takes 6-8 seconds to create the train entry after lashup assignment.
        May return cached data for wrong train, so retry up to 5 times.
        """
        time.sleep(delay)
        
        if train_id not in self.pending_train_queries:
            return
        
        self.pending_train_queries.discard(train_id)
        
        # Retry up to 5 times with 2 second intervals
        max_retries = 5
        retry_delay = 2.0
        
        for attempt in range(max_retries):
            logger.info(f"📡 Querying Base 3 for TR{train_id} lashup data (attempt {attempt + 1}/{max_retries})...")
            train_data = self.pdi_client.query_train_data(train_id)
            
            if train_data and train_data.get('consist_components'):
                components = train_data['consist_components']
                logger.info(f"🔗 TR{train_id} has {len(components)} engines: {components}")
                self.lashup_manager.update_lashup(train_id, components)
                # Mark this TR ID as queried (won't query again unless cleared)
                self.queried_trains.add(train_id)
                return
            elif train_data:
                # Query succeeded but no consist components - might still be building
                # Don't mark as queried yet - retry
                logger.info(f"📡 TR{train_id} has no consist data yet (attempt {attempt + 1}/{max_retries})")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    # Exhausted retries with empty consist - mark as queried
                    logger.warning(f"📡 TR{train_id} has no consist data after {max_retries} attempts")
                    self.queried_trains.add(train_id)
            else:
                # Query failed - retry
                if attempt < max_retries - 1:
                    logger.info(f"📡 TR{train_id} PDI query failed, retrying in {retry_delay}s...")
                    time.sleep(retry_delay)
                else:
                    logger.warning(f"📡 TR{train_id} PDI query failed after {max_retries} attempts")
                    self.queried_trains.add(train_id)
    
    def send_to_mth(self, command):
        """Send command to MTH WTIU via WiFi with proper sequence"""
        if not self.mth_connected or not self.mth_socket:
            logger.debug("MTH not connected - command not sent")
            return False
        
        try:
            with self.mth_lock:
                # Select the correct engine based on TMCC packet
                if self.current_lionel_engine > 0:
                    # Get MTH engine using new mapping system
                    wtiu_engine = self.get_mth_engine(self.current_lionel_engine)
                    
                    if wtiu_engine:
                        # Send engine selection command first
                        logger.info(f"🔧 Selecting WTIU Engine #{wtiu_engine} for Lionel Engine #{self.current_lionel_engine}")
                        select_cmd = f"y{wtiu_engine}\r\n"
                        self.mth_socket.send(select_cmd.encode())
                        time.sleep(0.1)  # Brief pause for engine selection
                        try:
                            select_response = self.mth_socket.recv(256).decode('latin-1')
                            logger.info(f"🔍 Engine selection response: {select_response.strip()}")
                        except:
                            pass  # Don't fail if no response to selection
                    else:
                        logger.warning(f"⚠️ No MTH engine mapping for Lionel Engine #{self.current_lionel_engine}")
                        return False
                
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
            },
            'bell': {
                'on': 'w4',
                'off': 'bFFFB',
                'toggle': 'w4',
                'ding': 'w4'
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
            elif cmd_type == 'function' and cmd_value == 'aux2_option1':
                # Cab 1L AUX2 button = Headlight Toggle - track state with debounce
                engine = self.current_lionel_engine
                current_time = time.time()
                if not hasattr(self, '_headlight_debounce_time'):
                    self._headlight_debounce_time = {}
                last_time = self._headlight_debounce_time.get(engine, 0)
                if current_time - last_time < 0.5:  # 500ms debounce
                    return None  # Ignore repeated command
                self._headlight_debounce_time[engine] = current_time
                
                if not hasattr(self, '_engine_headlight_state'):
                    self._engine_headlight_state = {}
                current_state = self._engine_headlight_state.get(engine, True)  # Default ON
                new_state = not current_state
                self._engine_headlight_state[engine] = new_state
                mth_cmd = "ab1" if new_state else "ab0"
                logger.info(f"💡 Engine {engine} headlight {'ON' if new_state else 'OFF'}: {mth_cmd}")
                return mth_cmd
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
                response = self.mth_socket.recv(256).decode('latin-1')
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
                response = self.mth_socket.recv(256).decode('latin-1')
                logger.info(f"🐛 DEBUG: Response: {response.strip()}")
            except socket.timeout:
                logger.info(f"🐛 DEBUG: Timeout for {cmd}")
            except Exception as e:
                logger.info(f"🐛 DEBUG: Error: {e}")
            time.sleep(0.5)
        
        logger.info("🔍 WTIU debug complete")
    
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
                            
                            # Check for PDI packets (consist broadcasts from Base 3)
                            # PDI packets start with 0xD1 (SOP) and end with 0xDF (EOP)
                            if PDI_SOP in data:
                                self._process_pdi_broadcast(data)
                            
                            # Check for 9-byte TRAIN_ADDRESS/TRAIN_UNIT multi-word commands
                            # Format: F8 <engine> 42/43 FB <engine> <data1> FB <engine> <data2>
                            # 0x42 = TRAIN_ADDRESS (assigns engine to train)
                            # 0x43 = TRAIN_UNIT (sets position: HEAD_FWD, TAIL_REV, etc.)
                            self._process_consist_commands(data)
                            
                            # Buffer for handling fragmented TMCC packets
                            if not hasattr(self, '_tmcc_buffer'):
                                self._tmcc_buffer = bytearray()
                            
                            # Add new data to buffer
                            self._tmcc_buffer.extend(data)
                            
                            # Process complete packets from buffer
                            while len(self._tmcc_buffer) >= 3:
                                # Look for TMCC packet start bytes
                                # NOTE: 0xFB is NOT a start byte - it's a continuation marker for multi-word commands
                                # Valid start bytes: 0xFE (TMCC1), 0xF8 (Legacy Engine), 0xF9 (Legacy Train)
                                start_idx = -1
                                for i in range(len(self._tmcc_buffer) - 2):
                                    if self._tmcc_buffer[i] in [0xFE, 0xF8, 0xF9]:
                                        start_idx = i
                                        break
                                
                                if start_idx == -1:
                                    # No valid start byte found, clear buffer up to last 2 bytes
                                    self._tmcc_buffer = self._tmcc_buffer[-2:]
                                    break
                                
                                # Remove any garbage before the start byte
                                if start_idx > 0:
                                    logger.debug(f"🔧 Discarding {start_idx} bytes before start byte")
                                    self._tmcc_buffer = self._tmcc_buffer[start_idx:]
                                
                                if len(self._tmcc_buffer) < 3:
                                    break  # Need more data
                                
                                # Check if this is a multi-word command (9 bytes total)
                                # Multi-word: F8/F9 + 2 bytes, then FB + 2 bytes, then FB + 2 bytes
                                # We detect by checking if byte[2] indicates a multi-word parameter index
                                first_byte = self._tmcc_buffer[0]
                                is_multiword = False
                                
                                if first_byte in [0xF8, 0xF9] and len(self._tmcc_buffer) >= 9:
                                    # Check if bytes 3 and 6 are 0xFB (continuation markers)
                                    if self._tmcc_buffer[3] == 0xFB and self._tmcc_buffer[6] == 0xFB:
                                        is_multiword = True
                                
                                if is_multiword:
                                    # Extract 9-byte multi-word packet
                                    packet = bytes(self._tmcc_buffer[:9])
                                    self._tmcc_buffer = self._tmcc_buffer[9:]
                                    packet_count += 1
                                    logger.info(f"🎯 Multi-word Packet #{packet_count}: {packet.hex()}")
                                    
                                    # Parse multi-word command (smoke, effects, etc.)
                                    command = self._parse_multiword_packet(packet)
                                    if command:
                                        protocol = command.get('protocol', 'legacy')
                                        logger.info(f"📤 {protocol.upper()}: {command.get('type')} = {command.get('value')}")
                                        if self.handle_lashup_command(command):
                                            logger.info("🔗 Lashup command handled")
                                            continue
                                        if protocol in ('legacy', 'legacy_train'):
                                            if self.send_to_mth_with_legacy(command):
                                                logger.info("✅ Legacy → MTH")
                                    continue
                                
                                # Extract 3-byte packet
                                packet = bytes(self._tmcc_buffer[:3])
                                self._tmcc_buffer = self._tmcc_buffer[3:]
                                
                                packet_count += 1
                                logger.info(f"🎯 TMCC Packet #{packet_count}: {packet.hex()}")
                                
                                # Parse and forward (handles both TMCC1 and Legacy)
                                command = self.parse_packet(packet)
                                if command:
                                    protocol = command.get('protocol', 'tmcc1')
                                    logger.info(f"📤 {protocol.upper()}: {command.get('type')} = {command.get('value')}")
                                    
                                    # Check for lashup commands first
                                    if self.handle_lashup_command(command):
                                        logger.info("🔗 Lashup command handled")
                                        continue
                                    
                                    # Use Legacy-aware sending for Legacy commands
                                    if protocol in ('legacy', 'legacy_train'):
                                        if self.send_to_mth_with_legacy(command):
                                            logger.info("✅ Legacy → MTH")
                                    else:
                                        # TMCC1 - use original path
                                        mth_cmd = self.convert_to_mth_protocol(command)
                                        if mth_cmd:
                                            logger.info(f"📤 MTH: {mth_cmd}")
                                        self.send_to_mcu(command)
                                        self.send_to_mth(command)
                        else:
                            # Log every 10 seconds if no data received
                            if time.time() - last_activity > 10:
                                logger.warning("⚠️ No data received from Lionel Base 3 for 10 seconds")
                                last_activity = time.time()
            
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Lionel listener error: {e}")
                time.sleep(1)
    
    def _parse_multiword_packet(self, packet: bytes):
        """Parse 9-byte multi-word Legacy command
        
        Multi-word format (9 bytes = 3 words):
        Word 1: F8/F9 + Address + Parameter Index (0x0C=Effects, 0x0D=Lighting, etc.)
        Word 2: FB + Address + Parameter Data
        Word 3: FB + Address + Checksum
        
        For smoke (Effects index 0x0C):
        - Data 0x00 = Smoke Off
        - Data 0x01 = Smoke Low  
        - Data 0x02 = Smoke Medium
        - Data 0x03 = Smoke High
        """
        if len(packet) != 9:
            return None
        
        first_byte = packet[0]
        
        # Extract address from word 1 (bits 14-8 of word)
        word1 = (packet[1] << 8) | packet[2]
        if first_byte == 0xF8:  # Engine command
            address = (word1 >> 9) & 0x7F
            param_index = word1 & 0xFF
        elif first_byte == 0xF9:  # Train command
            address = (word1 >> 9) & 0x7F
            param_index = word1 & 0x1FF
        else:
            return None
        
        # Extract parameter data from word 2
        word2 = (packet[4] << 8) | packet[5]
        param_data = word2 & 0xFF
        
        logger.info(f"🔧 Multi-word: first=0x{first_byte:02x}, addr={address}, param_idx=0x{param_index:02x}, data=0x{param_data:02x}")
        
        # Effects commands (param_index 0x0C)
        if param_index == 0x0C:
            # Smoke levels
            if param_data <= 0x03:
                smoke_levels = {0x00: 'off', 0x01: 'low', 0x02: 'med', 0x03: 'high'}
                smoke_value = smoke_levels.get(param_data, 'off')
                logger.info(f"💨 Multi-word smoke: {smoke_value} for engine {address}")
                return {'type': 'smoke_direct', 'value': smoke_value, 'engine': address, 'protocol': 'legacy'}
        
        # Lighting commands (param_index 0x0D)
        if param_index == 0x0D:
            logger.info(f"💡 Multi-word lighting: data=0x{param_data:02x} for engine {address}")
            # Could add lighting handling here
            return None
        
        return None
    
    def _process_consist_commands(self, data: bytes):
        """Process 9-byte TRAIN_ADDRESS/TRAIN_UNIT multi-word commands for consist detection
        
        From Dave Swindell's analysis:
        - ENGINE xx TRAIN_ADDRESS yy = F8 <engine> 42 FB <addr> <train_lo> FB <addr> <checksum>
        - ENGINE xx TRAIN_UNIT pos = F8 <engine> 43 FB <addr> <position> FB <addr> <checksum>
        
        Position values for TRAIN_UNIT (byte 5):
        - 0x01 = HEAD_FORWARD
        - 0x07 = TAIL_REVERSE
        """
        try:
            # Buffer for handling fragmented packets
            if not hasattr(self, '_consist_cmd_buffer'):
                self._consist_cmd_buffer = bytearray()
            
            # Track pending consist assignments
            if not hasattr(self, '_pending_consist_engines'):
                self._pending_consist_engines = {}  # {train_id: {engine_id: {position, direction}}}
            
            # Add new data to buffer
            self._consist_cmd_buffer.extend(data)
            
            # Keep buffer from growing too large
            if len(self._consist_cmd_buffer) > 1000:
                self._consist_cmd_buffer = self._consist_cmd_buffer[-500:]
            
            buf = self._consist_cmd_buffer
            i = 0
            processed_up_to = 0
            
            while i < len(buf) - 8:
                # Look for 9-byte TRAIN_ADDRESS pattern: F8 <engine> 42 FB <addr> <train> FB <addr> <checksum>
                # Note: Engine address in Legacy multi-word is shifted left by 1 bit
                if (buf[i] == 0xF8 and 
                    buf[i+2] == 0x42 and 
                    buf[i+3] == 0xFB and
                    buf[i+6] == 0xFB):
                    
                    engine_id = buf[i+1] >> 1  # Decode: address is shifted left by 1
                    train_id = buf[i+5]  # Train ID is in byte 5
                    
                    logger.info(f"📡 TRAIN_ADDRESS: Engine {engine_id} -> Train {train_id}")
                    
                    # Initialize train entry if needed
                    if train_id not in self._pending_consist_engines:
                        self._pending_consist_engines[train_id] = {}
                    
                    # Store engine assignment (position will come from TRAIN_UNIT)
                    if engine_id not in self._pending_consist_engines[train_id]:
                        self._pending_consist_engines[train_id][engine_id] = {'position': None, 'direction': None}
                    
                    processed_up_to = i + 9
                    i += 9
                    continue
                
                # Look for 9-byte TRAIN_UNIT pattern: F8 <engine> 43 FB <addr> <position> FB <addr> <checksum>
                # Note: Engine address in Legacy multi-word is shifted left by 1 bit
                if (buf[i] == 0xF8 and 
                    buf[i+2] == 0x43 and 
                    buf[i+3] == 0xFB and
                    buf[i+6] == 0xFB):
                    
                    engine_id = buf[i+1] >> 1  # Decode: address is shifted left by 1
                    position_byte = buf[i+5]  # Position is in byte 5
                    
                    # Decode position: 0x01=HEAD_FWD, 0x07=TAIL_REV, etc.
                    # Bits: 0-1 = position (0=single, 1=head, 2=middle, 3=tail)
                    # Bit 2 = direction (0=fwd, 1=rev)
                    position = position_byte & 0x03
                    direction = (position_byte >> 2) & 0x01
                    
                    pos_names = {0: 'SINGLE', 1: 'HEAD', 2: 'MIDDLE', 3: 'TAIL'}
                    dir_names = {0: 'FWD', 1: 'REV'}
                    
                    logger.info(f"📡 TRAIN_UNIT: Engine {engine_id} = {pos_names.get(position, '?')}_{dir_names.get(direction, '?')}")
                    
                    # Find which train this engine belongs to and update position
                    for train_id, engines in self._pending_consist_engines.items():
                        if engine_id in engines:
                            engines[engine_id] = {'position': position, 'direction': direction}
                            
                            # Check if we have complete info for all engines in this train
                            all_complete = all(e['position'] is not None for e in engines.values())
                            if all_complete and len(engines) >= 2:
                                # Schedule delayed lashup creation to allow more engines to arrive
                                # Base 3 sends engines in sequence, so we wait 2 seconds for all to arrive
                                logger.info(f"📡 Consist detected for TR{train_id}: {len(engines)} engines so far, waiting for more...")
                                self._schedule_lashup_creation(train_id)
                            break
                    
                    processed_up_to = i + 9
                    i += 9
                    continue
                
                i += 1
            
            # Remove processed data from buffer
            if processed_up_to > 0:
                self._consist_cmd_buffer = self._consist_cmd_buffer[processed_up_to:]
                
        except Exception as e:
            logger.debug(f"Consist command parse error: {e}")
    
    def _schedule_lashup_creation(self, train_id: int):
        """Schedule delayed lashup creation to allow all engines to arrive
        
        Base 3 sends TRAIN_ADDRESS/TRAIN_UNIT commands in sequence for each engine.
        We wait 2 seconds after detecting a complete consist to allow more engines to arrive.
        If more engines arrive, the timer is reset.
        """
        import threading
        
        if not hasattr(self, '_lashup_creation_timers'):
            self._lashup_creation_timers = {}
        
        # Cancel any existing timer for this train
        if train_id in self._lashup_creation_timers:
            self._lashup_creation_timers[train_id].cancel()
        
        def create_lashup():
            if train_id in self._pending_consist_engines:
                engines = self._pending_consist_engines[train_id]
                logger.info(f"📡 Creating lashup for TR{train_id} with {len(engines)} engines: {list(engines.keys())}")
                self._create_lashup_from_consist(train_id, engines)
        
        # Schedule creation after 2 seconds
        timer = threading.Timer(2.0, create_lashup)
        timer.daemon = True
        timer.start()
        self._lashup_creation_timers[train_id] = timer
    
    def _create_lashup_from_consist(self, train_id: int, engines: dict):
        """Create MTH lashup from detected consist engines"""
        try:
            # Build consist components list
            from dataclasses import dataclass
            components = []
            for engine_id, info in engines.items():
                # Create a simple object with the needed attributes
                class Component:
                    def __init__(self, tmcc_id, position, direction):
                        self.tmcc_id = tmcc_id
                        self.position = position
                        self.is_reversed = direction == 1
                
                components.append(Component(engine_id, info['position'], info['direction']))
            
            # Sort by position (head first, then middle, then tail)
            components.sort(key=lambda c: c.position if c.position else 0)
            
            logger.info(f"📡 Creating MTH lashup for TR{train_id} from TMCC commands")
            
            # Use lashup manager to create the lashup
            self.lashup_manager.update_lashup(train_id, components)
            
            # Clear pending engines for this train
            if train_id in self._pending_consist_engines:
                del self._pending_consist_engines[train_id]
                
        except Exception as e:
            logger.error(f"Error creating lashup from consist: {e}")
    
    def _process_pdi_broadcast(self, data: bytes):
        """Process PDI broadcast packets from Base 3 (consist info broadcasts)"""
        try:
            # Find PDI packets in the data stream
            i = 0
            while i < len(data):
                if data[i] == PDI_SOP:
                    # Found start of PDI packet - find the end
                    for j in range(i + 1, len(data)):
                        if data[j] == PDI_EOP:
                            # Check if preceded by stuff byte
                            if j > 0 and data[j-1] == PDI_STF:
                                continue  # This EOP is stuffed, keep looking
                            
                            # Extract packet (excluding SOP and EOP)
                            packet = data[i+1:j]
                            logger.info(f"📡 PDI Broadcast: {packet.hex()}")
                            
                            # Check if this is a BASE_TRAIN packet (0x21)
                            if len(packet) >= 3 and packet[0] == PdiCommand.BASE_TRAIN:
                                train_id = packet[1]
                                action = packet[2]
                                logger.info(f"📡 PDI: Train {train_id} action {action:#x}")
                                
                                # Action 0x02 = read response, 0x01 = write response
                                if action in (0x01, 0x02) and len(packet) > 10:
                                    # This is consist data - parse it
                                    result = self.pdi_handler._parse_train_response(packet)
                                    if result and result.get('consist_components'):
                                        components = result['consist_components']
                                        logger.info(f"📡 PDI Broadcast: TR{train_id} has {len(components)} engines")
                                        
                                        # Update lashup manager with this consist info
                                        self.lashup_manager.update_lashup(train_id, components)
                            
                            i = j + 1
                            break
                    else:
                        # No EOP found, move on
                        i += 1
                else:
                    i += 1
        except Exception as e:
            logger.debug(f"PDI broadcast parse error: {e}")
    
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
        self.start_time = time.time()
        
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
        
        # Start periodic MTH engine discovery (every 60 seconds)
        self.start_periodic_engine_discovery()
        
        return True
    
    def start_periodic_engine_discovery(self):
        """Start periodic MTH engine re-discovery thread"""
        def discovery_loop():
            while self.running:
                try:
                    time.sleep(60)  # Wait 60 seconds between discoveries
                    # Skip if lashup creation is in progress to avoid I0 spam
                    if getattr(self, '_lashup_creation_in_progress', False):
                        logger.debug("🔄 Skipping periodic discovery - lashup creation in progress")
                        continue
                    if self.mth_connected:
                        logger.info("🔄 Periodic MTH engine re-discovery...")
                        self.discover_mth_engines()
                except Exception as e:
                    logger.error(f"❌ Periodic engine discovery error: {e}")
        
        discovery_thread = threading.Thread(target=discovery_loop, daemon=True)
        discovery_thread.start()
        logger.info("🔄 Periodic MTH engine discovery started (every 60s)")
    
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

    def calibrate_legacy_speed(self, engine=1):
        """Calibrate Legacy speed mapping for smoother control"""
        logger.info("🎯 Starting Legacy speed calibration...")
        
        # Test speed points
        test_points = [
            0, 10, 20, 30, 40, 50, 75, 100, 125, 150, 175, 199
        ]
        
        results = []
        
        for legacy_speed in test_points:
            dcs_speed = self.legacy_speed_manager.convert_legacy_to_dcs(legacy_speed)
            
            # Send to MTH
            mth_cmd = f"s{dcs_speed}"
            success = self.send_wtiu_command(mth_cmd)
            
            if success:
                results.append({
                    'legacy': legacy_speed,
                    'dcs': dcs_speed,
                    'success': True
                })
                logger.info(f"  ✅ Legacy {legacy_speed:3d}/199 → DCS {dcs_speed:3d}/120")
                time.sleep(0.5)
            else:
                logger.warning(f"  ❌ Failed at Legacy {legacy_speed}")
                
        # Return to stop
        self.send_wtiu_command("s0")
        
        logger.info("🎯 Calibration complete!")
        return results
    
    def optimize_speed_curve(self):
        """Create optimized speed curve for better control"""
        # Custom speed curve for different types of locomotives
        speed_curves = {
            'steam': {
                'name': 'Steam Locomotive',
                'curve': [
                    (0, 0),     # Stop
                    (10, 5),    # Very slow creep
                    (30, 15),   # Switching speed
                    (75, 40),   # Medium speed
                    (125, 70),  # Line speed
                    (175, 100), # Fast
                    (199, 120)  # Maximum
                ]
            },
            'diesel': {
                'name': 'Diesel Locomotive',
                'curve': [
                    (0, 0),
                    (15, 10),
                    (40, 25),
                    (90, 55),
                    (140, 85),
                    (180, 110),
                    (199, 120)
                ]
            },
            'passenger': {
                'name': 'Passenger Train',
                'curve': [
                    (0, 0),
                    (20, 8),
                    (50, 25),
                    (100, 55),
                    (150, 90),
                    (185, 115),
                    (199, 120)
                ]
            }
        }
        
        return speed_curves

def test_legacy_support():
    """Test Legacy protocol support"""
    bridge = LionelMTHBridge()
    
    if bridge.connect_mth():
        logger.info("✅ Testing Legacy protocol support...")
        
        # Test Legacy speed commands
        test_commands = [
            # Simulate Legacy 200-step speed commands
            {'protocol': 'legacy', 'type': 'speed_legacy', 'speed': 50, 'scale': '200_step'},
            {'protocol': 'legacy', 'type': 'speed_legacy', 'speed': 100, 'scale': '200_step'},
            {'protocol': 'legacy', 'type': 'speed_legacy', 'speed': 150, 'scale': '200_step'},
            {'protocol': 'legacy', 'type': 'speed_legacy', 'speed': 199, 'scale': '200_step'},
            
            # Legacy action commands
            {'protocol': 'legacy', 'type': 'horn', 'value': 'primary'},
            {'protocol': 'legacy', 'type': 'bell', 'value': 'toggle'},
            {'protocol': 'legacy', 'type': 'direction', 'value': 'forward'},
        ]
        
        for cmd in test_commands:
            logger.info(f"🧪 Testing: {cmd}")
            success = bridge.send_to_mth_with_legacy(cmd)
            logger.info(f"  Result: {'✅' if success else '❌'}")
            time.sleep(0.5)
            
        # Calibrate
        bridge.calibrate_legacy_speed()
        
        bridge.stop()

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
                response = bridge.mth_socket.recv(256).decode('latin-1')
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

def check_bell_quick_press(self):
        """Check if bell button was released quickly (single ding) vs held (toggle)"""
        current_time = time.time()
        commands = []
        
        for engine, press_start in list(self.bell_button_press_time.items()):
            hold_triggered = self.bell_hold_triggered.get(engine, False)
            time_since_press = current_time - press_start
            
            # If button was pressed but no packets for 0.3s and hold wasn't triggered
            # This means it was a quick press - send single bell hit
            if 0.1 < time_since_press < 0.5 and not hold_triggered:
                # Check if we're still receiving packets (button still held)
                # If no recent packet, it was a quick release
                last_packet_time = self.bell_button_press_time.get(engine, 0)
                if current_time - last_packet_time > 0.15:
                    # Quick press detected - single bell hit
                    logger.info(f"🔔 Bell DING (quick press) for engine {engine}")
                    self.bell_button_press_time[engine] = 0  # Reset
                    commands.append({'type': 'bell', 'value': 'ding', 'engine': engine})
        
        return commands

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

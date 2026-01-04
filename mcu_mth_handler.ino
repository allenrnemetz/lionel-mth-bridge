/*
 * mcu_mth_handler.ino
 * 
 * MTH WTIU Handler for Arduino UNO Q MCU (Sub-processor)
 * Communicates with MPU via serial port and controls MTH trains via WiFi
 * 
 * Author: Allen Nemetz
 * Credits:
 * - Mark Divechhio for his immense work translating MTH commands to and from the MTH WTIU
 *   http://www.silogic.com/trains/RTC_Running.html
 * - Lionel LLC for publishing TMCC and Legacy protocol specifications
 * - O Gauge Railroading Forum (https://www.ogrforum.com/) for the model railroad community
 * 
 * Disclaimer: This software is provided "as-is" without warranty. The author assumes no liability 
 * for any damages resulting from the use or misuse of this software. Users are responsible for 
 * ensuring safe operation of their model railroad equipment.
 * 
 * Copyright (c) 2026 Allen Nemetz. All rights reserved.
 * 
 * License: GNU General Public License v3.0
 */

#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include "speck_functions.h"

// Command constants (must match MPU)
#define CMD_SPEED           1
#define CMD_DIRECTION       2
#define CMD_BELL            3
#define CMD_WHISTLE         4
#define CMD_STARTUP         5
#define CMD_SHUTDOWN        6
#define CMD_ENGINE_SELECT   7
#define CMD_PROTOWHISTLE    8

// Command packet structure (must match MPU)
struct CommandPacket {
  uint8_t command_type;
  uint8_t engine_number;
  uint16_t value;
  bool bool_value;
};

// WiFi configuration
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Global variables
WiFiClient wtiu_client;
char wtiu_host[16] = "0.0.0.0";       // Will be populated by mDNS
int wtiu_port = 8882;                   // Default port, will be updated from mDNS

// Status LED
#define STATUS_LED LED_BUILTIN

// Communication state
bool wtiu_connected = false;
unsigned long last_connection_attempt = 0;
const unsigned long CONNECTION_RETRY_INTERVAL = 5000; // 5 seconds

// ProtoWhistle state
bool protowhistle_enabled = false;
int protowhistle_pitch = 0; // 0-3 pitch levels

// Speck encryption state
SPECK_TYPE key[4] = {0x0100, 0x0302, 0x0504, 0x0706}; // Default key from RTCRemote
SPECK_TYPE round_keys[SPECK_ROUNDS];
bool encryption_enabled = true;

void setup() {
  // Initialize serial communication with MPU
  Serial.begin(115200);
  Serial.println("=== MTH WTIU Handler Starting ===");
  
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  // Test LED blink to show MCU is running
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(200);
    digitalWrite(STATUS_LED, LOW);
    delay(200);
  }
  
  Serial.println("MCU initialized - LED test complete");
  
  // Initialize WiFi
  initializeWiFi();
  
  // Initialize Speck encryption
  speck_expand(key, round_keys);
  Serial.println("Speck encryption initialized");
  
  // Discover and connect to MTH WTIU using mDNS
  if (!discoverWTIU()) {
    Serial.println("Failed to find WTIU on startup, will retry in background");
  }
  
  Serial.println("=== MTH WTIU Handler Ready ===");
  Serial.println("ProtoWhistle support enabled");
}

void loop() {
  // Check for commands from MPU
  if (Serial.available()) {
    receiveCommandFromMPU();
  }
  
  // Maintain WTIU connection
  maintainWTIUConnection();
  
  delay(10);
}

void initializeWiFi() {
  Serial.println("Initializing WiFi...");
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Initialize mDNS
    if (MDNS.begin("lionel-mth-bridge")) {
      Serial.println("mDNS responder started");
    } else {
      Serial.println("Error setting up mDNS responder");
    }
  } else {
    Serial.println("\nWiFi connection failed");
  }
}

bool discoverWTIU() {
  Serial.println("Searching for MTH WTIU via mDNS...");
  
  // Send mDNS query for WTIU service
  int n = MDNS.queryService(wtiu_service, wtiu_protocol);
  
  if (n == 0) {
    Serial.println("No WTIU services found");
    return false;
  }
  
  Serial.print(n);
  Serial.println(" WTIU service(s) found");
  
  // Use first available WTIU
  for (int i = 0; i < n; ++i) {
    Serial.print("  ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(MDNS.hostname(i));
    Serial.print(" (");
    Serial.print(MDNS.IP(i));
    Serial.print(":");
    Serial.print(MDNS.port(i));
    Serial.println(")");
    
    // Try to connect to this WTIU
    strcpy(wtiu_host, MDNS.IP(i).toString().c_str());
    wtiu_port = MDNS.port(i);
    
    if (connectToWTIU()) {
      Serial.print("Connected to WTIU: ");
      Serial.print(wtiu_host);
      Serial.print(":");
      Serial.println(wtiu_port);
      return true;
    }
  }
  
  return false;
}

bool connectToWTIU() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - cannot connect to WTIU");
    return false;
  }
  
  Serial.print("Connecting to MTH WTIU at ");
  Serial.print(wtiu_host);
  Serial.print(":");
  Serial.println(wtiu_port);
  
  if (wtiu_client.connect(wtiu_host, wtiu_port)) {
    wtiu_connected = true;
    Serial.println("Connected to MTH WTIU");
    digitalWrite(STATUS_LED, HIGH); // Turn on LED when connected
    
    // Send initial handshake if needed
    delay(100);
    return true;
  } else {
    wtiu_connected = false;
    Serial.println("Failed to connect to MTH WTIU");
    digitalWrite(STATUS_LED, LOW);
    return false;
  }
}

void maintainWTIUConnection() {
  // Try to reconnect if connection is lost
  if (!wtiu_connected || !wtiu_client.connected()) {
    wtiu_connected = false;
    digitalWrite(STATUS_LED, LOW);
    
    unsigned long current_time = millis();
    if (current_time - last_connection_attempt >= CONNECTION_RETRY_INTERVAL) {
      Serial.println("Attempting to rediscover and reconnect to MTH WTIU...");
      last_connection_attempt = current_time;
      
      // Use mDNS to rediscover WTIU
      if (!discoverWTIU()) {
        Serial.println("Failed to rediscover WTIU, will retry later");
      }
    }
  }
  
  // Update mDNS regularly
  MDNS.update();
}

void receiveCommandFromMPU() {
  static uint8_t buffer[sizeof(CommandPacket)];
  static size_t bytes_received = 0;
  
  while (Serial.available() && bytes_received < sizeof(CommandPacket)) {
    buffer[bytes_received++] = Serial.read();
  }
  
  if (bytes_received == sizeof(CommandPacket)) {
    CommandPacket* cmd = (CommandPacket*)buffer;
    executeMTHCommand(cmd);
    bytes_received = 0;
  }
}

void executeMTHCommand(CommandPacket* cmd) {
  char mth_cmd[32];
  bool command_sent = false;
  
  // Send to Serial for debugging
  Serial.print("Executing command: type=");
  Serial.print(cmd->command_type);
  Serial.print(", engine=");
  Serial.print(cmd->engine_number);
  Serial.print(", value=");
  Serial.print(cmd->value);
  Serial.print(", bool=");
  Serial.println(cmd->bool_value);
  
  switch (cmd->command_type) {
    case CMD_ENGINE_SELECT:
      // MTH uses engine numbers 1-99 (direct mapping, no offset)
      // Map Lionel engine 1-99 to MTH engine 1-99
      snprintf(mth_cmd, sizeof(mth_cmd), "y%d", cmd->engine_number);
      sendMTHCommand(mth_cmd);
      command_sent = true;
      break;
      
    case CMD_SPEED:
      snprintf(mth_cmd, sizeof(mth_cmd), "s%d", cmd->value);
      sendMTHCommand(mth_cmd);
      command_sent = true;
      break;
      
    case CMD_DIRECTION:
      if (cmd->bool_value) {
        sendMTHCommand("d1"); // Reverse
      } else {
        sendMTHCommand("d0"); // Forward
      }
      command_sent = true;
      break;
      
    case CMD_BELL:
      if (cmd->bool_value) {
        sendMTHCommand("w4"); // Bell on
      } else {
        sendMTHCommand("bFFFB"); // Bell off
      }
      command_sent = true;
      break;
      
    case CMD_WHISTLE:
      // Regular whistle - only works if protowhistle is disabled
      if (!protowhistle_enabled) {
        if (cmd->bool_value) {
          sendMTHCommand("w2"); // Whistle on
        } else {
          sendMTHCommand("bFFFD"); // Whistle off
        }
        command_sent = true;
      } else {
        Serial.println("Regular whistle ignored - ProtoWhistle is enabled");
      }
      break;
      
    case CMD_PROTOWHISTLE:
      // ProtoWhistle control
      if (cmd->value == 0) {
        // ProtoWhistle on/off
        if (cmd->bool_value) {
          sendMTHCommand("ab20"); // Enable protowhistle
          protowhistle_enabled = true;
          Serial.println("ProtoWhistle ENABLED");
        } else {
          sendMTHCommand("ab21"); // Disable protowhistle
          protowhistle_enabled = false;
          Serial.println("ProtoWhistle DISABLED");
        }
        command_sent = true;
      } else if (cmd->value == 1) {
        // ProtoWhistle quill (actual whistle sound)
        if (protowhistle_enabled) {
          if (cmd->bool_value) {
            sendMTHCommand("w2"); // Quill the whistle
            Serial.println("ProtoWhistle QUILL ON");
          } else {
            sendMTHCommand("bFFFD"); // Stop quilling
            Serial.println("ProtoWhistle QUILL OFF");
          }
          command_sent = true;
        }
      } else if (cmd->value == 2) {
        // Toggle protowhistle state
        protowhistle_enabled = cmd->bool_value;
        if (protowhistle_enabled) {
          sendMTHCommand("ab20"); // Enable protowhistle
          Serial.println("ProtoWhistle TOGGLED ON");
        } else {
          sendMTHCommand("ab21"); // Disable protowhistle
          Serial.println("ProtoWhistle TOGGLED OFF");
        }
        command_sent = true;
      } else if (cmd->value >= 10 && cmd->value <= 13) {
        // ProtoWhistle pitch control (0-3 mapped to 10-13)
        protowhistle_pitch = cmd->value - 10;
        char pitch_cmd[10];
        snprintf(pitch_cmd, sizeof(pitch_cmd), "ab%d", protowhistle_pitch + 26);
        sendMTHCommand(pitch_cmd);
        Serial.print("ProtoWhistle pitch set to ");
        Serial.println(protowhistle_pitch);
        command_sent = true;
      }
      break;
      
    case CMD_WLED:
      // WLED control commands
      if (cmd->bool_value) {
        snprintf(mth_cmd, sizeof(mth_cmd), "w%d", cmd->engine_number);
        sendMTHCommand(mth_cmd);
        command_sent = true;
      } else {
        // WLED off - no MTH command needed, just log
        Serial.print("WLED Engine ");
        Serial.print(cmd->engine_number);
        Serial.println(" OFF");
        command_sent = true;
      }
      break;
      
    case CMD_STARTUP:
      sendMTHCommand("u4"); // Startup
      command_sent = true;
      break;
      
    case CMD_SHUTDOWN:
      sendMTHCommand("u5"); // Shutdown
      command_sent = true;
      break;
      
    default:
      Serial.print("Unknown command type: ");
      Serial.println(cmd->command_type);
      break;
  }
  
  // Send response back to MPU
  CommandPacket response;
  response.command_type = cmd->command_type;
  response.engine_number = cmd->engine_number;
  response.value = command_sent ? 1 : 0; // Success/failure
  response.bool_value = cmd->bool_value;
  
  Serial.write((uint8_t*)&response, sizeof(CommandPacket));
  Serial.flush();
  
  if (command_sent) {
    Serial.println("Command sent successfully");
  } else {
    Serial.println("Command failed");
  }
}

void sendMTHCommand(const char* cmd) {
  if (!wtiu_connected || !wtiu_client.connected()) {
    Serial.print("Cannot send command - WTIU not connected: ");
    Serial.println(cmd);
    return;
  }
  
  Serial.print("Sending to WTIU: ");
  Serial.print(cmd);
  
  if (encryption_enabled) {
    // Encrypt the command using Speck
    size_t cmd_len = strlen(cmd);
    
    // Pad to multiple of 4 bytes (2 words)
    size_t padded_len = ((cmd_len + 3) / 4) * 4;
    uint8_t* padded_cmd = (uint8_t*)malloc(padded_len);
    memset(padded_cmd, 0, padded_len);
    memcpy(padded_cmd, cmd, cmd_len);
    
    // Encrypt in 4-byte chunks
    for (size_t i = 0; i < padded_len; i += 4) {
      SPECK_TYPE pt[2], ct[2];
      pt[0] = (padded_cmd[i] << 8) | padded_cmd[i+1];
      pt[1] = (padded_cmd[i+2] << 8) | padded_cmd[i+3];
      
      speck_encrypt(pt, ct, round_keys);
      
      // Send encrypted bytes
      wtiu_client.write((uint8_t)(ct[0] >> 8));
      wtiu_client.write((uint8_t)(ct[0] & 0xFF));
      wtiu_client.write((uint8_t)(ct[1] >> 8));
      wtiu_client.write((uint8_t)(ct[1] & 0xFF));
    }
    
    free(padded_cmd);
    Serial.println(" (encrypted)");
  } else {
    // Send plain text
    wtiu_client.printf("%s\r\n", cmd);
    Serial.println(" (plain text)");
  }
  
  wtiu_client.flush();
  
  // Brief delay to allow command processing
  delay(50);
}

void printStatus() {
  Serial.println("=== MTH WTIU Handler Status ===");
  Serial.print("WiFi Status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
  
  Serial.print("WTIU Connection: ");
  Serial.println(wtiu_connected ? "Connected" : "Disconnected");
  
  if (wtiu_connected) {
    Serial.print("WTIU Address: ");
    Serial.print(wtiu_host);
    Serial.print(":");
    Serial.println(wtiu_port);
  }
  
  Serial.print("ProtoWhistle: ");
  Serial.println(protowhistle_enabled ? "Enabled" : "Disabled");
  
  Serial.println("==============================");
}

&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 
![Logo](Docs/our_logo_white.png)

![Logo](Docs/racelink.png)
<br>
<br>
# Lightweight LoRa telemetry protocol


## Table of Contents
[Features](#features)

[Why LoRa?](#whylora)

[Hardware](#hardware)

[Software](#software)

[Testing](#testing)

## Features

- ### Low latency Low power long range telemetry link
- ### Live telemetry / Replay function
- ### Logging into CSV
- ### Brownout / Full MCU reset failsafe
- ### Web Adjustable: 
    - frequency (868MHz/2.4GHz band)
    - spreading factor (5-12)
    - bandwidth:
        - 62.5, 125, 250, 500 @ subGHz
        - 203.125, 406.25, 812.5 @ 2.4GHz  
    - transmit power:
        - up to 22 dBm @ subGHz
        - up to 13 dBm @ 2.4GHz
- ### Optional dynamic radio configuration based on RSSI & SNR





## Why Lora?



## 🔧Hardware

The hardware used is the radiomaster XR1 ELRS receiver module, the code can be compiled to any custom design or other ELRS hardware that's using an ESP32, LR1121 LoRa IC and follows the [generic C3 LR1121](https://github.com/ExpressLRS/targets/blob/master/RX/Generic%20C3%20LR1121.json) pinout and RF switch config.

#### Components:
- ESP32C3
- LR1121
- 4MB SPI flash
- SKY13373 RF switch
- Generic 2.4 Frontend IC
- 3V3 LDO
- WS2812 ARGB LED
- boot mode select / general purpose button on IO9
- Trace Antenna for ESP WiFi/BLE
- U-FL connector
#### Module Schematic:
![Schematic](Radiomaster_XR1_Shematic/Radiomaster_XR1_Schematic.png)


## 💻Firmware
### Transmitter.ino
- Handles the remote unit (user interface + iButton authentication).
- Implements menus for:
    - Battery type selection
    - Continuity check
    - Countdown time
    - Preheat start time
    - Preheat duration
    - Ignition delta
- Uses ESP-NOW to send launch and continuity test commands
- Provides OLED display feedback via LaunchDisplayLibrary
- Requires valid iButton for arming and launch
### Receiver.ino
- Handles the receiver unit (relay + buzzer)
- Waits for commands over ESP-NOW
- Performs continuity test by measuring V<sub>drop<sub>
- Executes countdown, preheat, and ignition relay control
- Includes buzzer startup sounds and launch feedback
### LaunchDisplayLibrary
- Custom display driver using U8g2 for SH1106 OLED.
- Provides functions for:
    - Splash/welcome screen
    - Authentication animations
    - Armed state screen (with T-, P_s, P_d, I_d values)
    - Countdown + preheat visual feedback
    - Ignition screen
    - Menus for parameter selection
### GetMacAddress.ino
- Utility to print the ESP32’s MAC address (used for setting ESP-NOW peers).
### ReadFob.ino
- Utility to read and print iButton IDs (used to whitelist authorized keys).
### Dependencies
#### [Arduino Core for ESP32](https://github.com/espressif/arduino-esp32)
### Libraries:
- esp_now.h (built-in)
- WiFi.h (built-in)
- esp_wifi.h (for long-range protocol mode)
- OneWire (for iButton reader)
- U8g2lib (for OLED display)
## 🚀 Setup & Upload
1. Clone repo and open in Arduino IDE.
2. Select ESP32 Dev Module (or your ESP32 board).
3. Upload separately:
    - Transmitter.ino → Remote unit
    - 2.Receiver.ino → Receiver unit
4. Use GetMacAddress.ino on each ESP32 to note their MAC addresses.
    - Update broadcastAddress[] in transmitter/receiver code accordingly.
5. Use Read_fob.ino to read your iButton ID.
    - Replace in allowedID[] array in the transmitter code.
## 📺 Display demo
![Display](CODE/LaunchDisplayLibrary/test/LaunchDisplayLibrary-demo.gif)

## 🔒 Safety

- Relay will not activate without valid iButton authentication.
- Launch packets use a session_id to prevent accidental re-triggering.
- Continuity test ensures igniter is properly connected before launch, if fails the system enters failsafe, requires full restart
## ✏️Schematic

![Schematic](Pictures/Schematic.svg)


## 📷Images

### Transmitter
![Transmitter](Pictures/Transmitter.jpg)
![Transmitter-open](Pictures/Transmitter-open.jpg)

### Igniter
![Igniter](Pictures/Igniter.jpg)
![Igniter-open](Pictures/Igniter-open.jpg)

### Accessories
![Accessories](Pictures/accessories.jpg)

## 🎥Demo

[![1S Demo](https://img.youtube.com/vi/6urxjE-0fT8/maxresdefault.jpg)](https://youtu.be/6urxjE-0fT8)

[![2S Demo](https://img.youtube.com/vi/g8prIVViN2I/maxresdefault.jpg)](https://youtu.be/g8prIVViN2I)


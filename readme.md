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



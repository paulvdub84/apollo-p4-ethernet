| Supported Targets | ESP32-P4 | 
| ----------------- | -------- | 

# ESP32-P4-ETH-POE with CC1101 RF Transceiver


## Overview

Building a DIY RF logger with MQTT output to an MQQT broker running on Home Assistant for Oil Tank Level status, but having the ability to read other 433Mhz signals, and also send 433Mhz signals to devices using this communication standard such as garage door, gate mover, or simple home devices.

### Hardware Required

ESP32-P4
CC1101 Chip

#### Pin Assignment

The pin assignments used in this project are:

CC1101 Pin	Label	    ESP32-P4 GPIO	    Role
Pin 1	      GND	      GND	              Ground
Pin 2	      VCC	      3.3V	            Power (3.3V Only)
Pin 3	      GDO0	    GPIO 14	          TX Control (Gate/Garage)
Pin 4	      CSN	      GPIO 5	          SPI Chip Select
Pin 5	      SCK	      GPIO 15	          SPI Clock
Pin 6	      MOSI	    GPIO 16	          SPI Data Out
Pin 7	      MISO	    GPIO 17	          SPI Data In
Pin 8	      GDO2	    GPIO 27	          RX Interrupt 

### Configure the project

```
idf.py menuconfig
```

### Build, Flash, and Run



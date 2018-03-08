# Carbon Rover

**Rev 3:**

Change Log:
- BlueTooth Controller Support:
  - Using HC-05 BT <-> UART Module
- Miscellaneous:
  - Code Cleanup
  - Pinout Changes
  - Uses non-upstream Zephyr branch for UART6

***

**[Rev 2:](https://github.com/96boards-projects/carbon_rover/releases/tag/v2.0)**

Change log:
- Reduced response time:
  - Threads now use k_yield() in place of k_sleep() to switch threads.
- NeoPixels over i2c
  - 22 Neopixels arranged around the outer edges of the rover change color according to the output of the ultrasonic sensors.
  - Arduino is used to directly convert i2c data to output for NeoPixels.
  - Carbon sends the i2c data in 4 bytes per pixel.
    - Byte 1: Pixel Location on the strip
    - Byte 2: Red 0-255
    - Byte 3: Green 0-255
    - Byte 4: Blue 0-255
- Miscellaneous Pinout Changes.

***

**[Rev 1:](https://github.com/96boards-projects/carbon_rover/releases/tag/v1.0)**

The Carbon Rover is a 4 WD Robot that uses 6 ultrasonic sensors and 4 IR sensors for object and edge detection.
The ultrasonic sensors make sure that the rover doesn't bump into objects in it's path and the IR sensors are used to detect edges so it doesn't fall off.

The Rover is programmed using Zephyr RTOS and heavily relies on its multi-threading functionality to collect sensor data and control the motors at the same time.

TODO:
  - NeoPixel Implementation
  - Bluetooth Controller Implementation

# Table of Contents

- [1) Hardware](#1-hardware)
   - [1.1) Hardware requirements](#11-hardware-requirements)
- [2) Software](#2-software)
   - [2.1) Build Environment Setup](#21-build-environment-setup)
- [3) HC-05 Module Setup](#3-hc-05-module-setup)
- [4) Carbon Rover](#5-carbon-rover)
   - [4.1) Hardware setup](#51-hardware-setup)
   - [4.2) Building](#52-building)
   - [4.3) Video Demonstration](#53-video-demonstration)

# 1) Hardware

## 1.1) Hardware requirements

- [96Boards Ccarbon IE](https://www.96boards.org/product/carbon/)
- 5v 2A USB Power Bank
- [6x Ultrasonic Sensors HC-SR04](https://www.robomart.com/buy-4-pin-ultrasonic-sensor-module-arduino-raspberry?search=hcsr04&dsearch=hcsr04)
- 6x 1KOhm Resistors
- 6x 2KOhm Resistors
- [4x IR Edge Detection Sensors](https://www.robomart.com/ir-based-digital-color-sensor-black-white)
- [1x L9110S Motor Controller](https://www.amazon.com/L9110S-Stepper-Driver-Atomic-Market/dp/B01ACIALJ4/ref=sr_1_1?ie=UTF8&qid=1516181215&sr=8-1&keywords=l9110s)
- [1x 4WD Car Kit](https://www.amazon.in/Rees52-4-Wheel-Chassis-Encoder-Arduino/dp/B071Z3XTQH/ref=sr_1_6?s=industrial&ie=UTF8&qid=1516181373&sr=1-6&keywords=car+kit)
- [22x NeoPixel LED Strip (Flexible)](https://www.amazon.com/Adafruit-NeoPixel-Digital-Weatherproof-LED-1m/dp/B072MNYMGS/ref=sr_1_7?ie=UTF8&qid=1518261705&sr=8-7&keywords=neopixel)
- [1x Arduino Nano](https://store.arduino.cc/usa/arduino-nano)
- [1x Logic Level Shifter](https://www.amazon.com/Logic-Converter-Bi-Directional-Module-Arduino/dp/B014MC1OAG/ref=sr_1_7?ie=UTF8&qid=1518261841&sr=8-7&keywords=logic+level+converter)
- Miscellaneous: Quantity as per required:
  - Connecting wires
  - Soldering Kit
  - Screwdriver Set
  - Double sided tape
- [1x HC05 BlueTooth Module](https://www.amazon.com/dp/B01G9KSAF6/ref=sspa_dk_detail_2?psc=1&pd_rd_i=B01G9KSAF6&pd_rd_wg=29bDC&pd_rd_r=HRSY8D8790ZFZDKX1S5Z&pd_rd_w=KiJpI)
- 1x Android Device with BlueTooth


# 2) Software

## 2.1 Build Environment Setup

- Follow the official Zephyr documentation to setup build environment
	- [Linux](http://docs.zephyrproject.org/getting_started/installation_linux.html)
	- [macOS](http://docs.zephyrproject.org/getting_started/installation_mac.html)
	- [Windows](http://docs.zephyrproject.org/getting_started/installation_mac.html)
- [Install Arduino IDE](https://www.arduino.cc/en/Main/Software)
- [Install Adafruit NeoPixel Library](https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library-installation)
- [Bluetooth Controller App](https://play.google.com/store/apps/details?id=braulio.calle.bluetoothRCcontroller&hl=en)

# 3) HC-05 Module Setup

**The HC-05 Module directly connects to the Carbon, but we need to prepare it before we can proceed with it.**
- Open the ".ino" file under "arduino/arduino_hc05" folder with Arduino IDE.
- Connect Arduino to Host PC and flash the .ino file.
- Disconnect the Arduino from any power source.
- Connect the HC-05 to the Arduino in the following manner:

| Arduino | HC-05 |
|:-------:|:-----:|
| 5v      | 5v    |
| GND     | GND   |
| 11      | RX    |
| 10      | TX    |
| 09*     | KEY*  |

- **If there is a button near the key/enable pin DO NOT connect the key pin**
- Connect the Arduino to the host PC, make sure the KEY Button is pressed while you connect the Arduino to the Host PC
- Open the Serial Monitor in the Arduino IDE
  - Set the Baud to 38400
  - Set it to "Both NL & CR"
  - It should say "Enter AT Commands"
- Type 'AT' (without the quotes) and click Send
  - If it says "OK" the everything is working as expected
  - If it gives "Error(0)", try entering 'AT' again
  - If there is no output, something is wrong with the setup, make sure you have followed all the steps
- Set BlueTooth Name
  - Enter 'AT+NAME=CARBON_ROVER'
  - It should respond with 'OK'
- Set UART Baud Rate
  - Enter 'AT+UART=115200'
  - It should respond with 'OK'
- Disconnect the HC-05 module from Arduino.

# 4) Carbon Rover

![](https://raw.githubusercontent.com/SeeedDocument/BLE-Carbon/master/img/pinout.png)

## 4.1) Hardware Setup

### Pinout

#### IR Edge Sensor

| IR Sensor Position | Carbon Pin |
|:------------------:|:----------:|
| Front Left         | PB12       |
| Front Right        | PB15       |
| Back Left          | PB14       |
| Back Right         | PB13       |
| +                  | +3v3       |
| GND                | GND        |


#### Ultrasonic Sensors

**All Ultrasonic sensors are connected using a Voltage Divider using a combination of 2kOhm and 1KOhm resistors for the echo pin as the Carbon is limited to 3v3 on the GPIO pins. Please follow [this blog](https://www.96boards.org/blog/zephyr-hcsr04/) for the details.**

![](https://www.96boards.org/assets/images/blog/hcsr04-voltage-divider.svg)

| Ultrasonic Sensor Position | Carbon Pin |
|:---------------------------|------------|
| Front Left Trigger         | PC2        |
| Front Left Echo            | PC4        |
| Front Center Trigger       | PC0        |
| Front Center Echo          | PC8        |
| Front Right Trigger        | PC3        |
| Front Right Echo           | PC5        |
| Back Right Trigger         | PC1        |
| Back Right Echo            | PC9        |
| Back Center Trigger        | PB3        |
| Back Center Echo           | PB10       |
| Back Left Trigger          | PB8        |
| Back Left Echo             | PB9        |
| +                          | +5v        |
| GND                        | GND        |

### Motor Controller

| Motor Controller | Carbon |
|:----------------:|:------:|
| A1-A             | PA0    |
| A1-B             | PA2    |
| B1-A             | PA3    |
| B1-B             | PA1    |

### HC-05

| HC-05      | Carbon             |
|:----------:|:------------------:|
| 5v or VCC  | 3v3                |
| GND        | GND                |
| TX         | UART6* or UART2 RX |
| STATE      | PC6                |

**NOTE: The code is using UART6, but that is not working yet upstream. However if you want to use UART2 just modify the line ```#define UART_DEV CONFIG_UART_STM32_PORT_6_NAME``` in main.c and "CONFIG_UART_STM32_PORT_6=y" in prj.conf**

### Arduino to Carbon

**NOTE: This uses a Bi-directional Logic Level Shifter to shift from 5v Logic to 3v3 Logic. This Setup might work without a LLS using Arduino Pro Mini but has not been tested**

| Arduino Nano | Level Shifter HV | <-> | Level Shifter LV | Carbon     |
|:------------:|:----------------:|:---:|:----------------:|:----------:|
| A5 (SCL)     | HV1              | <-> | LV1              | PB6 (SCL)  |
| A4 (SDA)     | HV2              | <-> | LV2              | PB7 (SDA)  |
| 5V           | HV               | <-> | LV               | VCC2 (3v3) |
| GND          | GND              | <-> | GND              | GND        |

### Arduino to NeoPixel

| Arduino Nano | NeoPixel |
|:------------:|:--------:|
| D9           | DIN      |
| 5v           | VCC/5V   |
| GND          | GND      |

## 4.2) Building and Flashing

- Clone This Repository
  ```$ git clone https://github.com/96boards-projects/carbon_rover```
- If you want to use UART6, use this zephyr source branch
  ```$ git clone https://github.com/ric96/zephyr -b uart6-96b_carbon```
- Copy the folder ```rover``` into the root directory of Zephyr source.
- Build
  ```shell
  $ cd /path/to/zephyr/root
  $ source zephyr-env.sh
  $ cd rover
  $ mkdir build; cd build
  $ cmake -DBOARD=96b_carbon ..
  $ make
  ```
- Connect the micro-USB cable to the USB OTG Carbon port and to your computer. The board should power ON. Force the board into DFU mode by keeping the BOOT0 switch pressed while pressing and releasing the RST switch
- You should see following confirmation on your Linux host:
```shell
$ dmesg
usb 1-2.1: new full-speed USB device number 14 using xhci_hcd
usb 1-2.1: New USB device found, idVendor=0483, idProduct=df11
usb 1-2.1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
usb 1-2.1: Product: STM32 BOOTLOADER
usb 1-2.1: Manufacturer: STMicroelectronics
usb 1-2.1: SerialNumber: 3574364C3034
```
- Flash
```shell
$ sudo make flash
```

- [Flash the arduino_neopixel.ino file on the Arduino Nano](https://www.arduino.cc/en/Guide/Environment#toc9

- At this moment your rover should come alive.

# 5) Video Demonstration
**Rev 2**

[![Demo](https://img.youtube.com/vi/jmrMJPTKGsc/0.jpg)](https://www.youtube.com/watch?v=jmrMJPTKGsc)

**Rev 1**

[![Demo](https://img.youtube.com/vi/BEC8CVmJyRc/0.jpg)](https://www.youtube.com/watch?v=BEC8CVmJyRc)

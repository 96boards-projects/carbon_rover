# Carbon Rover

The Carbon Rover is a 4 WD Robot that uses 6 ultrasonic sensors and 4 IR sensors for object and edge detection.
The ultrasonic sensors make sure that the rover doesn't bump into objects in it's path and the IR sensors are used to detect edges so it doesn't fall off.

The Rover is programmed using Zephyr RTOS and heavily relies on its multi-threading functionality to collect sensor data and control the motors at the same time.

Since this is Rev 1. there is no control mechanism and the rover follows a pre-defined path, Rev 2 will possibly include control via Bluetooth.

# Table of Contents

- [1) Hardware](#1-hardware)
   - [1.1) Hardware requirements](#11-hardware-requirements)
- [2) Software](#2-software)
   - [2.1) Build Environment Setup](#21-build-environment-setup)
- [3) Carbon Rover](#3-carbon-rover)
   - [3.1) Hardware setup](#31-hardware-setup)
   - [3.2) Building](#32-building)
   - [3.3) Video Demonstration](#33-video-demonstration)

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
- Miscellaneous: Quantity as per required:
  - Connecting wires
  - Soldering Kit
  - Screwdriver Set
  - Double sided tape


# 2) Software

## 2.1 Build Environment Setup

- Follow the official Zephyr documentation to setup build environment
	- [Linux](http://docs.zephyrproject.org/getting_started/installation_linux.html)
	- [macOS](http://docs.zephyrproject.org/getting_started/installation_mac.html)
	- [Windows](http://docs.zephyrproject.org/getting_started/installation_mac.html)

# 3) Carbon Rover

![](https://raw.githubusercontent.com/SeeedDocument/BLE-Carbon/master/img/pinout.png)

## 3.1) Hardware Setup

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
| Front Center Trigger       | PC3        |
| Front Center Echo          | PC5        |
| Front Right Trigger        | PC6        |
| Front Right Echo           | PC8        |
| Back Right Trigger         | PC7        |
| Back Right Echo            | PC9        |
| Back Center Trigger        | PB6        |
| Back Center Echo           | PB7        |
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

## 3.2) Building and Flashing

- Clone This Repository
  ```$ git clone https://github.com/96boards-projects/carbon_rover```
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
- At this moment your rover should come alive.

## 3.3) Video Demonstration

[![Demo](https://img.youtube.com/vi/BEC8CVmJyRc/0.jpg)](https://www.youtube.com/watch?v=BEC8CVmJyRc)

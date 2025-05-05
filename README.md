# Joystick controller for Hoverboard-Firmware-Hack-Gen2.x

Control [Hoverboard-Firmware-Hack-Gen2.x](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x) with a Raspberry Pi or any Linux/macOS system using a joystick. This project could also be adapted to other firmware, such as [hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC).

Control  with a Raspberry Pi or any Linux/macOS system using a joystick. Could be adapted to other firmwares like 
## Why do you want this?
Why waste a Raspberry Pi 5 with 8 GB of RAM to control a hoverboard? Maybe it doesn’t make sense for a final project, but it can be useful when testing motors and controllers on your bench.

## Overview
This project allows you to control a hoverboard running the Hoverboard-Firmware-Hack-Gen2.x firmware using a joystick/gamepad connected to your computer. It communicates with the hoverboard via UART and supports flexible configuration of joystick axes and buttons.

## Features
- Real-time joystick/gamepad control of hoverboard motors
- Configurable UART device and baud rate
- Selectable joystick, axis, and arming button
- Live feedback display of hoverboard telemetry
- Cross-platform: Linux and macOS supported


## Required hardware
A Raspberry Pi has built-in UARTs that can be used. If you're using a Mac or another computer, you'll need a USB to UART TTL module. I tested with [this one](https://www.aliexpress.com/item/1005005847955898.html).

## Dependencies
### Linux
`sudo apt-get install cmake libsdl2-dev`

I've only tested on Raspberry Pi 5.

### Mac
`brew install sdl2`

## Building
```sh
mkdir build
cd build
cmake ..
make
```

## Usage
Run the program with required options:
```sh
./hoverboard_control -d <uart_device> -a <axis> -m <button> [options]
```

### Command-line Options
- `-d <device>`    UART device (required, e.g. `/dev/ttyUSB0` or `/dev/cu.usbserial-0001`)
- `-a <axis>`      Joystick axis for speed (required, e.g. `3`)
- `-m <button>`    Joystick button for arming (required, e.g. `0`)
- `-b <baud>`      Baud rate (default: B19200; options: 9600, 19200, 38400, 57600, 115200)
- `-j <index>`     Joystick index (default: 0)
- `-l`             List available joysticks and exit
- `-h`             Show help/usage

### Example
```sh
./hoverboard_control -d /dev/cu.usbserial-0001 -a 3 -m 23
```

## How it works
- The program reads joystick events using SDL2.
- The selected axis controls the hoverboard speed (mapped from full negative to full positive power).
- The selected button must be held to "arm" the controller and send speed commands.
- Telemetry from the hoverboard is displayed live in the terminal.



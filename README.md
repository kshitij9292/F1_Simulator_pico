# F1 Simulator Pico

DIY Formula 1 simulator controller built using two Raspberry Pi Pico boards.

## Overview

This project is split into two independent USB HID devices.

### Pico 1 – Steering Base

* Force Feedback using a **775 DC motor**
* Quadrature steering encoder
* Clutch, Brake and Accelerator pedals
* USB HID Steering Wheel
<img width="1744" height="856" alt="Circuit_Diagram" src="https://github.com/user-attachments/assets/6f39965e-1f2d-42e0-a2ff-d1cae579b441" />

### Pico 2 – F1 Wheel

* Push buttons
* Rotary encoders
* Limit switches
* potentiometer as position switch
* 16×2 LCD
* USB HID Game Controller
<img width="1438" height="851" alt="wheel" src="https://github.com/user-attachments/assets/782866e9-46de-47dd-8c6c-0545cb7d1501" />

## Hardware

* 2 × Raspberry Pi Pico
* 775 DC Motor
* BTS7960 (IBT-2) Motor Driver
* Quadrature Encoder
* 3 × Pedal Potentiometers
* 16×2 LCD
* Push Buttons
* Rotary Encoders
* Toggle Switches

## Software

* Arduino IDE 2.x
* Earle Philhower Arduino-Pico Core
* USB Stack: **Pico SDK**

Install the RP2040 board package:

```
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
```

Then install **Raspberry Pi Pico/RP2040** by **Earle F. Philhower III** from the Arduino Boards Manager.

## Folder Structure

```text
F1_Simulator_pico/
├── SteeringBase/
├── F1Wheel/
├── CAD/
├── Images/
└── README.md
```

## Features

* USB HID Steering Wheel
* USB HID Button Box
* Force Feedback
* Modular design
* Low latency
* Easy to expand

## Planned

* CAD files
* Wiring diagrams
* PCB files
* Assembly guide
* Configuration utility

## License

MIT License.

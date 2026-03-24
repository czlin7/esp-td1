# ESP - Autonomous Line-Following Buggy Firmware
**Embedded Systems Project 2025-26**

## Overview
This repository contains the firmware for an autonomous line-following buggy developed as part of the Embedded System Project (ESP).

This system is built around an **STM32 Nucleo-F401RE (ARM Cortex-M4, 84 MHz)**.

The buggy is capable to do the following autonomously:
- Tracks a white line on a black track
- Handles slopes (≤ 18°)
- Tolerates line breaks (≤ 6 mm)
- Detects end-of-track
- Executes Bluetooth-triggered turnaround

This firmware implements a robust real-time control architecture with strctured modular design and safe motor actuation.

## System Architecture
### High-Level Control Structure
```
Line Sensors (ADC @ 500 Hz)
        ↓
Line Estimation (Weighted / Quadratic Interpolation)
        ↓
Outer PID Controller (Line Position)
        ↓
Speed Setpoint Adjustment
        ↓
Inner PID Controller (Wheel Speed)
        ↓
PWM Output → H-Bridge → Motors
```
with supporting subsystems:
- Finite State Machine (FSM)
- Battery monitoring
- BLE UART data command and monitor interface

  

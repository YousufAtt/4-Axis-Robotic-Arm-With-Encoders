# AS5600 Magnetic Encoder Integration for 4-DOF Robotic Arm (Work in Progress !)

## Overview

This project integrates **AS5600 magnetic rotary encoders** into a **4-DOF articulated robotic arm** to provide joint-angle feedback for calibration, position monitoring, homing, and future closed-loop control.

Each encoder is mounted directly at a rotational joint and measures the angular position of the joint using a diametrically magnetized magnet attached to the rotating shaft. Unlike mechanical potentiometers, the AS5600 provides **contactless absolute angular position sensing**, making it well suited for repeated robotic joint motion.

## System Purpose

The encoders are used to improve the arm's ability to:

- Measure joint angles in real time
- Establish repeatable reference positions
- Verify commanded stepper-motor motion
- Detect accumulated positioning error
- Enforce software-defined joint limits
- Assist with homing and calibration
- Support future closed-loop position control

## Robotic Arm

The system is designed for a **4-DOF articulated robotic manipulator** consisting of rotational joints such as:

1. Base rotation
2. Shoulder
3. Elbow
4. Wrist

The AS5600 sensors provide independent angular feedback for these joints.

## Hardware

- 4-DOF articulated robotic arm
- AS5600 magnetic rotary encoders
- Diametrically magnetized magnets
- Arduino Mega
- Stepper motors
- Stepper motor drivers
- Limit switches / homing switches
- Custom encoder mounting brackets
- External power supply

Depending on the final electronics configuration, an **I2C multiplexer such as the TCA9548A** may also be used.

## Encoder Operation

The AS5600 detects the orientation of a magnetic field produced by a diametrically magnetized magnet positioned above the sensor.

The encoder provides a **12-bit absolute angular measurement**, corresponding to:

- `0–4095` raw counts
- `0–360°` angular position

The measured joint angle can be calculated using:

```cpp
angle = rawPosition * 360.0 / 4096.0;


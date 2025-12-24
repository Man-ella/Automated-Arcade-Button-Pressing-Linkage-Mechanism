# Automated-Arcade-Button-Pressing-Linkage-Mechanism
## Overview

This project implements the electronics and control system for an arcade-style button-pressing mechanism. This code was developed as part of a team-based mechanical engineering design project class at Columbia University focused on integrating mechanical design, manufacturing, electronics, and controls. My primary contributions included controls architecture design, PID tuning, state machine implementation, and system integration.The system uses a single DC motor with encoder feedback, a finite state machine, and closed-loop PID control to autonomously move a linkage to multiple button locations and actuate each button using a solenoid.
The controller was designed to be fast, repeatable, and robust, enabling the four bar linkage mechanism to press illuminated buttons continuously under competition conditions.


## System Architecture

The control system is organized hierarchically:

A high-level finite state machine handles task sequencing (calibration, motion, waiting, and button actuation).

A low-level PID controller provides accurate motor position control using encoder feedback.

Hardware interrupts are used for encoder position tracking to ensure reliable velocity and position estimation.

A solenoid actuator presses each button exactly once per cycle.

## Key Features

Finite State Machine with calibration, motion, and wait states

Closed-loop PID position control

Interrupt-based quadrature encoder reading

Limit switch–based homing and safety

Solenoid-based button actuation

Real-time target selection based on button inputs

Designed for high-speed, repeatable operation

## State Machine Description

The controller operates through the following states:

CALIBRATE – Homes the mechanism using a limit switch and zeros the encoder.

FIRST / SECOND / THIRD BUTTON – Moves the linkage to predefined target positions using PID control.

WAIT – Briefly holds position, actuates the solenoid once, and determines the next target based on button inputs.

This structure allows the system to run autonomously with minimal logic complexity.

## Hardware Components

Arduino microcontroller

DC motor with quadrature encoder

H-bridge motor driver

Limit switch for calibration

12 V solenoid actuator

Toggle switch (enable/disable control)

External button input signals

## Control Parameters

The PID gains and target positions were tuned experimentally to balance speed and accuracy:

Proportional, integral, and derivative gains are defined in the code

Encoder counts are used as the primary position reference

Anti-windup logic prevents controller saturation

## How to Run

Upload the code to an Arduino-compatible board

Connect the motor, encoder, solenoid, and switches as defined in the pin assignments

Power the system and enable the toggle switch

The mechanism will automatically calibrate, detect active buttons, and begin operation

## Contributors
Kelsey Anthony, Raquel Conard, Sophya Elkihel, Yevgeniy Yesilevskiy

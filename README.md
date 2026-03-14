# UW EE201: Autonomous Line Following Robot

🏆 **All-Time Record Holder:** This robot achieved the all-time fastest track record of **24.5 seconds** in the University of Washington EE201 Line Following Car Competition, securing first place by a margin of over 6 seconds.

This repository contains the complete hardware, mechanical, and software design files for our group's autonomous line-following robot. The system features a custom 3D printed chassis, a bespoke sensor array PCB, and an advanced Arduino-based closed-loop PID control system optimized for aggressive cornering and high-speed straightaways.

## Repository Structure

Based on the root directory, the project files are organized as follows:

* **`ee201_final_project_sensor_array/`**
    Contains the schematic and board layout files for the custom 7-sensor Light Dependent Resistor (LDR) analog front-end and tuning interface.
* **`EE201_group6_chassis.stl`**
    The 3D CAD file for the custom-designed robot chassis, optimized for a low center of gravity and precise sensor-to-ground focal distance.
* **`final_line_follower.ino`**
    The final, competition-winning firmware featuring time-based dynamic throttling and quadratic error scaling.
* **`10_5_line_follower_code.ino`**
    A legacy development iteration of the line following logic.

## Hardware Architecture

### Electronics and PCB Design (`ee201_final_project_sensor_array/`)
The electrical system is driven by an Arduino Mega interfaced with an Adafruit Motor Shield V2. A custom PCB was engineered to streamline the analog front-end.
* **Sensor Array:** Seven LDRs configured in voltage divider circuits, spaced precisely to capture standard competition line widths.
* **Control Interface:** Four onboard potentiometers routed to analog pins (A0 through A3) allow for live parameter tuning of Speed, Proportional, Integral, and Derivative gains without requiring firmware recompilation.

### Mechanical Design (`EE201_group6_chassis.stl`)
The mechanical chassis was manufactured via 3D printing to house the hardware securely. 
* It securely mounts the DC motors, battery payload, and microcontroller stack to prevent center-of-gravity shifts during high-speed cornering. 
* It integrates the sensor mount to keep the LDR PCB at a fixed, optimal distance from the track surface to maximize the black-to-white contrast ratio.

## Software Architecture

The primary firmware (`final_line_follower.ino`) is built in C++ and implements an advanced kinematic control loop to handle extreme track geometry.

### Core Control Algorithms
1.  **Temporal Dynamic Throttling:** To handle a notoriously dense "stacked turn" section of the track, the code utilizes a `millis()` timer. The base speed is automatically cut by a factor of 2.3 between 8.5 seconds and 17.5 seconds into the run, ensuring maximum grip during tight corners and absolute maximum velocity on straightaways.
2.  **Quadratic Proportional Gain:** Instead of a linear multiplier, the proportional error is scaled quadratically. This flattens the response near the center line to eliminate high-speed wobbling while maintaining 100% control authority at the extreme edges for 90-degree pivots.
3.  **EMA Filtered Derivative:** An Exponential Moving Average (EMA) filter is applied to the derivative calculation to reject analog sensor noise and prevent erratic micro-corrections.
4.  **Continuous Line Memory:** The error calculation logic computes a weighted average of the darkest sensors. If the line is momentarily lost (e.g., during a violent pivot), the system defaults to the last known extreme error bound to force the robot to continue spinning until the line is reacquired.

## Setup and Calibration

1.  **Hardware Assembly:** Mount the Arduino, Motor Shield, and DC motors to the 3D printed chassis. Connect the custom PCB to the analog header block.
2.  **Firmware Upload:** Flash `final_line_follower.ino` to the Arduino Mega.
3.  **Calibration Sequence:**
    * Power the robot on while placing the sensor array strictly over the **white** surface.
    * Wait for the onboard LED indicator to blink, indicating the white calibration is complete.
    * Move the robot so the sensor array is directly over the **black** line during the delay window.
    * The robot will complete the black calibration, start the internal race clock, and immediately engage the motors.

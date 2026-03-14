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

### Electronics and PCB Design
The electrical system is driven by an Arduino Mega interfaced with an Adafruit Motor Shield V2. A custom PCB was engineered to streamline the analog front-end.
* **Sensor Array:** Seven LDRs configured in voltage divider circuits, spaced precisely to capture standard competition line widths.
* **Control Interface:** Four onboard potentiometers routed to analog pins (A0 through A3) allow for live parameter tuning of Speed, Proportional, Integral, and Derivative gains without requiring firmware recompilation.

### Mechanical Design
The mechanical chassis was manufactured via 3D printing to house the hardware securely. 
* It securely mounts the DC motors, battery payload, and microcontroller stack to prevent center-of-gravity shifts during high-speed cornering. 
* It integrates the sensor mount to keep the LDR PCB at a fixed, optimal distance from the track surface to maximize the black-to-white contrast ratio.

## Software Architecture

The primary firmware (`final_line_follower.ino`) is built in C++ and implements an advanced kinematic control loop. To achieve the record-breaking 24.5 second time, several major algorithmic optimizations were implemented to completely replace the baseline starter code logic.

### Algorithmic Advancements

* **Temporal Dynamic Throttling:** The baseline code utilized a static speed output. The winning firmware implements a clock-driven control state that tracks execution time using the hardware `millis()` timer. Between 8.5 seconds and 17.5 seconds into the run, the algorithm mathematically divides the user-defined speed boost by a factor of 2.3. This ensures the robot maintains absolute maximum velocity on the starting and ending straightaways while automatically decelerating to navigate the dense stacked-turn section with maximum grip.
* **Quadratic Proportional Gain:** The original PID implementation calculated the proportional turning force linearly. The updated control loop scales the proportional error quadratically. By multiplying the error by its absolute ratio over the maximum possible error, the algorithm heavily attenuates steering corrections near the center line. This eliminates high-speed straight-line wobbling while retaining 100 percent control authority at the extreme sensor edges for 90-degree pivots.
* **Exponential Moving Average (EMA) Derivative Filter:** The original raw discrete derivative calculation was highly susceptible to analog sensor noise, causing jitter. The modified code passes the raw derivative through an EMA low-pass filter with a smoothing factor of 0.4. This prevents erratic micro-corrections and allows the system to support a significantly higher derivative gain (kD = 170) to dampen overshoots during aggressive cornering.
* **Asynchronous Hardware Polling:** The starter code executed analog-to-digital conversions to read all four potentiometers during every loop iteration. The winning code introduces a modulo counter to restrict potentiometer polling to once every 5000 loop cycles. This dramatically reduces loop execution time, allowing the microcontroller to sample the photoresistors and update the motor PWM outputs at a much higher frequency. 
* **Hardware Safety Interlocks:** A dedicated safety algorithm continuously monitors the raw sensor array data to detect abnormal environmental states. If the system detects all darkness (indicating the robot has been lifted or driven off a table) or all white (indicating total line loss), it circumvents the PID calculation and instantly cuts power to the drive motors to prevent runaway hardware damage.

## Setup and Calibration

1. **Hardware Assembly:** Mount the Arduino, Motor Shield, and DC motors to the 3D printed chassis. Connect the custom PCB to the analog header block.
2. **Firmware Upload:** Flash `final_line_follower.ino` to the Arduino Mega.
3. **Calibration Sequence:**
    * Power the robot on while placing the sensor array strictly over the **white** surface.
    * Wait for the onboard LED indicator to blink, indicating the white calibration is complete.
    * Move the robot so the sensor array is directly over the **black** line during the delay window.
    * The robot will complete the black calibration, start the internal race clock, and immediately engage the motors.

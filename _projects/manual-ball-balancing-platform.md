---
layout: project
title: "Manual Ball-Balancing Platform Robot"
description: "A 3DOF manual ball-balancing platform controlled via joystick, featuring an Arduino Uno, 3 stepper motors, a resistive touchscreen, and an LCD interface for a 30-second balancing game. Developed for MTE-392 Embedded Systems."
date: 2025-05-31
categories: [Robotics, Embedded Systems, Arduino, Control Systems, Mechatronics]
featured_image: "/assets/images/projects/ball-balancing/featured.png"
github_url: "https://github.com/Yosuf-Soliman/Portfolio"
demo_url: "#"
interactive_plot: false

schematics:
  - file: "/assets/images/projects/ball-balancing/electronic-circuit.jpg"
    description: "Figure 3 — Electronic Circuit showing Arduino Uno, CNC shield, and stepper drivers"
  - file: "/assets/images/projects/ball-balancing/flow-chart.jpg"
    description: "Figure 5 — Software flow chart for game logic and system states"

components:
  - name: "Arduino Uno"
    quantity: 1
    description: "Central microcontroller managing all digital and analog I/O, game logic, and stepper coordination."
  - name: "Stepper Motors & Drivers"
    quantity: 3
    description: "Three stepper motors controlled via stepper motor drivers on a CNC shield for platform movement (X, Y, and Z tilting)."
  - name: "Resistive Touchscreen"
    quantity: 1
    description: "Tracks the steel ball's X and Y coordinates on the platform via analog inputs."
  - name: "Joystick"
    quantity: 1
    description: "Provides tilt direction and magnitude through analog readings, plus digital buttons for switching modes."
  - name: "LCD Screen"
    quantity: 1
    description: "Tilted 20° for better visibility, displays real-time scores and game status."
  - name: "3-Amp Power Supply"
    quantity: 1
    description: "Provides sufficient current for the three stepper motors and the control electronics."
  - name: "3D Printed Structure"
    quantity: 1
    description: "Custom-built 3RRS PRO controller casing and a 3DOF platform housing all actuators and the touchscreen."

gallery:
  - type: "image"
    file: "/assets/images/projects/ball-balancing/platform-solidworks.jpg"
    description: "Figure 1 — SolidWorks Model of the Platform"
  - type: "image"
    file: "/assets/images/projects/ball-balancing/controller-solidworks.jpg"
    description: "Figure 2 — SolidWorks Model of the 3RRS PRO Controller"
  - type: "image"
    file: "/assets/images/projects/ball-balancing/total-system.jpg"
    description: "Figure 4 — Total System setup with the platform, controller, and laptop"
---

## Project Overview

This project showcases a 3DOF (three degrees of freedom) manual ball-balancing platform created as part of the MTE-392 Embedded Systems course at Kadir Has University. The platform allows a user to control the platform's tilt angle using a joystick to maintain a steel ball centered. It combines control systems, real-time embedded software, and mechanical integration in a hands-on and interactive way.

The manual game mode with joystick input serves as an effective educational demonstration of embedded systems concepts, particularly in the areas of digital/analog I/O and user feedback systems.

**Team:** Islam Hesham Allam, Yosuf Soliman, Abdala Ismail Hasen  
**Project Mentors:** Dr. Ahmet Fevzi Bozkurt  
**Institution:** Kadir Has University, Faculty of Engineering and Natural Sciences (Spring 2025)

---

## System Architecture & Hardware

The physical structure is divided into two primary units: a custom-built, game-style controller (the "3RRS PRO") and the 3DOF platform itself.

### 3RRS PRO Controller
Inspired by modern game controllers, the 3RRS PRO houses all the main electronics, wires, and the joystick. It features an LCD screen tilted at 20° for optimal visibility and swappable handles for a better user experience.

### 3DOF Platform
A 3D printed platform houses the three stepper motors and the resistive touchscreen. The structural design allows precise movement across three degrees of freedom (X, Y, and Z tilting).

### Electronics & Connections
* **Arduino Uno:** Central microcontroller handling all computations and logic.
* **CNC Shield & Motor Drivers:** Used for clean wiring and driving the three stepper motors with precise step control.
* **Resistive Touchscreen:** Acts as the primary sensor, tracking the ball's X and Y positions.
* **Joystick:** Provides analog input for tilt direction/magnitude and digital input for mode switching (e.g., "ready" state).
* **Power Supply:** A 3-Amp power supply ensures stable delivery to the actuators and logic circuits.

---

## Embedded Systems Implementation

### Digital and Analog I/O
* **Analog Inputs:** Joystick potentiometers to control tilt direction and angle; touchscreen tracking the ball's coordinates.
* **Digital Inputs:** Joystick buttons used for readiness control.
* **Analog Outputs:** Real-time display of the Best & Current scores on the LCD screen; motor shaft radial position based on joystick movement.
* **Digital Outputs:** Platform state shifts (e.g., home vs. ready position switch).

### Communication
The system relies on the continuous reading of the ball's position from the touchscreen and continuous reading of the joystick potentiometers for screen tilt and angle adjustments. The LCD updates are handled in real-time to reflect scores and round status.

### Software Interrupts
Software interrupts are implemented for switching the system between the home and ready positions seamlessly when the joystick button is pressed.

---

## Game Logic and Scoring Algorithm

The manual game mode challenges users to balance the ball for as long as possible within a set timeframe.

1.  **Round Timer:** A strict 30-second timer controls each round.
2.  **Performance Tracking:** The ball's time spent near the center of the touchscreen is measured continuously.
3.  **Scoring:** The user gains points the longer the ball stays near the center.
4.  **Feedback:** Both the Best and Current scores are displayed dynamically. The Best score is updated in real-time if the user breaks the previous record.
5.  **Round Completion:** After 30 seconds, the round ends automatically, and the robot notifies the user by switching back to its home position (the platform lowers down).

---

## Future Work & Applications

The concept of a ball-balancing platform can be expanded into real-world applications such as stabilization platforms, robotic surgery tools, and drone gimbal systems. 

While this specific version implemented manual control, we successfully developed an autonomous mode using **inverse kinematics and PID control**. Future plans include integrating these advanced control algorithms, enhancing sensor accuracy, and improving the structural design for real-world deployment.

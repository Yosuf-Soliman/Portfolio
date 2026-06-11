---
layout: project
title: "Simulation and Control of a Quarter-Car Suspension System"
description: "A comprehensive mathematical modeling, simulation, and PID control design for a quarter-car suspension system. Combines work from MTE294 (Simulation) and MTE394 (Control) using MATLAB, Simulink, and Simscape."
date: 2024-06-01
categories: [Control Systems, Simulation, MATLAB/Simulink, Mechatronics, Automotive]
featured_image: "/assets/images/projects/suspension/quarter-car-diagram.jpg"
github_url: "https://github.com/Yosuf-Soliman/Portfolio"
demo_url: "#"
interactive_plot: false

schematics:
  - file: "/assets/images/projects/suspension/free-body-diagram.jpg"
    description: "Free Body Diagram showing forces exerted by springs and dampers on sprung and unsprung masses."
  - file: "/assets/images/projects/suspension/state-space-matrices.jpg"
    description: "State Space matrices (A, B, C, D) used to represent the system dynamics."
  - file: "/assets/images/projects/suspension/simulink-diff-eq.jpg"
    description: "Simulink model constructed using the derived differential equations of motion."
  - file: "/assets/images/projects/suspension/pid-controllers-design.jpg"
    description: "Two PID controllers designed in Simulink to control the output displacement."
  - file: "/assets/images/projects/suspension/stateflow-decision-logic.jpg"
    description: "Stateflow chart used for dynamic switching between two PID controllers during runtime."
  - file: "/assets/images/projects/suspension/simscape-model.jpg"
    description: "SimScape physical modeling environment for intuitive system simulation."

components:
  - name: "MATLAB & Simulink"
    quantity: 1
    description: "Used for mathematical modeling, transfer function calculations, and simulating system responses."
  - name: "Simscape"
    quantity: 1
    description: "Provided a physical modeling environment to create realistic block representations of mechanical components."

gallery:
  - type: "image"
    file: "/assets/images/projects/suspension/closed-loop-step-response.jpg"
    description: "Closed-loop step response comparing PID1 and PID2 performance."
  - type: "image"
    file: "/assets/images/projects/suspension/root-locus-plot.jpg"
    description: "Root locus plot indicating how closed-loop pole positions change as a function of varying gain."
  - type: "image"
    file: "/assets/images/projects/suspension/damping-comparison.jpg"
    description: "Simulink displacement outputs demonstrating a critically damped scenario."
---

## Project Overview

This project simulates and analyzes the performance of a quarter-car suspension system to evaluate vehicle handling and passenger comfort. A quarter-car model is a simplified representation of a vehicle suspension system that considers only the vertical dynamics of a single wheel. This portfolio entry combines two distinct academic focuses: mechanical simulation (MTE294) and system dynamics and control (MTE394).

**Team Members:** Yosuf Soliman, Abdala Ismail Hasen, Islam Hesham Allam, Abdelrahman Abdulsalam, and Islam Refaie.  
**Supervisors:** Faraz Tehranizadeh (MTE294).  
**Institution:** Kadir Has University, Department of Mechatronics Engineering (Spring 2024).

---

## System Modeling and Mathematical Representation

The physical system is modeled using two primary masses: a sprung mass representing the vehicle body (m₁) and an unsprung mass representing the wheel and suspension components (m₂). 

### Equations of Motion
The forces acting on the system include spring forces and damping forces. Derived from free-body diagrams, the equations of motion for the system are as follows:
* **Sprung Mass (m₁):** m₁q̈₁ = -k₁q₁ + k₂(q₂ - q₁) + c₁(q̇₂ - q̇₁)
* **Unsprung Mass (m₂):** m₂q̈₂ = -k₂(q₂ - q₁) - c₁(q̇₂ - q̇₁) + u

### Simulation Parameters
The initial baseline simulation parameters were defined in MATLAB as follows:
* **Vehicle Mass (m₁):** 290 kg
* **Wheel Mass (m₂):** 15 kg
* **Spring Stiffness (k₁, k₂):** 16200 N/m and 191000 N/m
* **Damping Coefficients (c₁, c₂):** 1000 Ns/m and 2500 Ns/m

### State-Space and Transfer Function
The system was defined using a State-Space representation formulated as:
* **Ẋ = AX(t) + Bu(t)**
* **Y = CX(t) + Du(t)**

The state variables chosen were the displacements and velocities of both the sprung and unsprung masses. Using these representations, the system's Transfer Function was calculated to evaluate the input-output relationship:

**H(s) = (0.3333s + 0.5556) / (s⁴ + 2s³ + 4.667s² + 1.333s + 2.222)**

---

## System Simulation (MTE294)

Simulink was utilized to provide a graphical interface for simulating the quarter-car suspension system, utilizing block diagrams to represent system equations. 

### Road Profile Scenarios
To evaluate ride comfort, a Signal Editor was used to provide two primary simulation scenarios:
* **Sidewalk:** Simulated a 10 cm height step input for a duration of 4 seconds.
* **Speed Bump:** Simulated a 1 meter bump for a duration of 3 seconds.

### Damping Analysis
The system's behavior was observed by calculating the damping coefficient ζ = c₁ / (2√(m₁k₁)). The system's response was plotted under four distinct conditions:
* **Original Damping:** c₁ = 1000
* **Critical Damping:** Evaluated at c₁ = 4335, resulting in the system stopping oscillation in the shortest time possible.
* **Under-Damped:** Evaluated at c₁ = 2167.5 (4335 × 0.5)
* **Over-Damped:** Evaluated at c₁ = 6502.5 (4335 × 1.5)

Additionally, Simscape was incorporated to create a physical modeling environment, providing realistic block representations of components like axles and vehicle bodies directly linked to a physical solver.

---

## System Control and Stability (MTE394)

### PID Controller Design
To manage the output displacement x₁(t) in response to an input force, two separate PID controllers were designed. By modifying gains, the controllers were aimed at reducing steady-state error, improving transient response, and stabilizing the system. 
* **PID1:** Kp = 3.5, Ki = 2.3, Kd = 3.5
* **PID2:** Kp = 10, Ki = 10, Kd = 10

Implementing closed-loop control with these PID controllers yielded shorter rise times, less overshoot, quicker settling, and reduced steady-state error compared to open-loop performance.

### Dynamic Controller Switching (Stateflow)
Decision logic was established using a Stateflow chart in Simulink to enable dynamic controller switching during simulations. This allowed real-time transitions between PID1 and PID2 based on an external input signal, ensuring only one controller configuration was active at any given moment.

### Stability Analysis
Analytical and simulation-based methods were utilized to verify stability.
* **Poles and Eigenvalues:** The calculated poles of the transfer function were -0.9682 ± 1.7460i and -0.0318 ± 0.7460i. Since all poles possess negative real values, the system is stable. The presence of complex conjugates indicates the system is also oscillatory.
* **Routh-Hurwitz Criterion:** Routh array calculations identified that the proportional gain K yields a stable response strictly within the range -4.00 < K < 4.00.
* **Root Locus & Bode Plots:** MATLAB was used to generate Root Locus plots illustrating how closed-loop poles shift into unstable regions with increasing gain. Bode plots provided gain and phase margins to further reinforce system robustness.

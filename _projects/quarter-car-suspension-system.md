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
    description: "Free Body Diagram showing forces exerted by springs and dampers on sprung and unsprung masses[cite: 5]."
  - file: "/assets/images/projects/suspension/state-space-matrices.jpg"
    description: "State Space matrices (A, B, C, D) used to represent the system dynamics[cite: 5]."
  - file: "/assets/images/projects/suspension/simulink-diff-eq.jpg"
    description: "Simulink model constructed using the derived differential equations of motion[cite: 5]."
  - file: "/assets/images/projects/suspension/pid-controllers-design.jpg"
    description: "Two PID controllers designed in Simulink to control the output displacement[cite: 5]."
  - file: "/assets/images/projects/suspension/stateflow-decision-logic.jpg"
    description: "Stateflow chart used for dynamic switching between two PID controllers during runtime[cite: 5]."
  - file: "/assets/images/projects/suspension/simscape-model.jpg"
    description: "SimScape physical modeling environment for intuitive system simulation[cite: 7]."

components:
  - name: "MATLAB & Simulink"
    quantity: 1
    description: "Used for mathematical modeling, transfer function calculations, and simulating system responses[cite: 5, 7]."
  - name: "Simscape"
    quantity: 1
    description: "Provided a physical modeling environment to create realistic block representations of mechanical components[cite: 7]."

gallery:
  - type: "image"
    file: "/assets/images/projects/suspension/closed-loop-step-response.jpg"
    description: "Closed-loop step response comparing PID1 and PID2 performance[cite: 5, 6]."
  - type: "image"
    file: "/assets/images/projects/suspension/root-locus-plot.jpg"
    description: "Root locus plot indicating how closed-loop pole positions change as a function of varying gain[cite: 5, 6]."
  - type: "image"
    file: "/assets/images/projects/suspension/damping-comparison.jpg"
    description: "Simulink displacement outputs demonstrating a critically damped scenario[cite: 7]."
---

## Project Overview

This project simulates and analyzes the performance of a quarter-car suspension system to evaluate vehicle handling and passenger comfort[cite: 5, 7]. A quarter-car model is a simplified representation of a vehicle suspension system that considers only the vertical dynamics of a single wheel[cite: 5, 7]. This portfolio entry combines two distinct academic focuses: mechanical simulation (MTE294) and system dynamics and control (MTE394)[cite: 5, 6, 7].

**Team Members:** Yosuf Soliman, Abdala Ismail Hasen, Islam Hesham Allam, Abdelrahman Abdulsalam, and Islam Refaie[cite: 5, 6, 7].  
**Supervisors:** Faraz Tehranizadeh (MTE294)[cite: 7].  
**Institution:** Kadir Has University, Department of Mechatronics Engineering (Spring 2024)[cite: 7].

---

## System Modeling and Mathematical Representation

The physical system is modeled using two primary masses: a sprung mass representing the vehicle body ($m_1$) and an unsprung mass representing the wheel and suspension components ($m_2$)[cite: 7]. 

### Equations of Motion
The forces acting on the system include spring forces and damping forces[cite: 7]. Derived from free-body diagrams, the equations of motion for the system are as follows[cite: 5, 7]:
*   **Sprung Mass ($m_1$):** $m_{1}\ddot{q}_{1} = -k_{1}q_{1} + k_{2}(q_{2} - q_{1}) + b(\dot{q}_{2} - \dot{q}_{1})$[cite: 5].
*   **Unsprung Mass ($m_2$):** $m_{2}\ddot{q}_{2} = -k_{2}(q_{2} - q_{1}) - b(\dot{q}_{2} - \dot{q}_{1}) + u$[cite: 5].

### Simulation Parameters
The initial baseline simulation parameters were defined in MATLAB as follows[cite: 7]:
*   Vehicle Mass ($m_1$): $290$ kg[cite: 7].
*   Wheel Mass ($m_2$): $15$ kg[cite: 7].
*   Spring Stiffness ($k_1$, $k_2$): $16200$ N/m and $191000$ N/m[cite: 7].
*   Damping Coefficients ($c_1$, $c_2$): $1000$ Ns/m and $2500$ Ns/m[cite: 7].

### State-Space and Transfer Function
The system was defined using a State-Space representation formulated as $\dot{X} = AX(t) + Bu(t)$ and $Y = CX(t) + Du(t)$[cite: 5]. The state variables chosen were the displacements and velocities of both the sprung and unsprung masses[cite: 7]. Using these representations, the system's Transfer Function was calculated to evaluate the input-output relationship:
$$H = \frac{0.3333s + 0.5556}{s^4 + 2s^3 + 4.667s^2 + 1.333s + 2.222}$$[cite: 5, 6].

---

## System Simulation (MTE294)

Simulink was utilized to provide a graphical interface for simulating the quarter-car suspension system, utilizing block diagrams to represent system equations[cite: 7]. 

### Road Profile Scenarios
To evaluate ride comfort, a Signal Editor was used to provide two primary simulation scenarios[cite: 7]:
*   **Sidewalk:** Simulated a $10$ cm height step input for a duration of $4$ seconds[cite: 7].
*   **Speed Bump:** Simulated a $1$ meter bump for a duration of $3$ seconds[cite: 7].

### Damping Analysis
The system's behavior was observed by calculating the damping coefficient $c_1$ using the formula $\zeta = c_1 / (2 \sqrt{m_1 k_1})$[cite: 7]. The system's response was plotted under four distinct conditions[cite: 7]:
*   **Original Damping:** $c_1 = 1000$[cite: 7].
*   **Critical Damping:** Evaluated at $c_1 = 4335$, resulting in the system stopping oscillation in the shortest time possible[cite: 7].
*   **Under-Damped:** Evaluated at $c_1 = 2167.5$ ($4335 \times 0.5$)[cite: 7].
*   **Over-Damped:** Evaluated at $c_1 = 6502.5$ ($4335 \times 1.5$)[cite: 7].

Additionally, Simscape was incorporated to create a physical modeling environment, providing realistic block representations of components like axles and vehicle bodies directly linked to a physical solver[cite: 7].

---

## System Control and Stability (MTE394)

### PID Controller Design
To manage the output displacement $x_1(t)$ in response to an input force, two separate PID controllers were designed[cite: 5]. By modifying gains, the controllers were aimed at reducing steady-state error, improving transient response, and stabilizing the system[cite: 5]. 
*   **PID1:** $K_p = 3.5$, $K_i = 2.3$, $K_d = 3.5$[cite: 5, 6].
*   **PID2:** $K_p = 10$, $K_i = 10$, $K_d = 10$[cite: 5, 6].

Implementing closed-loop control with these PID controllers yielded shorter rise times, less overshoot, quicker settling, and reduced steady-state error compared to open-loop performance[cite: 5].

### Dynamic Controller Switching (Stateflow)
Decision logic was established using a Stateflow chart in Simulink to enable dynamic controller switching during simulations[cite: 5]. This allowed real-time transitions between PID1 and PID2 based on an external input signal, ensuring only one controller configuration was active at any given moment[cite: 5].

### Stability Analysis
Analytical and simulation-based methods were utilized to verify stability[cite: 5].
*   **Poles and Eigenvalues:** The calculated poles of the transfer function were $-0.9682 \pm 1.7460i$ and $-0.0318 \pm 0.7460i$[cite: 5, 6]. Since all poles possess negative real values, the system is stable[cite: 5]. The presence of complex conjugates indicates the system is also oscillatory[cite: 5].
*   **Routh-Hurwitz Criterion:** Routh array calculations identified that the proportional gain $K$ yields a stable response strictly within the range $-4.00 < K < 4.00$[cite: 5, 6].
*   **Root Locus & Bode Plots:** MATLAB was used to generate Root Locus plots illustrating how closed-loop poles shift into unstable regions with increasing gain[cite: 5]. Bode plots provided gain and phase margins to further reinforce system robustness[cite: 5].
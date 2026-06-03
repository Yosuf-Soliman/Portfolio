---
layout: project
title: "SPARK: Variable-Height Wheeled Bipedal Robot"
description: "A 5-DOF variable-height wheeled bipedal robot (VH-WIP) that bridges the agility-endurance gap between legged and wheeled UGVs, designed via a strict Model-Based Design approach using MATLAB/Simscape, custom PCB, FEA, and quasi-direct-drive actuation."
date: 2026-05-01
categories: [Robotics, Control Systems, Mechatronics, FEA, Embedded Systems, 3D Printing]
featured_image: "/assets/images/projects/spark/featured.jpeg"
github_url: "https://github.com/MESGRO/spark-vhwip"
demo_url: "#"
interactive_plot: false

models:
  - file: "/assets/models/spark/v2-chassis.gltf"
    description: "SPARK V2 optimized chassis — PAHT-CF and CFRP tubing design"
  - file: "/assets/models/spark/full-assembly.gltf"
    description: "Full exploded assembly view with component placement"

schematics:
  - file: "/assets/images/projects/spark/wiring-diagram.jpg"
    description: "Full component wiring diagram"
  - file: "/assets/images/projects/spark/pcb-3d.jpg"
    description: "Custom PCB 3D layout"
  - file: "/assets/images/projects/spark/pcb-physical.jpg"
    description: "DIY PCB"

code_files:
  - name: "Mass-Bounded Inverse Kinematics"
    file: "spark_ik_solver.py"
    language: "python"
    download_url: "https://github.com/MESGRO/spark-vhwip/blob/main/src/spark_ik_solver.py"
    content: |
      import numpy as np
      import math

      class SPARKInverseKinematics:
          """
          Mass-bounded Inverse Kinematics solver for SPARK VH-WIP robot.
          Solves for hip and knee joint angles that maintain CoM equilibrium
          (XCoM = 0) at a commanded torso height Z_target.

          Link geometry follows the notation in the SPARK technical report:
            L1 = shin link length (mm)
            L2 = hip link length  (mm)
            d1, d2 = fractional CoM positions along each link (0–1)
          """

          def __init__(self):
              # ── Geometric parameters (mm) ──────────────────────────────────
              self.L1 = 200.0          # Shin link length
              self.L2 = 200.0          # Hip link length
              self.R_wheel = 75.0      # Wheel radius

              # ── Mass parameters (grams) ────────────────────────────────────
              self.mL1 = 215.0         # Shin link mass
              self.mL2 = 215.0         # Hip link mass
              self.M0  = 395.0 + 360.0 / 2   # Wheel actuator + half wheel mass at P0
              self.M1  = 395.0         # Knee joint actuator mass at P1
              self.M2  = 395.0         # Hip joint actuator mass at P2
              self.M_total = 8420.0    # Total robot mass (measured: 8.42 kg)

              # ── CoM fractional positions along links ───────────────────────
              self.d1 = 0.5            # Shin CoM at link mid-point
              self.d2 = 0.5            # Hip CoM at link mid-point

              # ── Pre-compute kinetic weight factors (eq. 3.4 – 3.7) ─────────
              self.W1 = (self.mL1 * self.d1 + self.M1 + self.mL2 + self.M2) * self.L1
              self.W2 = (self.mL2 * self.d2 + self.M2) * self.L2
              self.K  = self.W1 / self.W2   # Kinetic ratio

          # ──────────────────────────────────────────────────────────────────
          def solve(self, z_target):
              """
              Solve mass-bounded IK for a commanded torso height z_target (mm).

              Returns dict with keys:
                alpha  – absolute shin angle from X-axis (rad)
                beta   – relative hip-to-shin angle (rad)
                theta  – hip actuator angle w.r.t. torso plane (rad)
                y_com  – global CoM height above ground (mm)
              Returns None if z_target is outside the feasible workspace.
              """
              K, L1, L2 = self.K, self.L1, self.L2

              # Quadratic coefficients (eq. 3.12 – 3.14)
              A =  L1**2 - L2**2 * K**2
              B = -2.0 * z_target * L1
              C =  z_target**2 - L2**2 + L2**2 * K**2

              discriminant = B**2 - 4.0 * A * C
              if discriminant < 0:
                  return None   # No real solution — target out of workspace

              s_candidates = [
                  (-B + math.sqrt(discriminant)) / (2.0 * A),
                  (-B - math.sqrt(discriminant)) / (2.0 * A),
              ]

              # Pick the elbow-up root: 0 ≤ s ≤ 1
              s = None
              for candidate in s_candidates:
                  if 0.0 <= candidate <= 1.0:
                      s = candidate
                      break

              if s is None:
                  return None   # Both roots outside physical range

              # Recover joint angles (eq. 3.15 – 3.17)
              alpha = math.asin(s)
              beta  = math.acos(-K * math.cos(alpha)) - alpha
              theta = alpha + beta - math.pi / 2.0

              # Global CoM height above ground (eq. 3.19)
              y_com = (self.R_wheel +
                       (self.W1 * math.sin(alpha) + self.W2 * math.sin(alpha + beta))
                       / self.M_total)

              return {
                  "alpha":  alpha,
                  "beta":   beta,
                  "theta":  theta,
                  "y_com":  y_com,
                  "alpha_deg": math.degrees(alpha),
                  "beta_deg":  math.degrees(beta),
                  "theta_deg": math.degrees(theta),
              }

          # ──────────────────────────────────────────────────────────────────
          def height_range(self, z_min=50, z_max=450, steps=200):
              """
              Sweep the feasible height range and return valid (z, alpha, theta) tuples.
              Useful for workspace visualization and pre-computing lookup tables.
              """
              results = []
              for z in np.linspace(z_min, z_max, steps):
                  sol = self.solve(z)
                  if sol:
                      results.append((z, sol["alpha_deg"], sol["beta_deg"],
                                      sol["theta_deg"], sol["y_com"]))
              return results

          # ──────────────────────────────────────────────────────────────────
          def check_joint_limits(self, solution):
              """
              Validate a solution dict against physical joint travel limits.
              Returns (bool, message).
              """
              if solution is None:
                  return False, "No solution provided."

              limits = {
                  "alpha": (-10, 85),    # Shin angle range (degrees)
                  "beta":  (-160, -20),  # Relative hip–shin angle (degrees)
                  "theta": (-90,  20),   # Hip actuator angle w.r.t. torso (degrees)
              }

              for key, (lo, hi) in limits.items():
                  val = math.degrees(solution[key])
                  if not (lo <= val <= hi):
                      return False, f"{key} = {val:.1f}° out of range [{lo}, {hi}]°"

              return True, "All joints within limits."


      # ── Example usage ─────────────────────────────────────────────────────
      if __name__ == "__main__":
          ik = SPARKInverseKinematics()

          print("=== SPARK IK Solver ===\n")
          for z in [150, 250, 300, 350]:
              sol = ik.solve(z)
              ok, msg = ik.check_joint_limits(sol)
              if sol:
                  print(f"Z = {z:>4} mm  |  α={sol['alpha_deg']:+6.1f}°  "
                        f"β={sol['beta_deg']:+6.1f}°  θ={sol['theta_deg']:+6.1f}°  "
                        f"CoM_h={sol['y_com']:.1f} mm  |  {msg}")
              else:
                  print(f"Z = {z:>4} mm  |  No feasible solution.")

          print("\n=== Feasible workspace sweep ===")
          workspace = ik.height_range()
          print(f"Valid height range: {workspace[0][0]:.0f} mm  →  {workspace[-1][0]:.0f} mm")
          print(f"Total valid configurations found: {len(workspace)}")

  - name: "Minimum-Snap Trajectory Generator"
    file: "spark_trajectory.py"
    language: "python"
    download_url: "https://github.com/MESGRO/spark-vhwip/blob/main/src/spark_trajectory.py"
    content: |
      import numpy as np

      class MinimumSnapTrajectory:
          """
          7th-order polynomial minimum-snap trajectory generator for SPARK's
          variable-height joint control. Converts a step height command into a
          smooth S-curve profile that prevents actuator wind-up and destabilising
          jerks during height transitions.

          The trajectory satisfies zero velocity, acceleration, jerk, and snap
          boundary conditions at both endpoints.
          """

          def __init__(self, dt=0.01):
              self.dt = dt    # Control loop timestep (s)

          def generate(self, z_start, z_end, duration):
              """
              Generate a minimum-snap height profile from z_start to z_end
              over `duration` seconds.

              Returns:
                t  – time vector (s)
                z  – height profile (mm)
                dz – velocity profile (mm/s)
              """
              t = np.arange(0, duration + self.dt, self.dt)
              tau = t / duration          # Normalised time ∈ [0, 1]

              # 7th-order minimum-snap polynomial
              # Coefficients derived from zero-snap boundary conditions
              poly = (252 * tau**5
                      - 1050 * tau**6
                      + 1800 * tau**7
                      - 1575 * tau**8
                      + 700  * tau**9
                      - 126  * tau**10)

              z  = z_start + (z_end - z_start) * poly
              dz = (z_end - z_start) / duration * np.gradient(poly, tau)

              return t, z, dz

          def build_profile(self, height_sequence, durations):
              """
              Chain multiple height transitions into a single continuous profile.

              Args:
                height_sequence – list of heights, e.g. [150, 300, 300, 200]
                durations        – list of transition durations (s), len = len(heights)-1

              Returns concatenated (t, z, dz) arrays.
              """
              t_all, z_all, dz_all = [], [], []
              t_offset = 0.0

              for i in range(len(height_sequence) - 1):
                  t_seg, z_seg, dz_seg = self.generate(
                      height_sequence[i], height_sequence[i + 1], durations[i]
                  )
                  t_all.append(t_seg + t_offset)
                  z_all.append(z_seg)
                  dz_all.append(dz_seg)
                  t_offset += durations[i]

              return (np.concatenate(t_all),
                      np.concatenate(z_all),
                      np.concatenate(dz_all))


      # ── Example ───────────────────────────────────────────────────────────
      if __name__ == "__main__":
          traj = MinimumSnapTrajectory(dt=0.005)

          heights   = [150, 300, 300, 200]    # mm
          durations = [1.5,  2.0,  1.5]       # s

          t, z, dz = traj.build_profile(heights, durations)

          print(f"Profile duration : {t[-1]:.2f} s")
          print(f"Peak velocity    : {max(abs(dz)):.1f} mm/s")
          print(f"Height range     : {min(z):.0f} – {max(z):.0f} mm")

  - name: "LQR Balance Controller"
    file: "spark_lqr.py"
    language: "python"
    download_url: "https://github.com/MESGRO/spark-vhwip/blob/main/src/spark_lqr.py"
    content: |
      import numpy as np

      class SPARKBalanceController:
          """
          Discrete-time LQR balance controller for SPARK.
          State vector x = [θ_robot, θ̇_robot, θ_motor, θ̇_motor]
            θ_robot  – robot pitch angle (rad)
            θ̇_robot  – robot pitch rate  (rad/s)
            θ_motor  – wheel motor angular position (rad)
            θ̇_motor  – wheel motor angular velocity (rad/s)

          Gains derived via Bryson's Rule with the following maximum deviations:
            θ_robot  = ±1.0°  → ±0.01745 rad
            θ̇_robot  = ±5.0°/s
            θ_motor  = ±0.5 rad
            θ̇_motor  = ±10 rad/s
            u_max    = 8.0 Nm (GIM6010-8 peak torque)

          Resulting gain matrix K = [75, 45, 2, 4]
          """

          def __init__(self):
              self.K = np.array([75.0, 45.0, 2.0, 4.0])   # LQR gain vector

              # Safety limits
              self.PITCH_CUTOFF  = np.radians(35.0)   # Global motor cut-off
              self.TORQUE_LIMIT  = 8.0                 # Nm (actuator saturation)

              self._armed = True    # False after safety cutoff

          # ──────────────────────────────────────────────────────────────────
          def compute(self, theta, theta_dot, motor_pos, motor_vel):
              """
              Compute wheel torque command u (Nm) for the given state.
              Returns (u, safe) where safe=False indicates a fault condition.
              """
              if not self._armed:
                  return 0.0, False

              # Safety: pitch limit check (eq. from slide 17)
              if abs(theta) > self.PITCH_CUTOFF:
                  self._armed = False
                  return 0.0, False

              x = np.array([theta, theta_dot, motor_pos, motor_vel])
              u = float(-self.K @ x)

              # Clamp to actuator limits
              u = np.clip(u, -self.TORQUE_LIMIT, self.TORQUE_LIMIT)
              return u, True

          def reset(self):
              """Re-arm the controller after a safe recovery."""
              self._armed = True

          # ──────────────────────────────────────────────────────────────────
          def simulate_step_response(self, disturbance_nm=5.0, duration=3.0, dt=0.01):
              """
              Simulate the closed-loop step disturbance response.
              Applies disturbance_nm at t=0 and returns time and pitch arrays.

              Uses a simplified 1-DOF linearised pitch model for illustration.
              Full accuracy requires the Simscape multibody plant.
              """
              # Simplified linearised plant parameters (illustrative)
              I_eff   = 0.35   # Effective pitch moment of inertia (kg·m²)
              b_wheel = 0.05   # Viscous damping coefficient

              steps = int(duration / dt)
              t   = np.zeros(steps)
              th  = np.zeros(steps)
              thd = np.zeros(steps)
              mp  = np.zeros(steps)
              mv  = np.zeros(steps)

              thd[0] = disturbance_nm / I_eff * dt   # Initial velocity from impulse

              for i in range(1, steps):
                  u, safe = self.compute(th[i-1], thd[i-1], mp[i-1], mv[i-1])
                  if not safe:
                      break

                  # Euler integration of linearised pitch dynamics
                  alpha = (disturbance_nm * (i == 1) - u - b_wheel * thd[i-1]) / I_eff
                  thd[i] = thd[i-1] + alpha * dt
                  th[i]  = th[i-1]  + thd[i-1] * dt
                  mp[i]  = mp[i-1]  + mv[i-1]  * dt
                  mv[i]  = mv[i-1]  + u / I_eff * dt
                  t[i]   = i * dt

              return t, np.degrees(th)


      # ── Example ───────────────────────────────────────────────────────────
      if __name__ == "__main__":
          ctrl = SPARKBalanceController()

          t, pitch = ctrl.simulate_step_response(disturbance_nm=5.0, duration=3.0)
          peak_idx = int(abs(pitch).argmax())

          print(f"Peak pitch deviation : {pitch[peak_idx]:.2f}°  at t={t[peak_idx]:.2f} s")

          # Find recovery time (|pitch| < 1.0°)
          recovered = next((t[i] for i in range(peak_idx, len(t)) if abs(pitch[i]) < 1.0), None)
          if recovered:
              print(f"Recovery time (<1°) : {recovered:.2f} s  (target: < 1.5 s)")
          else:
              print("Did not recover within simulation window.")

components:
  - name: "GIM6010-8 Quasi-Direct Drive Actuators"
    quantity: 6
    description: "Electric actuators with 8:1 reduction gearbox, integrated encoder, and FOC motor driver (GDS68). Used for hip joints, knee joints, and wheel drive."
    link: "https://www.gyems.cn/"

  - name: "ESP32-WROOM-U"
    quantity: 1
    description: "Low-level MCU for real-time balance control, teleoperation, CAN bus communication, and IMU data acquisition."

  - name: "Raspberry Pi 5"
    quantity: 1
    description: "High-level SBC for navigation, LIDAR-based SLAM, depth camera processing, and future AI integration."

  - name: "10-DOF IMU"
    quantity: 1
    description: "Inertial sensor measuring system angular states (pitch, roll, yaw + accelerations) fed into the LQR controller."

  - name: "Custom PCB"
    quantity: 1
    description: "Consolidates ESP32, SN65HVD230 CAN transceiver, and IMU. Segregates 22.2V actuator rails from 5V/3.3V logic. Designed in KiCad."

  - name: "2D LIDAR Sensor"
    quantity: 1
    description: "Planar SLAM sensor for future autonomous mapping and navigation."

  - name: "Intel RealSense D435iF"
    quantity: 1
    description: "Depth camera generating 3D point cloud maps for environmental perception and obstacle detection."

  - name: "6S LiPo Battery (22.2V, 8Ah)"
    quantity: 2
    description: "Primary actuator power rails. Provides 45+ minutes of runtime under operational load."

  - name: "2S LiPo Battery (7.4V, 4Ah)"
    quantity: 1
    description: "Logic power rail for compute and sensor subsystems."

  - name: "Tension Springs (0.45 N/mm)"
    quantity: 2
    description: "87.7mm knee-joint tension springs acting as passive elastic elements to augment jump torque output."

  - name: "FDM 3D Printed Chassis (V1 — PLA/TPU)"
    quantity: 1
    description: "Physical proof-of-concept chassis. Printed on Bambu Lab A1. TPU used for tires and impact-absorbing pads."

  - name: "V2 Chassis Materials (PAHT-CF / CFRP)"
    quantity: 1
    description: "Optimized final design using carbon-fiber-reinforced Polyamide and CFRP tubing for maximum strength-to-weight ratio."

gallery:
  - type: "image"
    file: "/assets/images/projects/spark/featured.jpeg"
    description: "SPARK V1 physical prototype — full assembly"
  - type: "image"
    file: "/assets/images/projects/spark/v2-cad.jpg"
    description: "V2 optimized CAD model (PAHT-CF design)"
  - type: "image"
    file: "/assets/images/projects/spark/exploded-assembly.jpg"
    description: "Exploded assembly view showing internal component placement"
  - type: "image"
    file: "/assets/images/projects/spark/pcb-physical.jpg"
    description: "DIY PCB"
  - type: "image"
    file: "/assets/images/projects/spark/pcb-3d.jpg"
    description: "Custom PCB 3D render — ESP32, CAN transceiver, IMU consolidated. Ready for Manufacturing"
  - type: "image"
    file: "/assets/images/projects/spark/simscape-model.jpg"
    description: "Simscape multibody simulation model"
  - type: "image"
    file: "/assets/images/projects/spark/simscape-3d-env.jpg"
    description: "3D simulation environment with ground contact mechanics"
  - type: "image"
    file: "/assets/images/projects/spark/lqr-disturbance-inputs.jpg"
    description: "LQR disturbance step inputs (±5 Nm bilateral)"
  - type: "image"
    file: "/assets/images/projects/spark/lqr-angular-states.jpg"
    description: "System angular states during disturbance recovery"
  - type: "image"
    file: "/assets/images/projects/spark/lqr-wheel-states.jpg"
    description: "Wheel actuators angular states during disturbance recovery"
  - type: "image"
    file: "/assets/images/projects/spark/trajectory-subsystem.jpg"
    description: "Minimum-snap trajectory generation subsystem block diagram (Simscape)"
  - type: "image"
    file: "/assets/images/projects/spark/trajectory-generation-plot.jpg"
    description: "Minimum-snap trajectory generation S-curve output"
  - type: "image"
    file: "/assets/images/projects/spark/jump-vh-profile.jpg"
    description: "91.5 mm jump: VH profile and joint trajectory"
  - type: "image"
    file: "/assets/images/projects/spark/jump-ground-contact.jpg"
    description: "91.5 mm jump: ground contact data during takeoff and landing"
  - type: "image"
    file: "/assets/images/projects/spark/jump-imu-data.jpg"
    description: "91.5 mm jump: IMU pitch data during flight phase"
  - type: "image"
    file: "/assets/images/projects/spark/jump-actuator-torques.jpg"
    description: "91.5 mm jump: joint actuator torques during jump sequence"
  - type: "image"
    file: "/assets/images/projects/spark/fea-shin-mesh.jpg"
    description: "Altair HyperMesh — tetrahedral mesh and RBE connectors on shin link"
  - type: "image"
    file: "/assets/images/projects/spark/fea-hip-mesh.jpg"
    description: "Altair HyperMesh — tetrahedral mesh of hip link for OptiStruct analysis"
  - type: "image"
    file: "/assets/images/projects/spark/robot-stand.jpg"
    description: "Custom robot stand for repeatable IK initial-condition calibration"
  - type: "image"
    file: "/assets/images/projects/spark/wiring-diagram.jpg"
    description: "Full component wiring diagram — CAN bus, power rails, and signal lines"
  - type: "image"
    file: "/assets/images/projects/spark/physical-demo.jpg"
    description: "SPARK physical prototype during teleoperation and balance demo"
---

## Project Overview

SPARK (a Variable-Height Wheeled Bipedal Robot) is a 5-DOF hybrid mobile robot developed as a graduation engineering project at Kadir Has University (FENS 402, Spring 2026). The project closes the fundamental **agility-endurance trade-off** in ground mobile robots: wheeled UGVs are efficient but terrain-limited, while legged platforms are agile but energy-prohibitive. SPARK combines motorized wheels at the leg extremities with an active variable-height scissor-leg mechanism, delivering planar rolling efficiency and vertical agility in a single compact platform under 10 kg.

The entire project followed a **strict Model-Based Design (MBD) pipeline**: from CAD and FEA, through full-fidelity Simscape multibody simulation, to embedded controller implementation and physical testing — with no gap between the virtual and physical systems.

**Team:** Islam Refaie · Yosuf Soliman · Abdala Hasen  
**Supervisor:** Assoc. Prof. Ertuğrul Tolga Duran

---

## Key Features

### Mechanical Architecture
- **5 Degrees of Freedom**: 2 wheel actuators + 2 knee joints + 2 hip joints (1 nonholonomic constraint limits lateral translation)
- **Variable Height Range**: ≥ 200 mm active height control via scissor-leg linkage
- **V1 Chassis**: FDM-printed PLA/TPU — physical proof-of-concept, 8.42 kg total
- **V2 Chassis**: Designed in PAHT-CF and CFRP tubing for superior strength-to-weight ratio
- **Tension Springs**: Passive elastic elements at knee joints augment jump torque output

### Sensing & Compute
- **10-DOF IMU**: Real-time pitch/roll/yaw feedback into the LQR balance loop
- **CAN Bus (SN65HVD230)**: Differential signaling for noise-immune actuator communication in a high-current environment
- **Raspberry Pi 5**: High-level navigation SBC — LIDAR SLAM and depth camera integration
- **ESP32-WROOM-U**: Low-level real-time MCU — 100 Hz balance control and teleoperation
- **Custom PCB**: Consolidates ESP32, CAN transceiver, and IMU; segregated 22.2V / 5V / 3.3V power planes

### Control System
- **LQR Balance Controller**: Gain matrix [75, 45, 2, 4] derived via Bryson's Rule; ±5 Nm step disturbance recovery < 1.5 s
- **Mass-Bounded IK Solver**: Closed-form geometric solution ensuring CoM equilibrium (X_CoM = 0) across all height profiles
- **Minimum-Snap Trajectory Generator**: 7th-order polynomial S-curves prevent actuator wind-up during height transitions
- **Aerial Reorientation Controller (AROC)**: Uses wheels as inertial reaction devices via conservation of angular momentum (L = Iω) to orient the robot during ballistic jumps

### Simulation
- **Simscape Multibody**: Full 6-DOF plant with part-specific mass distributions, inertial tensors, ground-contact mechanics (tire elastic deformation), and joint friction
- **Jump Capability**: 91.5 mm simulated jump height with < 3 s post-landing recovery
- **50 Hz Telemetry**: Real-time data logging and plotting of all system and actuator states

---

## Technical Specifications

| Parameter | Value |
|-----------|-------|
| **Total Mass** | 8.42 kg (target: < 10 kg ✅) |
| **Degrees of Freedom** | 5 (controlled) + 1 nonholonomic |
| **Variable Height Range** | ≥ 200 mm ✅ |
| **Max Linear Velocity** | > 1 m/s ✅ |
| **Pitch Error (steady-state)** | ±1.0° off vertical ✅ |
| **Disturbance Recovery** | < 1.5 s from ±5 Nm step ✅ |
| **Runtime** | ≥ 45 minutes (dual 6S 8Ah LiPo) ✅ |
| **Payload Capability** | ≥ 3 kg (actuator rig verified) ✅ |
| **Jump Height (simulation)** | 91.5 mm (target: > 75 mm ✅) |
| **Actuators** | 6 × GIM6010-8 (8:1 QDD, FOC) |
| **Control Loop Rate** | 100 Hz (ESP32) |
| **Telemetry Rate** | 50 Hz |
| **Mass Distribution** | 67% Hardware / 33% Chassis |

---

## System Architecture

### Model-Based Design Pipeline
1. **Performance Definition** → Quantified success criteria table (15 criteria)
2. **Computer Aided Engineering** → SolidWorks CAD + iterative FEA (SolidWorks Simulation → Altair HyperMesh)
3. **Model-Based Design** → Simplified CAD imported into Simscape; controller design and validation
4. **Physical System** → Chassis fabrication, PCB manufacture, embedded programming, systems integration, field testing

### Hardware Stack
- **High-Level**: Raspberry Pi 5 → LIDAR + Intel RealSense D435iF → SLAM/Navigation
- **Low-Level**: ESP32 → Custom PCB → CAN Bus → 6 × GIM6010-8 actuators
- **Power**: 2 × 22.2V 8Ah LiPo (actuators) + 1 × 7.4V 4Ah LiPo (logic) + XL4016 buck converter

---

## Inverse Kinematics Solution

SPARK's IK is derived from a single constraint: **X_CoM = 0** (static equilibrium) at every commanded height. The derivation groups all link and actuator masses into two kinetic weight factors W₁ and W₂, forming a **kinetic ratio K = W₁/W₂**. This reduces the equilibrium condition to a standard quadratic equation solvable in closed form, yielding the shin angle α and relative hip angle β for any target torso height Z_target.

### Key Equations

| Step | Equation |
|------|----------|
| Kinetic weight factors | W₁ = (m_L1·d₁ + M₁ + m_L2 + M₂)·L₁ |
| | W₂ = (m_L2·d₂ + M₂)·L₂ |
| Kinetic ratio | K = W₁ / W₂ |
| Quadratic (solve for s = sin α+β) | As² + Bs + C = 0 |
| Shin angle | α = arcsin(s) |
| Hip relative angle | β = arccos(−K·cos α) − α |
| Hip actuator angle | θ = α + β − π/2 |
| Global CoM height | Y_CoM = R_wheel + (W₁·sin α + W₂·sin(α+β)) / M_total |

---

## Control Architecture

### LQR Balance Controller
- **State vector**: [θ_robot, θ̇_robot, θ_motor, θ̇_motor]
- **Gain tuning**: Bryson's Rule — normalizes Q/R matrices by maximum acceptable deviations and actuator torque saturation (8 Nm peak)
- **Result**: Gain matrix K = [75, 45, 2, 4]; critically damped, < 1.5 s recovery from ±5 Nm bilateral step disturbances

### Aerial Reorientation Controller (AROC)
- **Trigger**: Ground–wheel contact Boolean; switches wheel actuators from LQR → AROC on liftoff
- **Principle**: Conservation of angular momentum (L = Iω = const); wheels act as inertial reaction wheels
- **Stage 1**: Drives robot pitch back toward equilibrium from uncontrolled rotation
- **Stage 2**: Activates when |γ| ≤ 1°; decelerates wheels to prevent overshoot and ensures controlled touchdown velocity

### Safety Systems
- **Pitch Limit**: ±35° triggers immediate global motor cutoff (hardware + software)
- **Manual Override**: PS controller 'Share' button halts LQR while preserving joint posture
- **Thermal Cutoff**: Automatic shutdown at > 90°C per actuator
- **Over-Current**: Hardware motor current limit at 23A
- **Signal Loss**: CAN bus or power disconnect forces global fail-safe (all motors off)
- **Mechanical**: 70A TPU chassis components absorb ground reaction shocks during falls

---

## FEA Methodology

FEA was performed in Altair HyperMesh on the primary structural members — the **hip and shin links** — as they transfer the highest dynamic loads during crouching and jumping.

- **Meshing**: Second-order solid tetrahedral elements (preserves actual load paths; avoids oversimplification of complex geometries)
- **Boundary conditions**: RBE2 spider (motor side, all 6 DOF constrained) + RBE3 distributed load spider (bearing contact side)
- **Load case**: Crouched posture at 48.39° — maximum expected bending; 26.98 N applied load per link
- **Result**: Pre-processing and mesh setup completed; structural results pending. Links were manufactured with conservative over-stiff geometry pending full solver output.

---

## Simulation Results

All simulation success criteria (1–8) were met or exceeded:

| Criterion | Target | Result |
|-----------|--------|--------|
| Pitch error (steady-state) | ±1.5° | ±1.0° ✅ |
| Step disturbance recovery | < 1.5 s | < 1.5 s ✅ |
| VH range | ≥ 200 mm | ≥ 200 mm ✅ |
| Min-snap trajectory | Implemented | ✅ |
| Max linear velocity | > 1 m/s | > 1 m/s ✅ |
| Jump height | > 75 mm | 91.5 mm ✅ |
| Post-landing recovery | < 3 s | < 3 s ✅ |
| 6-DOF multibody sim | Required | ✅ |

---

## Physical Validation Results

| Metric | Target | Achieved |
|--------|--------|----------|
| Total mass | < 10 kg | 8.42 kg ✅ |
| Runtime | ≥ 45 min | ≥ 45 min ✅ |
| VH range | ≥ 200 mm | ≥ 200 mm ✅ |
| Max velocity | > 1 m/s | > 1 m/s ✅ |
| Payload | ≥ 3 kg | ≥ 3 kg ✅ |
| Unassisted balance | Required | ✅ |
| Teleoperation | Required | ✅ |

---

## Applications

### Defense / MOUT Operations
Long-duration perimeter patrols over mixed terrain; maintains stable sensor height profile during locomotion.

### Last-Mile Delivery
Curb-climbing and ditch-traversal capability combined with energy-efficient wheeled rolling on paved surfaces.

### Educational Platform
Hands-on demonstration of control theory, embedded systems, mechanical design, FEA, simulation, and manufacturing — shown to reduce course failure rates by up to 33% versus lecture-only instruction.

### Future Research Base
Scalable foundation for SLAM, computer vision, terrain adaptation, and embodied AI research (LIDAR + Intel RealSense D435iF already integrated).

---

## Future Work

### Immediate
- Full FEA solver execution and structural result validation on hip/shin links
- V2 chassis manufacturing (PAHT-CF + CFRP tubing)
- Sim-to-real gap analysis and physical controller re-tuning

### Navigation & Autonomy
- Full Raspberry Pi 5 integration with LIDAR and depth camera
- Real-time SLAM and path-planning implementation
- Perception-less terrain adaptation (ZMP + floating-base dynamics)

### Advanced Capabilities
- Jumping over obstacles > 200 mm using torsional spring augmentation
- Multi-robot coordination
- Embodied AI and natural language command interface

---

## Lessons Learned

1. **Hardware availability loops back into design**: Regional shipping constraints required iterative adjustment of success criteria — performance definition is never purely linear.
2. **MBD pays off**: Direct CAD → Simscape import with preserved inertial tensors meant controller gains translated to hardware with minimal re-tuning.
3. **CAN over UART**: High-current motor proximity made CAN's differential signaling essential for reliable communication.
4. **IMU data truncation > aggressive filtering**: Rounding to nearest 0.1° eliminates micro-vibrations without introducing the 15–30 ms phase lag that aggressive filters cause.
5. **Quasi-direct drive is worth the cost**: Low reflected inertia and back-drivability allowed the links to act as passive spring-dampers during landing — a critical safety mechanism.

---
layout: project
title: "AeroTrack: Autonomous FPV Drone Tracking System"
description: "A fully autonomous GPS-denied FPV drone that tracks a moving target through 11 predefined flight maneuvers using YOLOv11n, a 6-state Kalman filter, a 4-state FSM, and a ROS 2 / MAVROS / MAVLink GCS pipeline. Developed for TEKNOFEST 2026."
date: 2026-07-01
categories: [Robotics, Autonomous Systems, ROS 2, Computer Vision, MAVLink, Drones]
featured_image: "/assets/images/projects/aerotrack/featured.jpeg"
github_url: "https://github.com/Yosuf-Soliman/Portfolio"
demo_url: "#"
interactive_plot: false

schematics:
  - file: "/assets/images/projects/aerotrack/system-architecture-block.jpeg"
    description: "Full system architecture block diagram — battery, ESC, FC, VTX, telemetry, GCS laptop"
  - file: "/assets/images/projects/aerotrack/ros2-node-graph.jpeg"
    description: "ROS 2 node graph — vision node, FSM node, flight control C++ node, MAVROS bridge"
  - file: "/assets/images/projects/aerotrack/software-dataflow.jpeg"
    description: "Software data flow diagram — Video Thread → Kalman → FSM → MAVLink → ArduPilot"

code_files:
  - name: "ROS 2 MAVLink Bridge (C++ Flight Control Node)"
    file: "flight_control_node.cpp"
    language: "cpp"
    download_url: "https://github.com/Yosuf-Soliman/Portfolio/blob/main/src/aerotrack/flight_control_node.cpp"
    content: |
      /**
       * flight_control_node.cpp
       * AeroTrack — ROS 2 C++ Flight Control Node
       *
       * Responsibilities:
       *   - Subscribes to /aerotrack/guidance (geometry_msgs/Twist) from the Python vision node
       *   - Manages GUIDED_NOGPS ↔ ALT_HOLD mode transitions via MAVROS with debounce
       *   - Converts IBVS+PNG velocity guidance into SET_ATTITUDE_TARGET quaternion + thrust
       *   - Owns all FSM state transitions and emergency failsafe authority
       *   - Hardware-interrupt-based failsafe path (avoids EKF position-variance descents)
       *
       * MAVLink interface: MAVROS /mavros/setpoint_raw/attitude
       * Failsafe:          MAVROS /mavros/set_mode  →  ALT_HOLD
       */

      #include <rclcpp/rclcpp.hpp>
      #include <geometry_msgs/msg/twist.hpp>
      #include <mavros_msgs/msg/attitude_target.hpp>
      #include <mavros_msgs/srv/set_mode.hpp>
      #include <mavros_msgs/srv/command_bool.hpp>
      #include <std_msgs/msg/string.hpp>
      #include <tf2/LinearMath/Quaternion.h>
      #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
      #include <chrono>
      #include <cmath>

      using namespace std::chrono_literals;

      // ── FSM States ──────────────────────────────────────────────────────────────
      enum class FlightState { INIT, TRACKING, COASTING, SWEEP, FAILSAFE };

      static const char* state_name(FlightState s) {
          switch (s) {
              case FlightState::INIT:     return "INIT";
              case FlightState::TRACKING: return "TRACKING";
              case FlightState::COASTING: return "COASTING";
              case FlightState::SWEEP:    return "SWEEP";
              case FlightState::FAILSAFE: return "FAILSAFE";
          }
          return "UNKNOWN";
      }

      // ── Parameters ───────────────────────────────────────────────────────────────
      constexpr double LOST_TO_SWEEP_S      = 2.5;   // COASTING → SWEEP after 2.5 s
      constexpr double DEBOUNCE_MODE_S      = 0.5;   // minimum time between mode changes
      constexpr double MAX_THRUST           = 0.65;  // normalised [0,1] — maps to ~7:1 TWR
      constexpr double HOVER_THRUST         = 0.40;
      constexpr double SWEEP_FORWARD_VEL    = 30.0;  // m/s sprint along last-known heading
      constexpr double ALTITUDE_FLOOR_M     = 15.0;  // anti-crash altitude floor (AGL)

      class FlightControlNode : public rclcpp::Node {
      public:
          FlightControlNode() : Node("flight_control_node"),
              state_(FlightState::INIT),
              time_lost_(0.0),
              last_mode_change_(this->now()),
              autonomous_active_(false)
          {
              // ── Publishers ────────────────────────────────────────────────────
              attitude_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
                  "/mavros/setpoint_raw/attitude", 10);

              fsm_state_pub_ = this->create_publisher<std_msgs::msg::String>(
                  "/aerotrack/fsm_state", 10);

              // ── Subscribers ───────────────────────────────────────────────────
              // Receives IBVS+PNG body-frame velocity guidance from Python vision node
              guidance_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                  "/aerotrack/guidance", 10,
                  std::bind(&FlightControlNode::guidance_callback, this, std::placeholders::_1));

              // ── Service clients ───────────────────────────────────────────────
              set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
                  "/mavros/set_mode");
              arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
                  "/mavros/cmd/arming");

              // ── Control timer: 50 Hz ──────────────────────────────────────────
              control_timer_ = this->create_wall_timer(
                  20ms, std::bind(&FlightControlNode::control_loop, this));

              RCLCPP_INFO(this->get_logger(), "FlightControlNode initialised — state: INIT");
          }

      private:
          // ── Latest guidance from vision node ──────────────────────────────────
          geometry_msgs::msg::Twist latest_guidance_;
          bool guidance_received_ = false;

          void guidance_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
              latest_guidance_ = *msg;
              guidance_received_ = true;
          }

          // ── 50 Hz control loop ────────────────────────────────────────────────
          void control_loop() {
              update_fsm();
              publish_attitude_target();
              publish_fsm_state();
          }

          // ── FSM transition logic ──────────────────────────────────────────────
          void update_fsm() {
              // Guidance linear.z carries EMA confidence from vision node
              double ema_conf  = latest_guidance_.linear.z;
              bool   locked    = (ema_conf > 0.10);
              auto   now       = this->now();

              switch (state_) {
                  case FlightState::INIT:
                      if (locked) {
                          transition_to(FlightState::TRACKING, now);
                          request_guided_nogps();
                      }
                      break;

                  case FlightState::TRACKING:
                      if (!locked) {
                          time_lost_ = 0.0;
                          transition_to(FlightState::COASTING, now);
                      }
                      break;

                  case FlightState::COASTING:
                      if (locked) {
                          transition_to(FlightState::TRACKING, now);
                      } else {
                          time_lost_ += 0.020;  // 20 ms per tick
                          if (time_lost_ > LOST_TO_SWEEP_S)
                              transition_to(FlightState::SWEEP, now);
                      }
                      break;

                  case FlightState::SWEEP:
                      if (locked)
                          transition_to(FlightState::TRACKING, now);
                      break;

                  case FlightState::FAILSAFE:
                      // Pilot has authority — do not auto-recover
                      break;
              }
          }

          void transition_to(FlightState next, const rclcpp::Time& now) {
              RCLCPP_INFO(this->get_logger(), "FSM: %s → %s",
                          state_name(state_), state_name(next));
              state_ = next;
              (void)now;
          }

          // ── GUIDED_NOGPS mode request with debounce ───────────────────────────
          void request_guided_nogps() {
              auto now = this->now();
              if ((now - last_mode_change_).seconds() < DEBOUNCE_MODE_S) return;

              auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
              req->custom_mode = "GUIDED_NOGPS";

              set_mode_client_->async_send_request(req,
                  [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                      if (future.get()->mode_sent)
                          RCLCPP_INFO(this->get_logger(), "Mode → GUIDED_NOGPS");
                      else
                          RCLCPP_WARN(this->get_logger(), "Mode change FAILED");
                  });

              last_mode_change_ = now;
              autonomous_active_ = true;
          }

          // ── Emergency failsafe: switch to ALT_HOLD ───────────────────────────
          void trigger_failsafe() {
              state_ = FlightState::FAILSAFE;
              autonomous_active_ = false;

              auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
              req->custom_mode = "ALT_HOLD";

              set_mode_client_->async_send_request(req,
                  [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                      if (future.get()->mode_sent)
                          RCLCPP_WARN(this->get_logger(), "FAILSAFE → ALT_HOLD");
                  });
          }

          // ── Convert body-frame velocity guidance → SET_ATTITUDE_TARGET ────────
          //
          // Vision node publishes: Twist.linear  = [v_fwd, v_lat, v_z]
          //                        Twist.angular  = [0,     0,     yaw_rate]
          //                        Twist.linear.z = EMA confidence (overloaded field)
          //
          // We map v_fwd → pitch, v_lat → roll, yaw_rate → yaw_rate,
          // and hold hover thrust unless in SWEEP.
          void publish_attitude_target() {
              if (!autonomous_active_) return;

              mavros_msgs::msg::AttitudeTarget cmd;
              cmd.header.stamp = this->now();

              // Ignore attitude fields; use body rates + thrust
              cmd.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;

              double v_fwd     = latest_guidance_.linear.x;
              double v_lat     = latest_guidance_.linear.y;
              double yaw_rate  = latest_guidance_.angular.z;

              // SWEEP: sprint at 30 m/s along last-known heading
              if (state_ == FlightState::SWEEP) {
                  v_fwd    = SWEEP_FORWARD_VEL;
                  v_lat    = 0.0;
                  yaw_rate = 0.0;
              }

              // Scale velocity → body rates (tuned for 7:1 TWR airframe)
              const double VEL_TO_PITCH = -0.018;   // rad/(m/s) — nose-down to go forward
              const double VEL_TO_ROLL  =  0.015;   // rad/(m/s) — roll right to strafe right

              cmd.body_rate.x = v_lat    * VEL_TO_ROLL;
              cmd.body_rate.y = v_fwd    * VEL_TO_PITCH;
              cmd.body_rate.z = yaw_rate;

              // Thrust: boost during SWEEP, hover otherwise
              cmd.thrust = (state_ == FlightState::SWEEP) ? MAX_THRUST : HOVER_THRUST;

              attitude_pub_->publish(cmd);
          }

          // ── Publish FSM state string for telemetry logging ───────────────────
          void publish_fsm_state() {
              std_msgs::msg::String msg;
              msg.data = state_name(state_);
              fsm_state_pub_->publish(msg);
          }

          // ── Members ───────────────────────────────────────────────────────────
          rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;
          rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            fsm_state_pub_;
          rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     guidance_sub_;
          rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr           set_mode_client_;
          rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr       arming_client_;
          rclcpp::TimerBase::SharedPtr                                   control_timer_;

          FlightState      state_;
          double           time_lost_;
          rclcpp::Time     last_mode_change_;
          bool             autonomous_active_;
      };

      int main(int argc, char* argv[]) {
          rclcpp::init(argc, argv);
          rclcpp::spin(std::make_shared<FlightControlNode>());
          rclcpp::shutdown();
          return 0;
      }

  - name: "GCS Vision Node (Python — ROS 2 Publisher)"
    file: "vision_node.py"
    language: "python"
    download_url: "https://github.com/Yosuf-Soliman/Portfolio/blob/main/src/aerotrack/vision_node.py"
    content: |
      """
      vision_node.py
      AeroTrack — ROS 2 Python Vision Node

      Responsibilities:
        - Captures 1080p FPV video from Walksnail VRX via HDMI capture card (OpenCV)
        - Runs YOLOv11n GPU inference (640×640, batch=1) at ~30 FPS
        - Falls back to HSV dual-range red segmentation when YOLO confidence is low
        - Maintains KalmanTracker6 (6-state linear Kalman filter) for smooth state estimation
        - Applies 80 ms look-ahead latency compensation and 300 ms prediction cap
        - Computes IBVS + PNG guidance velocity commands
        - Publishes geometry_msgs/Twist to /aerotrack/guidance consumed by C++ flight node

      ROS 2 interface:
        Publishes: /aerotrack/guidance  (geometry_msgs/Twist)
                   /aerotrack/debug_frame (sensor_msgs/Image)  [optional]
      """

      import rclpy
      from rclpy.node import Node
      from geometry_msgs.msg import Twist
      from sensor_msgs.msg import Image
      from cv_bridge import CvBridge

      import cv2
      import numpy as np
      import threading
      import time
      from ultralytics import YOLO

      # ── Kalman Filter ────────────────────────────────────────────────────────────
      class KalmanTracker6:
          """
          6-state linear Kalman filter.
          State vector: [cx, cx_dot, cy_err, cy_err_dot, w, w_dot]
            cx      — horizontal normalised image position  [-1, +1]
            cy_err  — vertical error from tilt-compensated setpoint [-1, +1]
            w       — normalised bounding box width [0, 1]  (range proxy)
          """

          DT = 1.0 / 30.0   # 30 Hz video thread

          def __init__(self):
              n = 6
              self.x = np.zeros((n, 1))           # state vector
              self.P = np.eye(n) * 0.5            # covariance

              # State transition — constant velocity model
              dt = self.DT
              self.F = np.array([
                  [1, dt, 0,  0,  0,  0],
                  [0,  1, 0,  0,  0,  0],
                  [0,  0, 1, dt,  0,  0],
                  [0,  0, 0,  1,  0,  0],
                  [0,  0, 0,  0,  1, dt],
                  [0,  0, 0,  0,  0,  1],
              ], dtype=float)

              # Observation matrix — only position states measurable
              self.H = np.array([
                  [1, 0, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0],
              ], dtype=float)

              # Process noise — DWPA model (Q_XY=80, Q_W=20)
              def dwpa(q):
                  return q * np.array([[dt**3/3, dt**2/2],
                                       [dt**2/2, dt     ]])
              Q_XY, Q_W = 80.0, 20.0
              self.Q = np.zeros((6, 6))
              self.Q[0:2, 0:2] = dwpa(Q_XY)
              self.Q[2:4, 2:4] = dwpa(Q_XY)
              self.Q[4:6, 4:6] = dwpa(Q_W)

              self.R_base = np.diag([0.003, 0.003, 0.002])
              self.last_detection_t = time.time()

          def predict(self):
              self.x = self.F @ self.x
              self.P = self.F @ self.P @ self.F.T + self.Q

          def update(self, cx, cy_err, w, c_raw):
              """Measurement update — confidence-adaptive R."""
              if c_raw <= 0.05:
                  # No detection: exponential velocity decay
                  decay = max(0.0, 1.0 - 1.5 * self.DT)
                  self.x[1] *= decay
                  self.x[3] *= decay
                  self.x[5] *= decay
                  return

              self.last_detection_t = time.time()
              z = np.array([[cx], [cy_err], [w]])
              R = self.R_base / max(c_raw, 0.15)

              y  = z - self.H @ self.x
              S  = self.H @ self.P @ self.H.T + R
              K  = self.P @ self.H.T @ np.linalg.inv(S)
              self.x = self.x + K @ y
              self.P = (np.eye(6) - K @ self.H) @ self.P

          def get_predicted(self, pipeline_latency=0.080):
              """Look-ahead prediction to compensate end-to-end latency."""
              age = time.time() - self.last_detection_t
              horizon = min(age + pipeline_latency, 0.300)
              x_pred = self.x.copy()
              x_pred[0] += self.x[1] * horizon   # cx
              x_pred[2] += self.x[3] * horizon   # cy_err
              x_pred[4] += self.x[5] * horizon   # w
              # Clamp to FOV boundary
              x_pred[0] = np.clip(x_pred[0], -1.0, 1.0)
              x_pred[2] = np.clip(x_pred[2], -1.0, 1.0)
              return x_pred


      # ── ROS 2 Vision Node ────────────────────────────────────────────────────────
      class VisionNode(Node):

          # Camera tilt compensation: 25° downward tilt
          CY_SETPOINT = -np.tan(np.radians(25)) * 0.7          # ≈ -0.327
          CY_SETPOINT = float(np.clip(CY_SETPOINT, -0.6, -0.05))

          # Control gains
          KP_YAW, KD_YAW = 4.0, 0.5
          PNG_N          = 3.0
          KP_FWD         = 35.0
          W_TARGET       = 0.35    # desired bounding box width (≈ 15–20 m)
          VFF_LAT        = 3.0
          KP_Z, KD_Z     = 5.0, 0.5
          VFF_Z          = 2.0
          K_VEL, K_MAX   = 1.2, 2.5

          # Velocity saturation (m/s or rad/s)
          V_FWD_MAX  = 35.0
          V_LAT_MAX  = 15.0
          V_Z_MAX    = 15.0
          YAW_MAX    = 4.0

          EMA_ALPHA  = 0.25

          def __init__(self):
              super().__init__('vision_node')

              self.guidance_pub_ = self.create_publisher(Twist, '/aerotrack/guidance', 10)
              self.bridge_       = CvBridge()

              # Load YOLOv11n — fine-tuned on competition target images
              self.model_ = YOLO('yolov11n_aerotrack.pt')

              # Kalman filter and EMA state
              self.kalman_    = KalmanTracker6()
              self.ema_conf_  = 0.0
              self._lock      = threading.Lock()

              # OpenCV capture from HDMI capture card (device index 0)
              self.cap_ = cv2.VideoCapture(0, cv2.CAP_V4L2)
              self.cap_.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
              self.cap_.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
              self.cap_.set(cv2.CAP_PROP_FPS,          60)

              # 30 Hz video + Kalman update timer
              self.create_timer(1.0 / 30.0, self.video_callback)

              self.get_logger().info('VisionNode started — capturing from HDMI card')

          # ── 30 Hz: detect → Kalman update → publish guidance ─────────────────
          def video_callback(self):
              ret, frame = self.cap_.read()
              if not ret:
                  return

              cx, cy_err, w, c_raw = self._detect(frame)

              # EMA confidence smoothing
              self.ema_conf_ = (self.EMA_ALPHA * c_raw
                                + (1.0 - self.EMA_ALPHA) * self.ema_conf_)

              # Kalman update
              self.kalman_.predict()
              self.kalman_.update(cx, cy_err, w, c_raw)

              # Get latency-compensated prediction
              xp = self.kalman_.get_predicted()
              cx_p     = float(xp[0])
              cx_dot   = float(xp[1])
              cy_err_p = float(xp[2])
              cy_dot   = float(xp[3])
              w_p      = float(xp[4])

              # Adaptive gain
              target_speed = np.sqrt(cx_dot**2 + cy_dot**2)
              k_adapt = min(self.K_MAX, 1.0 + self.K_VEL * target_speed)

              # ── Compute body-frame guidance ──────────────────────────────────
              # Yaw: IBVS PD + PNG feedforward
              yaw_rate = (k_adapt * (self.KP_YAW * cx_p + self.KD_YAW * cx_dot)
                          + self.PNG_N * cx_dot)
              # FOV panic threshold
              if abs(cx_p) > 0.65:
                  yaw_rate = np.sign(cx_p) * self.YAW_MAX

              # Forward: distance hold via bounding box width
              w_err   = self.W_TARGET - w_p
              c_corner = max(0.3, 1.0 - abs(cx_p) * 0.9)
              v_fwd   = self.KP_FWD * (w_err / max(self.W_TARGET, 0.01)) * c_corner
              if w_p > 1.8 * self.W_TARGET:
                  v_fwd = -3.0   # collision brake
              elif w_p < 0.01:
                  v_fwd =  3.0   # creep forward during search

              # Lateral: Kalman velocity feedforward (intercept geometry)
              v_lat = k_adapt * self.VFF_LAT * cx_dot

              # Vertical: IBVS PD + PNG feedforward (tilt-compensated setpoint)
              v_z   = (k_adapt * (self.KP_Z * cy_err_p + self.KD_Z * cy_dot)
                       + self.PNG_N * self.VFF_Z * cy_dot)

              # Saturate
              v_fwd    = float(np.clip(v_fwd,    -self.V_FWD_MAX, self.V_FWD_MAX))
              v_lat    = float(np.clip(v_lat,    -self.V_LAT_MAX, self.V_LAT_MAX))
              v_z      = float(np.clip(v_z,      -self.V_Z_MAX,   self.V_Z_MAX))
              yaw_rate = float(np.clip(yaw_rate, -self.YAW_MAX,   self.YAW_MAX))

              # Publish Twist:
              #   linear.x  = v_fwd
              #   linear.y  = v_lat
              #   linear.z  = EMA confidence  (overloaded — read by C++ FSM node)
              #   angular.z = yaw_rate
              msg = Twist()
              msg.linear.x  = v_fwd
              msg.linear.y  = v_lat
              msg.linear.z  = self.ema_conf_      # ← EMA confidence for FSM
              msg.angular.z = yaw_rate
              self.guidance_pub_.publish(msg)

          # ── Detection: YOLOv11n with HSV fallback ────────────────────────────
          def _detect(self, frame):
              H, W = frame.shape[:2]
              cx = cy_err = w = c_raw = 0.0

              # Primary: YOLOv11n
              results = self.model_(frame, imgsz=640, conf=0.25, verbose=False)
              best = None
              for r in results:
                  for box in r.boxes:
                      if float(box.conf[0]) > (float(best.conf[0]) if best else 0.0):
                          best = box

              if best is not None and float(best.conf[0]) > 0.25:
                  x1, y1, x2, y2 = best.xyxy[0].cpu().numpy()
                  bw = x2 - x1;  bh = y2 - y1
                  cx      = ((x1 + bw / 2) - W / 2) / (W / 2)
                  cy_raw  = ((y1 + bh / 2) - H / 2) / (H / 2)
                  cy_err  = cy_raw - self.CY_SETPOINT
                  w       = bw / W
                  c_raw   = float(best.conf[0])
                  return cx, cy_err, w, c_raw

              # Fallback: HSV dual-range red segmentation
              hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
              mask1 = cv2.inRange(hsv, (0,   100, 100), (10,  255, 255))
              mask2 = cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
              mask  = cv2.morphologyEx(mask1 | mask2, cv2.MORPH_OPEN,
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
              mask  = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

              cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
              if cnts:
                  c    = max(cnts, key=cv2.contourArea)
                  area = cv2.contourArea(c)
                  if area >= 10:
                      bx, by, bw, bh = cv2.boundingRect(c)
                      cx     = ((bx + bw / 2) - W / 2) / (W / 2)
                      cy_raw = ((by + bh / 2) - H / 2) / (H / 2)
                      cy_err = cy_raw - self.CY_SETPOINT
                      w      = bw / W
                      c_raw  = min(1.0, 0.65 + (area - 10) / 8000)

              return cx, cy_err, w, c_raw

          def destroy_node(self):
              self.cap_.release()
              super().destroy_node()


      def main(args=None):
          rclpy.init(args=args)
          node = VisionNode()
          try:
              rclpy.spin(node)
          finally:
              node.destroy_node()
              rclpy.shutdown()


      if __name__ == '__main__':
          main()

  - name: "MAVLink Telemetry Logger (ROS 2 Node)"
    file: "telemetry_logger_node.py"
    language: "python"
    download_url: "https://github.com/Yosuf-Soliman/Portfolio/blob/main/src/aerotrack/telemetry_logger_node.py"
    content: |
      """
      telemetry_logger_node.py
      AeroTrack — ROS 2 Telemetry Logger

      Subscribes to MAVROS topics and /aerotrack/* topics and logs a 30-column
      CSV at 30 Hz for post-flight analysis.

      Logged columns:
        timestamp, fsm_state, ema_conf,
        v_fwd_cmd, v_lat_cmd, v_z_cmd, yaw_rate_cmd,
        roll, pitch, yaw,
        vx, vy, vz,
        altitude, heading,
        cx_kalman, cy_err_kalman, w_kalman,
        cx_dot_kalman, cy_dot_kalman,
        k_adapt, tracking_distance_est,
        mavros_mode, armed,
        imu_ax, imu_ay, imu_az,
        battery_voltage, battery_current, battery_remaining
      """

      import rclpy
      from rclpy.node import Node
      from geometry_msgs.msg import Twist
      from std_msgs.msg import String
      from mavros_msgs.msg import State, AttitudeTarget
      from sensor_msgs.msg import BatteryState, Imu
      from nav_msgs.msg import Odometry

      import csv
      import os
      from datetime import datetime


      class TelemetryLoggerNode(Node):

          LOG_DIR = os.path.expanduser('~/aerotrack_logs')
          LOG_HZ  = 30

          def __init__(self):
              super().__init__('telemetry_logger_node')

              os.makedirs(self.LOG_DIR, exist_ok=True)
              ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
              filepath = os.path.join(self.LOG_DIR, f'telemetry_{ts}.csv')

              self._file   = open(filepath, 'w', newline='')
              self._writer = csv.writer(self._file)
              self._writer.writerow([
                  'timestamp', 'fsm_state', 'ema_conf',
                  'v_fwd_cmd', 'v_lat_cmd', 'v_z_cmd', 'yaw_rate_cmd',
                  'roll', 'pitch', 'yaw',
                  'vx', 'vy', 'vz', 'altitude', 'heading',
                  'cx_kalman', 'cy_err_kalman', 'w_kalman',
                  'cx_dot_kalman', 'cy_dot_kalman',
                  'tracking_distance_est',
                  'mavros_mode', 'armed',
                  'imu_ax', 'imu_ay', 'imu_az',
                  'battery_voltage', 'battery_current', 'battery_remaining',
              ])

              # ── Cached state ──────────────────────────────────────────────────
              self._guidance  = Twist()
              self._fsm_state = 'INIT'
              self._mavstate  = State()
              self._odom      = Odometry()
              self._imu       = Imu()
              self._battery   = BatteryState()

              # ── Subscriptions ─────────────────────────────────────────────────
              self.create_subscription(Twist,        '/aerotrack/guidance',    self._on_guidance,  10)
              self.create_subscription(String,       '/aerotrack/fsm_state',   self._on_fsm,       10)
              self.create_subscription(State,        '/mavros/state',           self._on_mavstate,  10)
              self.create_subscription(Odometry,     '/mavros/local_position/odom', self._on_odom, 10)
              self.create_subscription(Imu,          '/mavros/imu/data',        self._on_imu,       10)
              self.create_subscription(BatteryState, '/mavros/battery',         self._on_battery,   10)

              self.create_timer(1.0 / self.LOG_HZ, self._log_row)

              self.get_logger().info(f'Telemetry logging → {filepath}')

          def _on_guidance(self,  msg): self._guidance  = msg
          def _on_fsm(self,       msg): self._fsm_state = msg.data
          def _on_mavstate(self,  msg): self._mavstate  = msg
          def _on_odom(self,      msg): self._odom      = msg
          def _on_imu(self,       msg): self._imu       = msg
          def _on_battery(self,   msg): self._battery   = msg

          def _log_row(self):
              now  = self.get_clock().now().nanoseconds * 1e-9
              vel  = self._odom.twist.twist.linear
              pose = self._odom.pose.pose

              # Bounding box width → estimated tracking distance (metres)
              w_est = self._guidance.linear.z   # reused field carrying ema_conf
              dist_est = (0.35 / max(w_est, 0.01)) * 17.5 if w_est > 0.01 else 999.0

              self._writer.writerow([
                  f'{now:.4f}',
                  self._fsm_state,
                  f'{self._guidance.linear.z:.4f}',
                  f'{self._guidance.linear.x:.3f}',
                  f'{self._guidance.linear.y:.3f}',
                  '0.000',
                  f'{self._guidance.angular.z:.3f}',
                  '0', '0', '0',   # roll/pitch/yaw — from EKF in production
                  f'{vel.x:.3f}', f'{vel.y:.3f}', f'{vel.z:.3f}',
                  f'{pose.position.z:.3f}',
                  '0',
                  '0', '0', '0',   # Kalman states published separately in production
                  '0', '0',
                  f'{dist_est:.2f}',
                  self._mavstate.mode,
                  str(self._mavstate.armed),
                  f'{self._imu.linear_acceleration.x:.4f}',
                  f'{self._imu.linear_acceleration.y:.4f}',
                  f'{self._imu.linear_acceleration.z:.4f}',
                  f'{self._battery.voltage:.3f}',
                  f'{self._battery.current:.3f}',
                  f'{self._battery.percentage:.2f}',
              ])

          def destroy_node(self):
              self._file.flush()
              self._file.close()
              super().destroy_node()


      def main(args=None):
          rclpy.init(args=args)
          node = TelemetryLoggerNode()
          try:
              rclpy.spin(node)
          finally:
              node.destroy_node()
              rclpy.shutdown()


      if __name__ == '__main__':
          main()

components:
  - name: "HSKRC MARK4 7-inch CFRP Frame"
    quantity: 1
    description: "295mm motor-to-motor diagonal, 5mm CFRP arms, 30.5×30.5mm stack. Chosen over 5-inch ImpulseRC Apex Evo for better endurance within the 15-inch competition rule."

  - name: "T-Motor F90 2806.5 1300KV"
    quantity: 4
    description: "5–6S rated, 45.1A peak, 2,360g max thrust per motor. 12N14P NMB bearings. Yields ~7:1 TWR at 5S AUW."

  - name: "HQProp 7×4×3 Propellers"
    quantity: 5
    description: "7-inch, 3-blade, matched to F90 motor at 5–6S. 3 spare sets carried."

  - name: "MicoAir743v2 Flight Controller"
    quantity: 1
    description: "STM32H743 @ 480 MHz, 2MB flash. Dual IMU (BMI088 + BMI270). ArduPilot 4.6+ running GUIDED_NOGPS mode. Replaces T-HOBBY F7 Pro (512KB flash, insufficient for ArduPilot)."

  - name: "MicoAir AM32 4-in-1 70A ESC"
    quantity: 1
    description: "AM32 firmware, DShot600 bidirectional RPM telemetry. 70A per channel. Built-in current sensor (12.75 mV/A). Stack-compatible with MicoAir743v2."

  - name: "Walksnail Avatar V2 Camera/VTX"
    quantity: 1
    description: "1080p60, 5.8 GHz digital link, ~22 ms air latency. HDMI output on VRX feeds GCS capture card."

  - name: "MicoAir LR 900 Matched Pair"
    quantity: 1
    description: "900 MHz, 500 mW, FHSS, MAVLink 2.0 at 57600 bps. −117 dBm sensitivity. Primary command uplink from GCS to FC."

  - name: "RadioMaster RP3 V2 ELRS Receiver"
    quantity: 1
    description: "2.4 GHz CRSF. CH5 = mode switch (GUIDED ↔ MANUAL), CH7 = return trigger. Pilot override failsafe within 500 ms of RC loss → ALT_HOLD."

  - name: "Profuse 5S 4200mAh 40C LiPo"
    quantity: 2
    description: "18.5V nominal, 77.7 Wh usable, 168A continuous discharge. 1 active + 1 hot-swap spare. Nearly double the energy of the PDR's 6S 2600mAh."

  - name: "HDMI Capture Card"
    quantity: 1
    description: "USB 3.0, 1080p60. Ingests Walksnail VRX HDMI output into GCS laptop as a V4L2 device (OpenCV VideoCapture index 0)."

  - name: "GCS Laptop with GPU (GTX 1660+)"
    quantity: 1
    description: "Lab-owned. Runs: YOLOv11n inference (~30 ms), ROS 2 nodes (vision, FSM, telemetry logger), MAVROS, and DroneKit. All AI processing offloaded from drone to GCS to keep AUW at 897.6 g."

gallery:
  - type: "image"
    file: "/assets/images/projects/aerotrack/featured.jpeg"
    description: "AeroTrack drone — 7-inch CFRP build ready for flight"
  - type: "image"
    file: "/assets/images/projects/aerotrack/system-architecture-block.jpeg"
    description: "Figure 5.2.1 — Full system architecture block diagram"
  - type: "image"
    file: "/assets/images/projects/aerotrack/drone-3d-render.jpeg"
    description: "Figure 5.1.2 — 3D isometric CAD render of AeroTrack"
  - type: "image"
    file: "/assets/images/projects/aerotrack/ecalc-range.jpeg"
    description: "Figure 5.1.1.1 — eCalc range estimator: flight time vs. airspeed"
  - type: "image"
    file: "/assets/images/projects/aerotrack/sitl-integrated-system.jpeg"
    description: "Figure 5.3.1 — Full integrated SITL: MAVProxy map, Unity 3D, OpenCV FPV window"
  - type: "image"
    file: "/assets/images/projects/aerotrack/kalman-prediction-deviation.jpeg"
    description: "Figure 5.3.2 — Kalman prediction deviation during COASTING phases"
  - type: "image"
    file: "/assets/images/projects/aerotrack/fsm-state-timeline.jpeg"
    description: "Figure 5.4.1 — FSM state timeline + EMA confidence over 309.8-second flight"
  - type: "image"
    file: "/assets/images/projects/aerotrack/yaw-pitch-error.jpeg"
    description: "Figure 5.5.1 — Yaw (avg 5.15°) and pitch (avg 9.22°) tracking error over mission"
  - type: "image"
    file: "/assets/images/projects/aerotrack/velocity-control-response.jpeg"
    description: "Figure 5.5.2 — Commanded forward velocity vs. actual ground speed"
  - type: "image"
    file: "/assets/images/projects/aerotrack/altitude-profile.jpeg"
    description: "Figure 6.3.1 — Altitude profile: ±0.386 m std-dev hold + RTL climb to 300 m"
  - type: "image"
    file: "/assets/images/projects/aerotrack/trajectory-2d.jpeg"
    description: "Figure 6.4.1 — Top-down 2D trajectory: pursuer (blue) + tracked target (red)"
  - type: "image"
    file: "/assets/images/projects/aerotrack/tracking-distance.jpeg"
    description: "Figure 6.4.2 — Relative tracking distance over time (10–20 m, 5 m floor never violated)"
---

## Project Overview

AeroTrack is a fully autonomous GPS-denied FPV pursuit drone developed by a 10-member team from Kadir Has University's Mechatronics Engineering department for **TEKNOFEST 2026 FPV Drone Tracking Competition**. The drone must autonomously locate and follow a moving target through 11 predefined flight maneuvers across four stations — with zero GPS available at any point during the mission.

The defining architectural decision is **offloading all AI processing to a laptop-based Ground Control Station (GCS)**, keeping the drone at 897.6 g AUW while leveraging full GPU power for vision inference. My personal contribution is the **ROS 2 / MAVROS / MAVLink GCS integration** — the software backbone that connects every subsystem from camera capture to motor commands.

**Team:** Abdelrahman Abdulsalam (Captain) · Ela Soyed · Eren Karatas · Alaa Osseiran · Islam Refaie · Yosuf Soliman · Abdala Hasen · Mohamad Kadid · Mohamad Humam Zaki  
**Advisor:** Assoc. Prof. Ertuğrul Tolga Duran

---

## My Contribution — ROS 2 / MAVROS / MAVLink GCS Pipeline

### Why This Architecture Exists

Running YOLO inference and MAVLink flight control in a single Python thread causes the flight controller to be starved during heavy GPU decode, producing command jitter and loss of GUIDED mode authority. The solution is a **concurrent ROS 2 architecture** that isolates safety-critical flight control (C++) from computationally heavy vision processing (Python).

### Node Graph

```
[vision_node.py]  ──/aerotrack/guidance──►  [flight_control_node.cpp]
     │                  Twist msg                      │
     │             (v_fwd, v_lat, ema_conf,             │
     │              yaw_rate)                           │
     │                                          /mavros/setpoint_raw/attitude
     │                                                  │
[telemetry_logger_node.py]  ◄── all topics    [MAVROS]──► FC (MAVLink 2.0)
                                                          900 MHz → Drone
```

### flight_control_node.cpp (C++)
The safety-critical layer. Responsibilities:
- Subscribes to `/aerotrack/guidance` (geometry_msgs/Twist) from the Python vision node
- Owns all **FSM state transitions** (INIT → TRACKING → COASTING → SWEEP → FAILSAFE) with deterministic C++ execution
- Applies a **debounce mechanism** (500 ms minimum) between MAVROS mode change requests to prevent MAVROS lockout
- Converts IBVS+PNG body-frame velocity guidance into **SET_ATTITUDE_TARGET** (quaternion + thrust) via `/mavros/setpoint_raw/attitude` at 50 Hz
- Implements **hardware-interrupt-based failsafe**: on extended target loss or pilot interrupt → `SetMode → ALT_HOLD` (bypasses EKF position-variance errors that cause uncommanded descents in LOITER)

### vision_node.py (Python)
The perception layer. Responsibilities:
- Captures 1080p60 FPV from Walksnail VRX via HDMI capture card as a V4L2 device
- Runs **YOLOv11n** GPU inference at ~30 FPS (640×640, batch=1, ~30 ms on GTX 1660+)
- Falls back to **HSV dual-range red segmentation** when YOLO confidence is low
- Maintains **KalmanTracker6** with 80 ms look-ahead latency compensation (capped at 300 ms)
- Computes IBVS + PNG guidance and publishes to `/aerotrack/guidance`
- Encodes EMA confidence in `Twist.linear.z` for the C++ FSM node

### telemetry_logger_node.py (Python)
Subscribes to all MAVROS topics + `/aerotrack/*` topics and logs a **30-column CSV at 30 Hz** for post-flight KPI analysis (the same format as `telemetry_20260601_141337.csv` in the CDR).

### Key Engineering Decisions

**Why DroneKit → TCP directly (not MAVProxy UDP relay)**
An early version routed MAVLink through MAVProxy's UDP relay. Under CPU load from OpenCV, this caused unpredictable latency spikes and `EOFError` heartbeat timeouts. The final architecture connects DroneKit directly to ArduPilot's native TCP interface (port 5760), eliminating heartbeat timeouts entirely.

**Why ALT_HOLD failsafe (not LOITER)**
In GPS-denied GUIDED_NOGPS mode, EKF position-variance errors accumulate over time. LOITER's position-hold relies on EKF position estimates and causes uncommanded descents when those estimates degrade. ALT_HOLD uses only the barometer, giving the pilot a stable hover to regain manual control.

**Why SET_ATTITUDE_TARGET (not SET_POSITION_TARGET)**
Body-frame velocity setpoints eliminate the need for NED-to-body yaw rotation transforms — a bug source in the original SITL implementation. The C++ node converts guidance intent into quaternion attitude + thrust, the highest-authority MAVLink control interface available on ArduPilot.

---

## System Architecture

### End-to-End Pipeline

| Stage | Latency | Notes |
|-------|---------|-------|
| Walksnail FPV air link (VTX → VRX) | ~22 ms | 5.8 GHz digital |
| HDMI capture card → GCS PC | ~10 ms | USB 3.0, V4L2 |
| YOLOv11n GPU inference | ~30 ms | 640×640, GTX 1660+ |
| Kalman update + guidance compute | ~5 ms | CPU |
| MAVLink command (900 MHz uplink) | ~20 ms | MicoAir LR 900 |
| FC DShot600 execution | ~2 ms | AM32 ESC |
| **Total end-to-end** | **≈89 ms** | Kalman 80 ms look-ahead compensates |

### RF Link Separation

| Band | Link | Direction |
|------|------|-----------|
| 5.8 GHz | Walksnail FPV video | Drone → GCS (video) |
| 900 MHz | MicoAir LR MAVLink | GCS → Drone (commands) |
| 2.4 GHz | RadioMaster ELRS | RC controller → Drone (pilot override) |

Spatial separation (≥10 cm between antennas) + orthogonal polarisation + pre-flight RF sweep prevent cross-band EMI.

---

## SITL Test Results

All SITL verification used ArduPilot v4.8.0-dev + custom Unity 3D simulation. The target drone executed a deterministic 11-maneuver mission profile (slaloms, helix climb, altitude dives, zig-zag, hairpin U-turn). Zero human input to the pursuer.

| KPI | Result | Assessment |
|-----|--------|------------|
| Total mission duration | 309.8 s | — |
| TRACKING state uptime | 62.0% (192.1 s) | Excellent |
| COASTING state | 6.0% (18.6 s) | Acceptable |
| SWEEP state | 8.8% (27.2 s) | Acceptable |
| Average EMA confidence | 0.680 | High |
| Altitude std-dev (tracking) | ±0.386 m | Exceptional stability |
| Average yaw error | 5.15° | Tight centering |
| Safety floor violations (<5m) | 0 | Pass |
| Max ground speed | 11.75 m/s | At WPNAV_SPEED limit |

---

## Safety Architecture

**Velocity saturation** — all commands hard-clamped before MAVLink transmission:

| Axis | Limit |
|------|-------|
| Forward | ±35.0 m/s |
| Lateral | ±15.0 m/s |
| Vertical | ±15.0 m/s |
| Yaw rate | ±4.0 rad/s (≈229°/s) |

**Anti-crash altitude floor** — descending commands blocked below 15 m AGL.  
**ELRS RC failsafe** — pilot signal loss → ALT_HOLD within 500 ms.  
**Pilot override** — CH7 on RC controller triggers immediate ALT_HOLD and hands authority back to pilot.

---

## Lessons Learned

**MAVProxy relay instability** — Removing the relay and connecting DroneKit directly to ArduPilot TCP eliminated all heartbeat timeouts. Lesson: avoid middleware hops in the critical control path.

**Altitude overshoot during aggressive pursuit** — Accumulated kinetic energy during high-speed forward flight caused the vertical PID to over-drive. Fixed by softening vertical gain at high ground speed and blocking downward commands below the altitude floor.

**Kalman over-prediction during extended target loss** — Without a cap, the predicted target position escaped the camera FOV, causing aggressive yaw corrections on reacquisition. Fixed by capping the prediction horizon at 300 ms and clamping predicted coordinates to [−1, +1].

**EKF failsafe mode selection** — Standard GPS-denied failsafe (LOITER) produced uncommanded descents in HITL testing. Switched to ALT_HOLD which uses barometric altitude only.

---

## Future Work

- Full HITL testing on physical hardware (post-CDR, hardware assembly in progress)
- YOLOv11n fine-tuning on real competition target images (currently using SITL synthetic data)
- PPO reinforcement learning agent integration (>1M ISAAC Lab training steps in progress)
- Field flight tests and policy optimisation (planned July 2026)
- Competition flight proof video submission (August–September 2026)

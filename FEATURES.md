# DecodeNL — Robot Feature Overview

This document describes every major feature of the DecodeNL FTC robot at a functional level: what each feature does, how it behaves, what other systems it integrates with, and what responsibilities it owns.

---

## 1. Drivetrain & Motion Control

**Purpose:** Move the robot precisely around the field in both teleoperated and autonomous modes.

**Behavior:**
- Four mecanum wheels provide omnidirectional movement (forward, strafe, rotate, and any vector combination).
- In TeleOp, supports both **field-centric** and **robot-centric** drive modes — the robot drives relative to the field's coordinate system or relative to its own chassis orientation.
- In autonomous, follows pre-built paths composed of straight lines (BezierLine) and curves (BezierCurve) with smooth heading interpolation along each segment.
- Paths can be chained together to form multi-step autonomous routines.

**Integrations:**
- Receives velocity commands from the gamepad in TeleOp or from path-following algorithms in autonomous mode.
- Provides estimated pose (X, Y, heading) to the vision and turret systems for localization and aiming.

**Responsibilities:**
- Execute drive commands reliably and at configurable maximum speeds/powers.
- Provide accurate pose estimation to all subsystems that depend on position (turret aiming, vision correction, path following).

---

## 2. Pedro Pathing Autonomous System

**Purpose:** Enable the robot to navigate the field autonomously through predefined or programmatically generated paths.

**Behavior:**
- Paths are constructed as chains of Bezier curves or lines with configurable starting and ending poses and headings.
- Each path has constraints: maximum velocity, acceleration, angular velocity, and angular acceleration.
- The follower uses a GoBilda Pinpoint localization system (dual encoder pods) for dead-reckoning position tracking.
- During autonomous execution, paths are followed sequentially. After each path completes, a mechanism action (e.g., intake, shoot) can be triggered before the next path begins.
- Supports path power scaling: individual paths can run at lower or higher maximum power.

**Integrations:**
- Uses odometry vision (Limelight) for periodic pose corrections to reduce drift.
- Feeds current and target pose data to the dashboard drawing system for visual debugging.
- Provides position data to the turret subsystem for automatic aiming.

**Responsibilities:**
- Navigate the robot through the full autonomous routine without human intervention.
- Maintain positional accuracy within tolerances sufficient for scoring and pickup operations.
- Handle path transitions gracefully (smooth acceleration/deceleration between segments).

---

## 3. Turret Shooter System

**Purpose:** Aim and launch game elements (artifacts) into the alliance-specific goal from any position on the field.

**Behavior:**
- The turret rotates horizontally to face the target goal. A PID controller continuously adjusts the rotation angle as the robot moves.
- Two flywheel motors spin up to a distance-dependent velocity, calculated by interpolating from a pre-calibrated lookup table.
- A hood servo adjusts the launch angle based on distance, also via an interpolated lookup table.
- The system compensates for robot motion by computing a "virtual robot pose" — it looks ahead along the robot's movement vector to lead the target.
- The turret knows which alliance side (Red or Blue) it is on and uses the corresponding goal location and angular offset.
- The shooter monitors its own flywheel velocity and reports when it has reached the target speed.

**Integrations:**
- Receives the robot's current pose from the drivetrain follower.
- Receives vision target data (yaw angle to the AprilTag on the goal) from the turret vision system.
- Sends telemetry data to the FTC Dashboard for tuning and debugging.

**Responsibilities:**
- Consistently aim at the correct goal for the current alliance.
- Automatically select the correct shooter velocity and hood angle based on distance.
- Track moving targets by anticipating robot motion.
- Report readiness (flywheels at target speed) before a shot is commanded.

---

## 4. Vision System — Architecture Overview

**Purpose:** The vision system provides two distinct capabilities: (1) tracking scoring targets for turret aiming, and (2) correcting odometry drift for global pose accuracy. Both capabilities share a common hardware abstraction layer.

**Architecture:**
- All Limelight cameras inherit functionality from `LimelightBase`, an abstract base class that handles:
  - Hardware initialization and pipeline switching
  - Camera lifecycle (`start()` / `stop()`)
  - Dashboard camera stream setup
  - Result validation (`isResultUsable`)
  - Tag ID validation (`isValidTagId`)
- Two concrete implementations: `TurretVision` (target tracking) and `VisionOdometry` (pose correction).
- Constants are separated by concern: `OdometryConstants`, `TurretVisionConstants`, and shared types in `VisionConstants`.
- Pose corrections are encapsulated in `UpdatePoseCommand` (FTCLib `CommandBase`) for clean command-based integration.

---

## 4a. Vision System — Turret Targeting

**Purpose:** Detect and track the goal's AprilTag to provide precise angular aiming data for the turret.

**Behavior:**
- Uses a dedicated Limelight 3A camera (named `limelight-turret`) pointed at the goal direction.
- Runs at approximately 30 Hz, extracting fiducial (AprilTag) results each cycle.
- Selects the best tag by largest target area among valid tag IDs (20 and 24).
- Applies a 1D Kalman filter to smooth the raw yaw measurement, reducing jitter from frame-to-frame noise.
- Uses meaningful Kalman tuning (`Q=0.1` process noise, `R=2.0` measurement noise) for effective filtering.
- Reports the filtered yaw offset and target area. Returns zero when no valid tag is detected.

**Integrations:**
- Outputs the filtered yaw angle to the turret subsystem for PID setpoint calculation.
- Validates data quality before forwarding (checks result validity, tag ID, and camera status).
- Streams its camera feed to the FTC Dashboard at 120 FPS for driver/operator viewing.

**Responsibilities:**
- Provide reliable, low-latency angular tracking of the scoring target.
- Gracefully handle temporary loss of sight (maintain last known angle, report no target).
- Filter out false or low-confidence detections.

---

## 4b. Vision System — Odometry Correction

**Purpose:** Correct cumulative dead-reckoning drift by fusing Limelight vision data with the Pinpoint odometry estimate.

**Behavior:**
- Uses a second dedicated Limelight 3A camera (named `odometryLimelight`) mounted for field-view.
- Runs at approximately 20 Hz, extracting 3D robot pose from detected AprilTags.
- Converts the 3D camera-space pose to 2D field coordinates, compensating for the camera's physical offset on the robot.
- Applies three independent 1D Kalman filters (X, Y, Heading) to smooth the vision pose estimates.
- Supports two pose retrieval methods:
  - **Fiducial-based**: extracts 3D pose from individual AprilTag detections (existing behavior).
  - **MT2-based**: uses Limelight's MegaTag2 fused pose output (lower latency, more stable).
- When a pose reset is triggered (manually in TeleOp or automatically in autonomous via `UpdatePoseCommand`):
  - Fuses the vision pose with the current odometry pose using configurable weights (70% odometry / 30% vision).
  - Updates the follower's pose to the fused result.
  - Rejects outlier readings that deviate more than 1 meter from expected position.
- `UpdatePoseCommand` handles both initial pose setting (in `initialize()`) and runtime corrections, with outlier rejection.

**Integrations:**
- Reads the current follower (odometry) pose when performing a correction.
- Writes the fused pose back into the follower, replacing the dead-reckoning estimate.
- Streams its camera feed to the FTC Dashboard at 30 FPS.
- Used by `UpdatePoseCommand` for both TeleOp manual resets and autonomous periodic corrections.

**Responsibilities:**
- Maintain accurate global position over the course of a match, counteracting wheel slip and encoder drift.
- Provide pose corrections gentle enough not to cause sudden robot jumps or path-following instability.
- Recognize and reject clearly erroneous vision readings.
- Separate initial pose estimation (hard reset) from incremental runtime corrections (fusion).

---

## 6. Intake System

**Purpose:** Collect game elements (artifacts) from the field and feed them into the robot's internal mechanism.

**Behavior:**
- A single DC motor with encoder feedback runs at a constant power (speed-controlled) when activated.
- Simple on/off behavior: pressing a button turns the intake on, releasing turns it off.
- Runs in RUN_USING_ENCODER mode to maintain consistent power regardless of battery voltage or load.

**Integrations:**
- Controlled from TeleOp via button bindings (left bumper on gamepad 1: press for on, release for off).
- Feeds collected artifacts into the gate mechanism, which then passes them to the shooter.

**Responsibilities:**
- Reliably collect game elements from the floor or pickup zone.
- Maintain consistent intake speed throughout the match.
- Consume minimal driver attention (hold-button operation).

---

## 7. Gate System

**Purpose:** Control the flow of game elements from the intake to the shooter by opening and closing a servo-operated gate.

**Behavior:**
- A servo moves between two positions: open (allows artifacts to pass) and closed (holds artifacts).
- Controlled by explicit commands (OpenGate / CloseGate), typically scheduled as part of a shooting sequence.
- Single-shot operation — the gate opens, the artifacts passes through into the shooter, then the gate closes.

**Integrations:**
- Physically positioned between the intake and the shooter on the robot.
- Acted upon by autonomous command sequences during scoring cycles.

**Responsibilities:**
- Hold game elements securely when closed.
- Release elements cleanly when opened, without jamming.
- Coordinate with shooter readiness (do not open until flywheels are at speed).

---

## 8. Dashboard Visualization

**Purpose:** Provide real-time visual debugging during development and match performance monitoring.

**Behavior:**
- Draws a representation of the robot (circle + heading line) on the FTC Dashboard field overlay.
- Draws the planned path for the current autonomous segment.
- Draws the robot's pose history trail to visualize recent movement.
- Displays numerical telemetry: robot pose (X, Y, heading), turret angle, distance to goal, shooter encoder velocities, vision tag status, and more.
- Streams camera feeds from both Limelights.

**Integrations:**
- Receives follower pose, current path, and pose history from the Pedro Pathing system.
- Receives turret state and shooter state from the turret subsystem.
- Receives vision detection status from both vision modules.

**Responsibilities:**
- Provide actionable feedback to drivers and programmers without distracting from match operations.
- Render data at a frame rate sufficient for smooth visual tracking (typically 20-30 FPS).

---

## 9. Pose Management & Alliance Awareness

**Purpose:** Maintain knowledge of the robot's alliance color and provide correct pose reset positions for each alliance.

**Behavior:**
- Tracks which alliance (Red or Blue) the robot is on, set at initialization.
- Mirrors goal poses and reset positions based on alliance (Red positions are mirror images of Blue across the field centerline).
- Provides a `resetPose` method that places the robot at the correct starting position for its alliance.

**Integrations:**
- Used by the turret subsystem to select the correct goal pose and angular offset for aiming.
- Used by autonomous routines to determine the correct starting pose and mirror paths for Red vs. Blue side.

**Responsibilities:**
- Ensure all alliance-dependent logic (aiming, pathing, scoring) uses the correct coordinate system.
- Eliminate the need for separate code paths for Red and Blue — mirroring handles the difference.

---

## 10. Hardware Abstraction Layer

**Purpose:** Provide robust, reusable wrappers around raw FTC hardware objects (servos, motors) to add feedback capabilities and closed-loop control.

### 10a. TauraServo

**Behavior:**
- Wraps a standard FTC Servo with optional analog feedback sensor support.
- When a feedback sensor is connected, it can report the servo's actual position as a raw angle (degrees) or as a normalized universal position.
- Supports continuous rotation mode with an incremental accumulator that unwraps across the 0° boundary, allowing tracking of multi-turn motion.

### 10b. PIDServo

**Behavior:**
- A closed-loop servo controller built on top of TauraServo.
- Uses the analog feedback reading as a measurement and a PIDFController to drive the servo to a commanded angular setpoint.
- Automatically converts the PIDF output back into servo position commands.

### 10c. PIDFController

**Behavior:**
- General-purpose PID + Feedforward controller with proportional, integral, derivative, and feedforward terms.
- Includes integral anti-windup (configurable limit) and output clamping.
- Timestep-aware: accurately computes derivative using elapsed real time between measurements.

**Responsibilities (of the hardware layer):**
- Isolate higher-level subsystems from hardware-specific details (e.g., analog voltage calibration, servo range scaling).
- Enable closed-loop position control for mechanisms that need it (hood, future articulated arms).
- Provide a consistent interface that can be swapped or upgraded without affecting subsystem code.
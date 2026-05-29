# Rosmaster S-curve Obstacle Avoidance Technical Report

## 1. System Goal

This solution implements a full autonomous run for the contest sequence:
Start -> Channel 1 -> Avoidance entry -> S-curve around two obstacles -> Avoidance exit -> Channel 2 -> Goal zone stop.

The implementation is non-ROS for motion control and directly drives Rosmaster via `Rosmaster_Lib`.

## 2. Hardware and Sensors

- Chassis controller: Rosmaster main board through serial SDK (`Rosmaster_Lib`).
- Vision sensor: Astra depth camera video stream (`/dev/camera_depth`, camera id `0x50`).
- Optional range aid: 4ROS lidar (via Python `rplidar`, no ROS node required).

## 3. Software Architecture

Main program: `s_curve_obstacle_avoidance.py`

Pipeline blocks:

1. Perception
2. Pose estimation
3. S-curve path planning
4. Trajectory tracking and safety guard
5. Chassis adaptation and actuation
6. Gate event logging for entrance/exit checkpoints

## 4. Perception Design

### 4.1 Red/Blue obstacle detection

- HSV segmentation for red (dual hue range) and blue.
- Morphology + contour filtering (area and circularity) to keep cylindrical-like blobs.
- Outputs per obstacle:
  - image position `(cx, cy)`
  - pixel diameter
  - estimated physical diameter (mm)
  - relative side (left/right)
  - forward/lateral distance estimate

### 4.2 White boundary line detection (50 mm)

- HSV white segmentation.
- Distance transform stroke-width estimation.
- Line width pixels -> metric scale using nominal 50 mm line width:
  - `px_per_mm = line_width_px / 50`
- This scale is used to estimate obstacle diameter in mm.

### 4.3 Entrance/exit observation

- Stripe score from ROI whiteness in lower-middle image region.
- Debounced gate events produce coarse positions for:
  - Channel 1 entry
  - Avoidance entry
  - Avoidance exit
  - Channel 2 entry
  - Goal zone

## 5. Pose and Localization

- Dead reckoning integrates velocity and yaw rate over time.
- If available, `get_motion_data()` feedback is fused as primary motion estimate.
- Output pose: `(x, y, yaw)` in local run frame.

## 6. Path Planning (S-curve)

- Judge gives obstacle left/right order (e.g. left-right).
- Robot independently detects which obstacle is red/blue.
- Planner selects first/second obstacle according to judge order.
- Generates local anchor points before, between, and after obstacles.
- Applies Chaikin smoothing to form a continuous S-shaped curve.

## 7. Motion Control and Multi-Chassis Adaptation

- Pure pursuit style tracking on the smoothed path.
- Lane-center correction from white line center.
- Chassis adapter modes:
  - `wheel`: direct differential/mecanum style command
  - `multi_leg`: reduced speed and yaw limits, stronger filtering
  - `humanoid`: more conservative speed and turning

Actuation API (non-ROS):

- `set_car_motion(vx, vy, wz)`

## 8. Safety and Robustness

### 8.1 Collision and line crossing priority

- If obstacle is too close in front and centered -> emergency stop.
- If line clearance on either side is too small -> steering bias away from line.

### 8.2 Real-time adaptation for noise/friction/slip

- EMA filtering on line center and line width estimates.
- Slip indicator from command yaw vs measured yaw discrepancy.
- If slip rises, linear speed is automatically reduced.

## 9. Speed Limits and Parking Accuracy

Hard constraints in code:

- Linear speed: `<= 0.2 m/s`
- Angular speed: `<= 10 rad/s`

Goal behavior:

- Enter goal hold state with zero velocity.
- Keep still for configured hold time to ensure stable final parking.

## 10. Innovation Points

- Fully non-ROS motion pipeline while keeping sensor fusion capability.
- Onboard metric calibration from known line width (50 mm).
- Unified planner-controller framework adaptable to wheel / multi-leg / humanoid motion envelopes.
- Slip-aware speed adaptation for variable surface friction.

## 11. Deployment and Run

Example command:

```bash
/usr/bin/python /home/jetson/codedemmo/new/s_curve_obstacle_avoidance.py --show --chassis wheel --judge-order left-right --use-lidar --lidar-port /dev/ttyUSB0
```

Without lidar:

```bash
/usr/bin/python /home/jetson/codedemmo/new/s_curve_obstacle_avoidance.py --show --chassis wheel --judge-order left-right
```

## 12. Limitations and Improvement Plan

- Monocular fallback distance is approximate without lidar depth association.
- Gate detection uses visual stripe heuristics and may need threshold retuning per lighting condition.
- Future work:
  - camera-lidar extrinsic calibration
  - model-based friction observer
  - final centimeter-level stop correction with fiducial marker or depth target

#!/usr/bin/env python3
# coding: utf-8

"""
Standalone S-curve obstacle avoidance for Yahboom Rosmaster.

Key constraints from contest task:
- No ROS motion-topic publishing.
- Detect red / blue cylindrical obstacles and estimate diameter (nominal 300 mm).
- Detect white boundary line (nominal width 50 mm).
- Build S-curve path around two obstacles with judge-specified left/right order.
- Linear speed <= 0.2 m/s, angular speed <= 10 rad/s.
"""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import cv2 as cv
import numpy as np


ROSMASTER_PATHS = [
    "/home/jetson/Rosmaster/rosmaster",
    "/home/jetson/Rosmaster/auto_drive",
]
for _p in ROSMASTER_PATHS:
    if _p not in sys.path and os.path.isdir(_p):
        sys.path.append(_p)

try:
    from Rosmaster_Lib import Rosmaster  # type: ignore
except Exception:
    Rosmaster = None

try:
    from camera_rosmaster import Rosmaster_Camera  # type: ignore
except Exception:
    Rosmaster_Camera = None


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def ema(prev: Optional[float], now: float, alpha: float) -> float:
    if prev is None:
        return now
    return alpha * now + (1.0 - alpha) * prev


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class ObstacleDetection:
    color: str
    cx: float
    cy: float
    radius_px: float
    diameter_px: float
    diameter_mm: float
    forward_m: float
    lateral_m: float
    side: str


@dataclass
class PerceptionFrame:
    frame: np.ndarray
    lane_center_px: Optional[float]
    left_clearance_px: float
    right_clearance_px: float
    line_width_px: Optional[float]
    px_per_mm: Optional[float]
    stripe_score: float
    obstacles: List[ObstacleDetection] = field(default_factory=list)


class DummyRobot:
    def __init__(self) -> None:
        self._motion = (0.0, 0.0, 0.0)

    def create_receive_threading(self) -> None:
        return

    def set_car_motion(self, vx: float, vy: float, wz: float) -> None:
        self._motion = (vx, vy, wz)

    def get_motion_data(self) -> Tuple[float, float, float]:
        return self._motion

    def set_beep(self, ms: int) -> None:
        _ = ms


class Lidar4ROS:
    def __init__(self, port: str, baudrate: int = 230400, debug: bool = False) -> None:
        self.debug = debug
        self._samples = np.full(360, np.nan, dtype=np.float32)
        self._alive = False
        self._thread: Optional[threading.Thread] = None
        self._lidar = None

        try:
            from rplidar import RPLidar  # type: ignore
        except Exception as ex:
            if self.debug:
                print("[lidar] rplidar import failed:", ex)
            return

        try:
            self._lidar = RPLidar(port, baudrate=baudrate)
            self._alive = True
            self._thread = threading.Thread(target=self._scan_loop, daemon=True)
            self._thread.start()
            if self.debug:
                print("[lidar] started on", port)
        except Exception as ex:
            self._lidar = None
            self._alive = False
            if self.debug:
                print("[lidar] init failed:", ex)

    def _scan_loop(self) -> None:
        if self._lidar is None:
            return
        try:
            for scan in self._lidar.iter_scans(max_buf_meas=500):
                if not self._alive:
                    break
                for _, angle, distance_mm in scan:
                    idx = int(angle) % 360
                    d = float(distance_mm) / 1000.0
                    if 0.05 <= d <= 8.0:
                        self._samples[idx] = d
        except Exception as ex:
            if self.debug:
                print("[lidar] scan stopped:", ex)

    def get_range(self, angle_deg: float, window: int = 6) -> Optional[float]:
        if not self._alive:
            return None
        center = int(round(angle_deg)) % 360
        idx = [(center + i) % 360 for i in range(-window, window + 1)]
        vals = self._samples[idx]
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
            return None
        return float(np.median(vals))

    def close(self) -> None:
        self._alive = False
        if self._lidar is None:
            return
        try:
            self._lidar.stop()
        except Exception:
            pass
        try:
            self._lidar.stop_motor()
        except Exception:
            pass
        try:
            self._lidar.disconnect()
        except Exception:
            pass


class VisionPerception:
    def __init__(self, fov_deg: float = 70.0) -> None:
        self.fov_deg = fov_deg
        self._line_width_px_ema: Optional[float] = None
        self._lane_center_ema: Optional[float] = None

    def _extract_lane(self, frame: np.ndarray) -> Tuple[np.ndarray, Optional[float], float, float, Optional[float], float]:
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        white_mask = cv.inRange(hsv, (0, 0, 160), (180, 65, 255))
        kernel = np.ones((3, 3), np.uint8)
        white_mask = cv.morphologyEx(white_mask, cv.MORPH_OPEN, kernel, iterations=1)
        white_mask = cv.morphologyEx(white_mask, cv.MORPH_CLOSE, kernel, iterations=1)

        h, w = white_mask.shape
        roi = white_mask[int(h * 0.45):, :]
        col_sum = np.sum(roi > 0, axis=0).astype(np.float32)

        lane_center_px = None
        if float(np.max(col_sum)) > 20.0:
            xs = np.arange(w, dtype=np.float32)
            lane_center_px = float(np.sum(xs * col_sum) / (np.sum(col_sum) + 1e-6))
            lane_center_px = ema(self._lane_center_ema, lane_center_px, 0.35)
            self._lane_center_ema = lane_center_px

        left_clearance_px = 1e6
        right_clearance_px = 1e6
        if lane_center_px is not None:
            cy = int(h * 0.80)
            row = white_mask[cy, :]
            center_i = int(clamp(lane_center_px, 0, w - 1))
            left_idx = np.where(row[:center_i] > 0)[0]
            right_idx = np.where(row[center_i:] > 0)[0]
            if left_idx.size > 0:
                left_clearance_px = float(center_i - left_idx[-1])
            if right_idx.size > 0:
                right_clearance_px = float(right_idx[0])

        dt = cv.distanceTransform((white_mask > 0).astype(np.uint8), cv.DIST_L2, 5)
        stroke = dt[dt > 0]
        line_width_px = None
        if stroke.size > 200:
            line_width_px = float(2.0 * np.median(stroke))
            line_width_px = ema(self._line_width_px_ema, line_width_px, 0.25)
            self._line_width_px_ema = line_width_px

        stripe_roi = white_mask[int(h * 0.65):int(h * 0.9), int(w * 0.25):int(w * 0.75)]
        stripe_score = float(np.mean(stripe_roi > 0))

        return white_mask, lane_center_px, left_clearance_px, right_clearance_px, line_width_px, stripe_score

    def _detect_color_obstacles(
        self,
        frame: np.ndarray,
        px_per_mm: Optional[float],
        lidar: Optional[Lidar4ROS],
    ) -> List[ObstacleDetection]:
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        h, w = frame.shape[:2]

        red_mask1 = cv.inRange(hsv, (0, 90, 40), (12, 255, 255))
        red_mask2 = cv.inRange(hsv, (165, 90, 40), (180, 255, 255))
        red_mask = cv.bitwise_or(red_mask1, red_mask2)
        blue_mask = cv.inRange(hsv, (95, 80, 40), (135, 255, 255))

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv.morphologyEx(red_mask, cv.MORPH_OPEN, kernel, iterations=1)
        blue_mask = cv.morphologyEx(blue_mask, cv.MORPH_OPEN, kernel, iterations=1)

        detections: List[ObstacleDetection] = []
        for color, mask in (("red", red_mask), ("blue", blue_mask)):
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv.contourArea(cnt)
                if area < 250.0:
                    continue
                (cx, cy), r = cv.minEnclosingCircle(cnt)
                if r < 6.0:
                    continue

                circularity = area / (math.pi * r * r + 1e-6)
                if circularity < 0.45:
                    continue

                diam_px = float(2.0 * r)
                diam_mm = 0.0
                if px_per_mm is not None and px_per_mm > 1e-3:
                    diam_mm = diam_px / px_per_mm

                x_norm = (cx - w * 0.5) / (w * 0.5)
                angle = x_norm * (self.fov_deg * 0.5)

                range_lidar = lidar.get_range(angle) if lidar is not None else None
                if range_lidar is not None:
                    forward_m = range_lidar
                else:
                    # Monocular fallback using image row ordering only.
                    forward_m = clamp(0.25 + 2.4 * (1.0 - cy / float(h)), 0.2, 3.0)

                lateral_m = x_norm * max(0.2, forward_m * math.tan(math.radians(self.fov_deg * 0.5)))
                side = "left" if cx < (w * 0.5) else "right"

                detections.append(
                    ObstacleDetection(
                        color=color,
                        cx=float(cx),
                        cy=float(cy),
                        radius_px=float(r),
                        diameter_px=diam_px,
                        diameter_mm=float(diam_mm),
                        forward_m=float(forward_m),
                        lateral_m=float(lateral_m),
                        side=side,
                    )
                )

        detections.sort(key=lambda d: d.forward_m)
        return detections

    def process(self, frame: np.ndarray, lidar: Optional[Lidar4ROS]) -> PerceptionFrame:
        lane_mask, lane_center_px, left_clr, right_clr, line_width_px, stripe_score = self._extract_lane(frame)

        px_per_mm = None
        if line_width_px is not None and line_width_px > 1e-3:
            # Nominal boundary line width is 50 mm.
            px_per_mm = line_width_px / 50.0

        obstacles = self._detect_color_obstacles(frame, px_per_mm, lidar)

        return PerceptionFrame(
            frame=frame,
            lane_center_px=lane_center_px,
            left_clearance_px=left_clr,
            right_clearance_px=right_clr,
            line_width_px=line_width_px,
            px_per_mm=px_per_mm,
            stripe_score=stripe_score,
            obstacles=obstacles,
        )


class PoseEstimator:
    def __init__(self) -> None:
        self.pose = Pose2D()
        self.last_ts = time.time()
        self.cmd_v = 0.0
        self.cmd_w = 0.0

    def set_command(self, v: float, w: float) -> None:
        self.cmd_v = v
        self.cmd_w = w

    def update(self, robot) -> Tuple[Pose2D, float]:
        now = time.time()
        dt = max(1e-3, now - self.last_ts)
        self.last_ts = now

        v = self.cmd_v
        w = self.cmd_w

        try:
            motion = robot.get_motion_data()
            if motion is not None and len(motion) >= 3:
                v = float(motion[0])
                w = float(motion[2])
        except Exception:
            pass

        self.pose.x += v * math.cos(self.pose.yaw) * dt
        self.pose.y += v * math.sin(self.pose.yaw) * dt
        self.pose.yaw = wrap_pi(self.pose.yaw + w * dt)

        slip = abs(self.cmd_w - w)
        return self.pose, slip


class GateTracker:
    def __init__(self) -> None:
        self.events: List[Tuple[str, Pose2D, float]] = []
        self.last_trigger_t = 0.0

    def update(self, stripe_score: float, pose: Pose2D, now: float) -> None:
        if stripe_score < 0.08:
            return
        if now - self.last_trigger_t < 2.0:
            return

        labels = [
            "channel1_entry",
            "avoid_entry",
            "avoid_exit",
            "channel2_entry",
            "goal_zone",
        ]
        idx = len(self.events)
        if idx >= len(labels):
            return

        self.events.append((labels[idx], Pose2D(pose.x, pose.y, pose.yaw), now))
        self.last_trigger_t = now


class SCurvePlanner:
    def __init__(self, judge_order: str = "left-right", safe_margin_m: float = 0.38) -> None:
        self.judge_order = judge_order
        self.safe_margin_m = safe_margin_m

    @staticmethod
    def _to_world(pose: Pose2D, lx: float, ly: float) -> Tuple[float, float]:
        wx = pose.x + lx * math.cos(pose.yaw) - ly * math.sin(pose.yaw)
        wy = pose.y + lx * math.sin(pose.yaw) + ly * math.cos(pose.yaw)
        return wx, wy

    @staticmethod
    def _chaikin(points: List[Tuple[float, float]], repeat: int = 2) -> List[Tuple[float, float]]:
        out = points
        for _ in range(repeat):
            if len(out) < 3:
                return out
            new_pts = [out[0]]
            for i in range(len(out) - 1):
                p0 = np.array(out[i], dtype=np.float32)
                p1 = np.array(out[i + 1], dtype=np.float32)
                q = 0.75 * p0 + 0.25 * p1
                r = 0.25 * p0 + 0.75 * p1
                new_pts.append((float(q[0]), float(q[1])))
                new_pts.append((float(r[0]), float(r[1])))
            new_pts.append(out[-1])
            out = new_pts
        return out

    def build(self, pose: Pose2D, obstacles: List[ObstacleDetection]) -> List[Tuple[float, float]]:
        if len(obstacles) < 2:
            coarse = [
                self._to_world(pose, 0.0, 0.0),
                self._to_world(pose, 0.8, 0.0),
                self._to_world(pose, 1.6, 0.0),
                self._to_world(pose, 2.4, 0.0),
            ]
            return self._chaikin(coarse, repeat=1)

        left_obs = [o for o in obstacles if o.side == "left"]
        right_obs = [o for o in obstacles if o.side == "right"]

        if self.judge_order == "left-right":
            first = min(left_obs, key=lambda o: o.forward_m) if left_obs else obstacles[0]
            second_candidates = [o for o in obstacles if o is not first]
            second = min(second_candidates, key=lambda o: o.forward_m)
        else:
            first = min(right_obs, key=lambda o: o.forward_m) if right_obs else obstacles[0]
            second_candidates = [o for o in obstacles if o is not first]
            second = min(second_candidates, key=lambda o: o.forward_m)

        pass_y_first = -self.safe_margin_m if first.side == "left" else self.safe_margin_m
        pass_y_second = self.safe_margin_m if first.side == "left" else -self.safe_margin_m

        x1 = max(0.6, first.forward_m)
        x2 = max(x1 + 0.4, second.forward_m)
        x_mid = 0.5 * (x1 + x2)

        local_pts = [
            (0.0, 0.0),
            (x1 - 0.35, 0.0),
            (x1, pass_y_first),
            (x_mid, 0.0),
            (x2, pass_y_second),
            (x2 + 0.45, 0.0),
            (x2 + 1.10, 0.0),
        ]
        world_pts = [self._to_world(pose, lx, ly) for lx, ly in local_pts]
        return self._chaikin(world_pts, repeat=2)


class TrajectoryFollower:
    def __init__(self) -> None:
        self.path: List[Tuple[float, float]] = []
        self.target_idx = 0

    def set_path(self, path: List[Tuple[float, float]]) -> None:
        self.path = path
        self.target_idx = 0

    def is_finished(self) -> bool:
        return len(self.path) > 0 and self.target_idx >= (len(self.path) - 1)

    def _nearest_idx(self, x: float, y: float) -> int:
        if not self.path:
            return 0
        pts = np.array(self.path, dtype=np.float32)
        d2 = np.sum((pts - np.array([x, y], dtype=np.float32)) ** 2, axis=1)
        return int(np.argmin(d2))

    def step(
        self,
        pose: Pose2D,
        lane_center_px: Optional[float],
        frame_w: int,
        max_linear: float,
        max_angular: float,
    ) -> Tuple[float, float]:
        if not self.path:
            return 0.0, 0.0

        nearest = self._nearest_idx(pose.x, pose.y)
        lookahead = 4
        self.target_idx = min(nearest + lookahead, len(self.path) - 1)

        tx, ty = self.path[self.target_idx]
        dx = tx - pose.x
        dy = ty - pose.y
        heading_target = math.atan2(dy, dx)
        heading_err = wrap_pi(heading_target - pose.yaw)

        dist = math.hypot(dx, dy)
        w_cmd = 2.2 * heading_err

        if lane_center_px is not None:
            lane_err = (lane_center_px - frame_w * 0.5) / (frame_w * 0.5)
            w_cmd += -0.8 * lane_err

        v_cmd = min(max_linear, 0.06 + 0.18 * math.exp(-abs(w_cmd)))
        if dist < 0.22:
            v_cmd *= 0.5

        return clamp(v_cmd, -max_linear, max_linear), clamp(w_cmd, -max_angular, max_angular)


class SafetySupervisor:
    def __init__(self, min_obs_dist: float = 0.30, min_border_clearance_px: float = 8.0) -> None:
        self.min_obs_dist = min_obs_dist
        self.min_border_clearance_px = min_border_clearance_px

    def apply(self, v_cmd: float, w_cmd: float, obs: List[ObstacleDetection], perc: PerceptionFrame) -> Tuple[float, float, str]:
        reason = "ok"

        for o in obs:
            if abs(o.lateral_m) < 0.22 and o.forward_m < self.min_obs_dist:
                return 0.0, 0.0, "emergency_obstacle"

        if perc.left_clearance_px < self.min_border_clearance_px:
            w_cmd -= 0.8
            reason = "left_line_guard"
        if perc.right_clearance_px < self.min_border_clearance_px:
            w_cmd += 0.8
            reason = "right_line_guard"

        return v_cmd, w_cmd, reason


class ChassisAdapter:
    def __init__(self, robot, chassis_type: str, max_linear: float, max_angular: float) -> None:
        self.robot = robot
        self.chassis_type = chassis_type
        self.max_linear = max_linear
        self.max_angular = max_angular
        self._vf = 0.0
        self._wf = 0.0

    def send(self, v_cmd: float, w_cmd: float) -> Tuple[float, float]:
        if self.chassis_type == "multi_leg":
            v_cmd = clamp(v_cmd, -0.12, 0.12)
            w_cmd = clamp(w_cmd, -5.0, 5.0)
        elif self.chassis_type == "humanoid":
            v_cmd = clamp(v_cmd, -0.08, 0.08)
            w_cmd = clamp(w_cmd, -3.0, 3.0)

        self._vf = 0.55 * self._vf + 0.45 * v_cmd
        self._wf = 0.55 * self._wf + 0.45 * w_cmd

        v = clamp(self._vf, -self.max_linear, self.max_linear)
        w = clamp(self._wf, -self.max_angular, self.max_angular)
        self.robot.set_car_motion(v, 0.0, w)
        return v, w

    def stop(self) -> None:
        try:
            self.robot.set_car_motion(0.0, 0.0, 0.0)
        except Exception:
            pass


class SCurveAutonomy:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.perception = VisionPerception(fov_deg=args.camera_fov_deg)
        self.pose_est = PoseEstimator()
        self.gates = GateTracker()
        self.planner = SCurvePlanner(judge_order=args.judge_order, safe_margin_m=args.safe_margin_m)
        self.follower = TrajectoryFollower()
        self.safety = SafetySupervisor(min_obs_dist=args.min_obstacle_dist)

        self.robot = self._build_robot(args.dry_run)
        self.camera = self._build_camera(args.camera_id, args.debug)
        self.lidar = Lidar4ROS(args.lidar_port, debug=args.debug) if args.use_lidar else None
        self.adapter = ChassisAdapter(self.robot, args.chassis, args.max_linear_speed, args.max_angular_speed)

        self.state = "SEARCH"
        self.goal_hold_start: Optional[float] = None
        self.stop_flag = False

    def _build_robot(self, dry_run: bool):
        if dry_run or Rosmaster is None:
            print("[robot] dummy mode")
            return DummyRobot()

        bot = Rosmaster(debug=self.args.debug)
        bot.create_receive_threading()
        try:
            bot.set_beep(100)
        except Exception:
            pass
        return bot

    def _build_camera(self, camera_id: int, debug: bool):
        if Rosmaster_Camera is None:
            raise RuntimeError("camera_rosmaster import failed")

        # Try preferred camera first, then common fallbacks used on Rosmaster.
        candidates = [camera_id, 0x50, 0x51, 0]
        tried = []
        for cid in candidates:
            if cid in tried:
                continue
            tried.append(cid)
            cam = Rosmaster_Camera(video_id=cid, debug=debug)
            if cam.isOpened():
                print(f"[camera] opened with id={hex(cid) if isinstance(cid, int) else cid}")
                return cam
            try:
                cam.clear()
            except Exception:
                pass

        # Fallback: try jetcam.USBCamera if available (some Jetson setups use jetcam)
        try:
            try:
                from jetcam.usb_camera import USBCamera  # type: ignore
            except Exception:
                # Try adding local workspace jetcam path
                local_jetcam = "/home/jetson/jetcam"
                if local_jetcam not in sys.path and os.path.isdir(local_jetcam):
                    sys.path.append(local_jetcam)
                from jetcam.usb_camera import USBCamera  # type: ignore

            class JetcamWrapper:
                def __init__(self, w=640, h=480):
                    self._cam = USBCamera(width=w, height=h)

                def isOpened(self):
                    return True

                def get_frame(self):
                    img = self._cam.read()
                    return True, img

                def get_frame_jpg(self, text="", color=(0, 255, 0)):
                    img = self._cam.read()
                    if text:
                        cv.putText(img, str(text), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    success, jpeg = cv.imencode('.jpg', img)
                    return success, jpeg.tobytes()

                def clear(self):
                    try:
                        del self._cam
                    except Exception:
                        pass

            print("[camera] trying jetcam.USBCamera fallback")
            jc = JetcamWrapper()
            if jc.isOpened():
                print("[camera] opened via jetcam.USBCamera")
                return jc
        except Exception:
            pass

        raise RuntimeError(
            "camera open failed for ids: " + ", ".join([hex(x) if isinstance(x, int) else str(x) for x in tried]) + ". No jetcam fallback."
        )

    def _draw(self, perc: PerceptionFrame, pose: Pose2D, status: str) -> np.ndarray:
        view = perc.frame.copy()
        h, w = view.shape[:2]

        if perc.lane_center_px is not None:
            cx = int(perc.lane_center_px)
            cv.line(view, (cx, h), (cx, int(h * 0.5)), (0, 255, 0), 2)
        cv.line(view, (w // 2, h), (w // 2, int(h * 0.5)), (80, 80, 255), 1)

        for obs in perc.obstacles:
            center = (int(obs.cx), int(obs.cy))
            rad = int(obs.radius_px)
            col = (0, 0, 255) if obs.color == "red" else (255, 0, 0)
            cv.circle(view, center, rad, col, 2)
            txt = f"{obs.color} d={obs.diameter_mm:.0f}mm f={obs.forward_m:.2f}m"
            cv.putText(view, txt, (center[0] - 80, max(20, center[1] - rad - 8)), cv.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

        if self.follower.path:
            px_pts = []
            for i in range(max(0, self.follower.target_idx - 5), min(len(self.follower.path), self.follower.target_idx + 20)):
                wx, wy = self.follower.path[i]
                dx = wx - pose.x
                dy = wy - pose.y
                lx = dx * math.cos(-pose.yaw) - dy * math.sin(-pose.yaw)
                ly = dx * math.sin(-pose.yaw) + dy * math.cos(-pose.yaw)
                ix = int(w * 0.5 + (ly / 1.2) * (w * 0.5))
                iy = int(h - (lx / 3.0) * h)
                px_pts.append((ix, iy))
            for i in range(len(px_pts) - 1):
                cv.line(view, px_pts[i], px_pts[i + 1], (0, 255, 255), 2)

        cv.putText(view, status, (10, 22), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
        cv.putText(
            view,
            f"pose=({pose.x:.2f},{pose.y:.2f},{pose.yaw:.2f}) stripe={perc.stripe_score:.2f}",
            (10, 44),
            cv.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
        return view

    def _select_two_obstacles(self, obs: List[ObstacleDetection]) -> List[ObstacleDetection]:
        if len(obs) <= 2:
            return obs

        by_dist = sorted(obs, key=lambda o: o.forward_m)
        return by_dist[:2]

    def _state_machine(self, perc: PerceptionFrame, pose: Pose2D, slip: float) -> Tuple[float, float, str]:
        status = f"state={self.state}"
        obs = self._select_two_obstacles(perc.obstacles)

        if self.state == "SEARCH":
            if len(obs) >= 2:
                path = self.planner.build(pose, obs)
                self.follower.set_path(path)
                self.state = "FOLLOW_S_CURVE"
                status = "state=FOLLOW_S_CURVE planned"
                return 0.0, 0.0, status
            return 0.06, 0.0, status

        if self.state == "FOLLOW_S_CURVE":
            if self.follower.is_finished():
                self.state = "CHANNEL2_TO_GOAL"
                status = "state=CHANNEL2_TO_GOAL"
                return 0.05, 0.0, status

            v, w = self.follower.step(
                pose,
                perc.lane_center_px,
                perc.frame.shape[1],
                self.args.max_linear_speed,
                self.args.max_angular_speed,
            )

            # Slip-aware speed adaptation.
            if slip > 1.2:
                v *= 0.65
                status += " slip_slow"

            return v, w, status

        if self.state == "CHANNEL2_TO_GOAL":
            lane_err = 0.0
            if perc.lane_center_px is not None:
                lane_err = (perc.lane_center_px - perc.frame.shape[1] * 0.5) / (perc.frame.shape[1] * 0.5)
            v = 0.07
            w = -1.2 * lane_err

            if len(self.gates.events) >= 5:
                self.state = "GOAL_HOLD"
                self.goal_hold_start = time.time()
                return 0.0, 0.0, "state=GOAL_HOLD"

            return v, w, status

        if self.state == "GOAL_HOLD":
            if self.goal_hold_start is None:
                self.goal_hold_start = time.time()
            if time.time() - self.goal_hold_start >= self.args.goal_hold_sec:
                self.state = "DONE"
                return 0.0, 0.0, "state=DONE"
            return 0.0, 0.0, status

        return 0.0, 0.0, "state=DONE"

    def run(self) -> None:
        print("[run] start")
        try:
            while not self.stop_flag:
                ok, frame = self.camera.get_frame()
                if not ok:
                    print("[camera] frame failed, reconnecting...")
                    self.camera.reconnect()
                    time.sleep(0.05)
                    continue

                pose, slip = self.pose_est.update(self.robot)
                perc = self.perception.process(frame, self.lidar)
                self.gates.update(perc.stripe_score, pose, time.time())

                v_cmd, w_cmd, status = self._state_machine(perc, pose, slip)
                v_cmd, w_cmd, safety_reason = self.safety.apply(v_cmd, w_cmd, perc.obstacles, perc)
                if safety_reason != "ok":
                    status += f" {safety_reason}"

                sent_v, sent_w = self.adapter.send(v_cmd, w_cmd)
                self.pose_est.set_command(sent_v, sent_w)

                if self.args.show:
                    view = self._draw(perc, pose, status)
                    cv.imshow("s_curve_obstacle_avoidance", view)
                    key = cv.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break

                if self.state == "DONE":
                    break

                time.sleep(0.02)

        finally:
            self.adapter.stop()
            if self.lidar is not None:
                self.lidar.close()
            try:
                self.camera.clear()
            except Exception:
                pass
            cv.destroyAllWindows()

            gate_dump = [
                {
                    "name": name,
                    "t": round(ts, 3),
                    "x": round(p.x, 3),
                    "y": round(p.y, 3),
                    "yaw": round(p.yaw, 3),
                }
                for name, p, ts in self.gates.events
            ]
            print("[run] gates:")
            print(json.dumps(gate_dump, indent=2))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Rosmaster S-curve obstacle avoidance (no ROS motion topic).")
    parser.add_argument("--chassis", type=str, default="wheel")
    parser.add_argument("--judge-order", choices=["left-right", "right-left"], default="left-right")
    parser.add_argument("--camera-id", type=lambda x: int(x, 0), default=0x50)
    parser.add_argument("--camera-fov-deg", type=float, default=70.0)
    parser.add_argument("--max-linear-speed", type=float, default=0.2)
    parser.add_argument("--max-angular-speed", type=float, default=10.0)
    parser.add_argument("--safe-margin-m", type=float, default=0.38)
    parser.add_argument("--min-obstacle-dist", type=float, default=0.30)
    parser.add_argument("--use-lidar", action="store_true")
    parser.add_argument("--lidar-port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--goal-hold-sec", type=float, default=1.2)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--debug", action="store_true")
    return parser


def normalize_chassis(chassis_raw: str) -> str:
    text = chassis_raw.strip().lower().replace("-", "_")
    aliases = {
        "wheel": "wheel",
        "wheels": "wheel",
        "wheeled": "wheel",
        "whell": "wheel",
        "multi_leg": "multi_leg",
        "multileg": "multi_leg",
        "multi-legged": "multi_leg",
        "humanoid": "humanoid",
        "human": "humanoid",
    }
    if text not in aliases:
        raise ValueError(
            f"invalid --chassis '{chassis_raw}'. Use one of: wheel, multi_leg, humanoid"
        )
    return aliases[text]


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

    try:
        args.chassis = normalize_chassis(args.chassis)
    except ValueError as ex:
        parser.error(str(ex))

    args.max_linear_speed = min(args.max_linear_speed, 0.2)
    args.max_angular_speed = min(args.max_angular_speed, 10.0)

    node = SCurveAutonomy(args)

    def _handle_sig(*_):
        node.stop_flag = True

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    node.run()


if __name__ == "__main__":
    main()

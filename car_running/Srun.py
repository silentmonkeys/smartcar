#!/usr/bin/env python3
# coding=utf-8
"""Open-loop mecanum route runner for the S-shaped course.

The script uses the key waypoints from the map, samples them into a smooth
trajectory, checks the sampled path against the obstacle safety radius, and
then drives the Rosmaster mecanum base with vx/vy commands while keeping
omega at zero.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


Point = tuple[float, float]


P0: Point = (-400.0, 0.0)
P1: Point = (0.0, 180.0)
P2: Point = (520.0, 325.0)
P3: Point = (850.0, 180.0)
P4: Point = (1200.0, 0.0)


SAFE_CLEARANCE_MM = 370.0
OBSTACLE_CENTERS: tuple[Point, ...] = ((325.0, 650.0), (325.0, 0.0))


@dataclass(frozen=True)
class CubicBezier:
	p0: Point
	p1: Point
	p2: Point
	p3: Point


def lerp_point(start: Point, end: Point, ratio: float) -> Point:
	return (
		start[0] + (end[0] - start[0]) * ratio,
		start[1] + (end[1] - start[1]) * ratio,
	)


def straight_curve(start: Point, end: Point) -> CubicBezier:
	return CubicBezier(
		start,
		lerp_point(start, end, 1.0 / 3.0),
		lerp_point(start, end, 2.0 / 3.0),
		end,
	)


ROUTE_WAYPOINTS: tuple[Point, ...] = (
	P0,
	P1,
	(-400.0, 180.0),
	(-400.0, 1100.0),
	(800.0, 1100.0),
	(800.0, 325.0),
	P2,
	P3,
	P4,
)


ROUTE_CURVES: tuple[CubicBezier, ...] = tuple(
	straight_curve(start, end)
	for start, end in zip(ROUTE_WAYPOINTS, ROUTE_WAYPOINTS[1:])
)


def _ensure_rosmaster_path() -> None:
	"""Add the local Rosmaster package path when it is available in the workspace."""

	candidate = Path("/home/jetson/yahboomcar_ros2_ws/software/py_install_V3.3.1")
	if candidate.exists() and str(candidate) not in sys.path:
		sys.path.insert(0, str(candidate))


def create_bot(port: str, car_type: int, debug: bool):
	"""Create the Rosmaster instance lazily so dry-run mode can work without hardware."""

	_ensure_rosmaster_path()
	try:
		from Rosmaster_Lib import Rosmaster
	except ImportError as exc:
		raise SystemExit(
			"Cannot import Rosmaster_Lib. Check that the Rosmaster package is installed "
			"or that /home/jetson/yahboomcar_ros2_ws/software/py_install_V3.3.1 exists."
		) from exc

	return Rosmaster(car_type, port, debug=debug)


def distance(a: Point, b: Point) -> float:
	return math.hypot(a[0] - b[0], a[1] - b[1])


def bezier_point(curve: CubicBezier, t: float) -> Point:
	u = 1.0 - t
	b0 = u * u * u
	b1 = 3.0 * u * u * t
	b2 = 3.0 * u * t * t
	b3 = t * t * t
	x = (
		curve.p0[0] * b0
		+ curve.p1[0] * b1
		+ curve.p2[0] * b2
		+ curve.p3[0] * b3
	)
	y = (
		curve.p0[1] * b0
		+ curve.p1[1] * b1
		+ curve.p2[1] * b2
		+ curve.p3[1] * b3
	)
	return (x, y)


def control_polygon_length(curve: CubicBezier) -> float:
	return (
		distance(curve.p0, curve.p1)
		+ distance(curve.p1, curve.p2)
		+ distance(curve.p2, curve.p3)
	)


def sample_curve(curve: CubicBezier, spacing_mm: float) -> list[Point]:
	sample_count = max(16, int(math.ceil(control_polygon_length(curve) / spacing_mm)))
	points = [bezier_point(curve, index / sample_count) for index in range(sample_count)]
	points.append(curve.p3)
	return points


def build_route(spacing_mm: float) -> list[Point]:
	route: list[Point] = []
	for curve in ROUTE_CURVES:
		segment_points = sample_curve(curve, spacing_mm)
		if route and segment_points:
			segment_points = segment_points[1:]
		route.extend(segment_points)
	return route


def clearance_margin(point: Point) -> float:
	nearest = min(distance(point, center) for center in OBSTACLE_CENTERS)
	return nearest - SAFE_CLEARANCE_MM


def validate_route(route: Sequence[Point]) -> float:
	min_margin = float("inf")

	for point in route:
		margin = clearance_margin(point)
		if margin < min_margin:
			min_margin = margin
		if margin < 0.0:
			raise ValueError(
				f"Path violates the safety radius at point {point!r}; margin={margin:.1f} mm"
			)

	return min_margin


def speed_for_margin(margin_mm: float, cruise_speed: float) -> float:
	if margin_mm < 40.0:
		return min(cruise_speed, 0.10)
	if margin_mm < 90.0:
		return min(cruise_speed, 0.14)
	if margin_mm < 160.0:
		return min(cruise_speed, 0.16)
	if margin_mm < 260.0:
		return min(cruise_speed, 0.20)
	return cruise_speed


def clamp(value: float, low: float, high: float) -> float:
	return max(low, min(high, value))


class RouteRunner:
	def __init__(
		self,
		port: str,
		car_type: int,
		debug: bool,
		mm_per_sec_at_unit_speed: float,
		command_period: float,
		invert_x: bool,
		invert_y: bool,
	) -> None:
		self._bot = create_bot(port=port, car_type=car_type, debug=debug)
		self._mm_per_sec_at_unit_speed = mm_per_sec_at_unit_speed
		self._command_period = command_period
		self._invert_x = -1.0 if invert_x else 1.0
		self._invert_y = -1.0 if invert_y else 1.0

	def stop(self) -> None:
		try:
			self._bot.set_car_motion(0.0, 0.0, 0.0)
		except Exception:
			pass

	def close(self) -> None:
		self.stop()
		try:
			self._bot.close()
		except Exception:
			try:
				del self._bot
			except Exception:
				pass

	def drive_segment(self, start: Point, end: Point, cruise_speed: float) -> None:
		dx = end[0] - start[0]
		dy = end[1] - start[1]
		length = math.hypot(dx, dy)
		if length < 1e-6:
			return

		margin = min(clearance_margin(start), clearance_margin(end))
		speed = speed_for_margin(margin, cruise_speed)
		speed = clamp(speed, 0.08, 0.22)
		vx = self._invert_x * speed * dx / length
		vy = self._invert_y * speed * dy / length
		duration = max(0.05, length / (self._mm_per_sec_at_unit_speed * speed))

		deadline = time.monotonic() + duration
		while True:
			remaining = deadline - time.monotonic()
			if remaining <= 0.0:
				break
			self._bot.set_car_motion(vx, vy, 0.0)
			time.sleep(min(self._command_period, remaining))

	def follow_path(self, route: Sequence[Point], cruise_speed: float) -> None:
		for start, end in zip(route, route[1:]):
			self.drive_segment(start, end, cruise_speed)
		self.stop()


def print_route_summary(route: Sequence[Point], min_margin: float, cruise_speed: float) -> None:
	print("Route waypoints:")
	for point in ROUTE_WAYPOINTS:
		print(f"  {point}")
	print(f"Sampled path points: {len(route)}")
	print(f"Minimum sampled safety margin: {min_margin:.1f} mm")
	print(f"Cruise speed command: {cruise_speed:.2f}")


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Run the mecanum S-curve route.")
	parser.add_argument("--port", default=os.getenv("ROSMASTER_PORT", "/dev/myserial"))
	parser.add_argument("--car-type", type=int, default=1)
	parser.add_argument("--debug", action="store_true")
	parser.add_argument("--dry-run", action="store_true")
	parser.add_argument("--spacing-mm", type=float, default=28.0)
	parser.add_argument("--cruise-speed", type=float, default=0.08)
	parser.add_argument("--mm-per-sec", type=float, default=520.0)
	parser.add_argument("--command-period", type=float, default=0.08)
	parser.add_argument("--invert-x", action="store_true", help="Flip the forward axis if needed")
	parser.add_argument("--invert-y", action="store_true", help="Flip the lateral axis if needed")
	return parser.parse_args()


def main() -> int:
	args = parse_args()
	route = build_route(spacing_mm=args.spacing_mm)
	min_margin = validate_route(route)
	cruise_speed = clamp(args.cruise_speed, 0.08, 0.12)
	print_route_summary(route, min_margin=min_margin, cruise_speed=cruise_speed)

	if args.dry_run:
		print("Dry-run mode: trajectory validated, no motion commands were sent.")
		return 0

	runner = RouteRunner(
		port=args.port,
		car_type=args.car_type,
		debug=args.debug,
		mm_per_sec_at_unit_speed=args.mm_per_sec,
		command_period=args.command_period,
		invert_x=args.invert_x,
		invert_y=args.invert_y,
	)

	def _shutdown(*_args: object) -> None:
		runner.stop()

	signal.signal(signal.SIGINT, _shutdown)
	signal.signal(signal.SIGTERM, _shutdown)

	try:
		runner.follow_path(route, cruise_speed=cruise_speed)
		print("Route completed.")
		return 0
	finally:
		runner.close()


if __name__ == "__main__":
	raise SystemExit(main())

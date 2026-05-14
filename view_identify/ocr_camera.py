#!/usr/bin/env python3

import os
import subprocess
import sys
import time
from pathlib import Path


def ensure_opencv_runtime():
	"""Switch to a runtime that can import cv2 and easyocr, preferring GUI when display exists."""
	candidate_runtimes = (
		"/home/jetson/yolov5_env/bin/python",
		"/home/jetson/miniconda3/envs/car1/bin/python",
	)
	need_gui = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

	probe_script = r"""
import re
import sys

require_gui = sys.argv[1] == "1"

try:
	import cv2  # noqa: F401
	import numpy  # noqa: F401
except Exception:
	sys.exit(1)

try:
	import easyocr  # noqa: F401
except Exception:
	sys.exit(3)

if not require_gui:
	sys.exit(0)

info = cv2.getBuildInformation()
match = re.search(r"^\s*GUI:\s*(.+)$", info, re.MULTILINE)
gui_backend = (match.group(1).strip().upper() if match else "")
if gui_backend in ("", "NONE", "NO"):
	sys.exit(2)

sys.exit(0)
"""

	def probe_runtime(python_path):
		return subprocess.run(
			[python_path, "-c", probe_script, "1" if need_gui else "0"],
			stdout=subprocess.DEVNULL,
			stderr=subprocess.DEVNULL,
			check=False,
		).returncode

	current_python = str(Path(sys.executable).resolve())
	current_status = probe_runtime(current_python)
	if current_status == 0:
		return
	if current_status == 2 and need_gui:
		print("[WARN] Current OpenCV runtime lacks GUI backend. Trying fallback runtimes...")
	if current_status == 3:
		print("[WARN] Current Python runtime cannot import easyocr. Trying fallback runtimes...")

	for candidate in candidate_runtimes:
		if not (os.path.isfile(candidate) and os.access(candidate, os.X_OK)):
			continue

		candidate_real = str(Path(candidate).resolve())
		if candidate_real == current_python:
			continue

		candidate_status = probe_runtime(candidate)
		if candidate_status == 0:
			print(f"[INFO] Switching to compatible Python runtime: {candidate}")
			os.execv(candidate, [candidate, str(Path(__file__).resolve()), *sys.argv[1:]])

	if current_status == 1:
		raise RuntimeError(
			"OpenCV import failed in the current Python runtime, and no compatible fallback runtime was found."
		)

	if current_status == 2 and need_gui:
		print("[WARN] Current OpenCV runtime lacks GUI backend. Falling back to headless OCR mode.")

	if current_status == 3:
		raise RuntimeError(
			"easyocr is not available in the current Python runtime, and no compatible fallback runtime was found."
		)


ensure_opencv_runtime()

import cv2


def open_camera(preferred_indexes=range(5), configurations=((1280, 720, 30), (640, 480, 30), (1920, 1080, 30))):
	"""Try multiple V4L2 indices and settings until a frame can actually be read."""
	for camera_index in preferred_indexes:
		for width, height, fps in configurations:
			cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
			if not cap.isOpened():
				cap.release()
				continue

			cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
			cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
			cap.set(cv2.CAP_PROP_FPS, fps)
			cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

			warmup_frame = None
			for _ in range(3):
				ret, frame = cap.read()
				if ret and frame is not None:
					warmup_frame = frame
					break

			if warmup_frame is None:
				cap.release()
				continue

			actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
			actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
			actual_fps = cap.get(cv2.CAP_PROP_FPS)
			print(f"[INFO] Camera found at index {camera_index}")
			print(f"[INFO] Resolution: {actual_width}x{actual_height} @ {actual_fps} FPS")
			return cap

		print(f"[WARN] Camera index {camera_index} opened but could not return a frame.")

	raise RuntimeError("Could not open a readable Astra RGB camera on any tested device index.")


def has_graphical_display():
	return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def has_opencv_highgui():
	"""Check whether current OpenCV build supports GUI windows."""
	try:
		cv2.namedWindow("__highgui_test__", cv2.WINDOW_NORMAL)
		cv2.destroyWindow("__highgui_test__")
		return True
	except cv2.error:
		return False


def build_reader():
	try:
		import easyocr
	except ImportError:
		print("[ERROR] easyocr not found.")
		print("Please install with:")
		print(f"  {sys.executable} -m pip install easyocr")
		sys.exit(1)

	try:
		return easyocr.Reader(["ch_sim", "en"], gpu=False)
	except Exception as exc:
		print(f"[ERROR] Failed to initialize EasyOCR: {exc}")
		sys.exit(1)


def draw_ocr_results(frame, results, min_confidence=0.30):
	for item in results:
		if len(item) != 3:
			continue
		bbox, text, conf = item
		if conf < min_confidence:
			continue

		points = [(int(p[0]), int(p[1])) for p in bbox]
		for i in range(len(points)):
			cv2.line(frame, points[i], points[(i + 1) % len(points)], (0, 255, 0), 2)

		x = max(0, points[0][0])
		y = max(20, points[0][1] - 10)
		label = f"{text} ({conf:.2f})"
		cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


def main():
	print("Initializing Astra OCR Camera...")

	try:
		cap = open_camera()
	except RuntimeError as exc:
		print(f"Error: {exc}")
		print("Please ensure:")
		print("1. Orbbec Astra camera is connected via USB")
		print("2. Astra drivers are installed")
		print("3. Camera is not being used by another application")
		sys.exit(1)

	reader = build_reader()
	print("[INFO] EasyOCR loaded.")

	preview_enabled = has_graphical_display() and has_opencv_highgui()
	if preview_enabled:
		print("Press 'q' to quit")
	else:
		print("[WARN] OpenCV GUI is unavailable in current environment; running in headless OCR mode.")
		print("[INFO] Press Ctrl+C to quit")

	last_ocr_time = 0.0
	ocr_interval_seconds = 1.0
	latest_results = []
	latest_texts = []

	try:
		while True:
			ret, frame = cap.read()
			if not ret or frame is None:
				print("Error: Could not read frame.")
				break

			now = time.time()
			if now - last_ocr_time >= ocr_interval_seconds:
				results = reader.readtext(frame)
				filtered = [item for item in results if len(item) == 3 and item[2] >= 0.30]

				current_texts = [item[1].strip() for item in filtered if item[1].strip()]
				if current_texts != latest_texts:
					if current_texts:
						print("[OCR] " + " | ".join(current_texts))
					else:
						print("[OCR] (no text)")
					latest_texts = current_texts

				latest_results = filtered
				last_ocr_time = now

			if preview_enabled:
				draw_ocr_results(frame, latest_results)
				try:
					cv2.imshow("Astra OCR View", frame)
				except cv2.error:
					print("[WARN] imshow failed; switching to headless OCR mode.")
					preview_enabled = False
					print("[INFO] Press Ctrl+C to quit")
					continue

				if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
					break
	except KeyboardInterrupt:
		print("\n[INFO] Stopped by user.")
	finally:
		cap.release()
		if preview_enabled:
			try:
				cv2.destroyAllWindows()
			except cv2.error:
				pass
		print("OCR viewer closed.")


if __name__ == "__main__":
	main()

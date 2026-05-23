#!/usr/bin/env python3

import os
import time
import subprocess
import sys
from pathlib import Path

def opencv_has_gui_support(cv2_module):
	build_info = cv2_module.getBuildInformation()
	for line in build_info.splitlines():
		if line.strip().startswith("GUI:"):
			upper_line = line.upper()
			return "NO" not in upper_line and "NONE" not in upper_line
	return False


def ensure_opencv_runtime():
	"""Switch to a known-good Python runtime if the current one cannot use OpenCV HighGUI."""
	try:
		import cv2

		if opencv_has_gui_support(cv2):
			return

		print("[WARN] Current OpenCV build has no GUI support; running in headless mode.")
		return
	except Exception as exc:
		current_exception = exc

	candidate_runtimes = (
		"/home/jetson/yolov5_env/bin/python",
		"/home/jetson/miniconda3/envs/car1/bin/python",
	)

	for candidate in candidate_runtimes:
		if not (os.path.isfile(candidate) and os.access(candidate, os.X_OK)):
			continue

		probe = subprocess.run(
			[
				candidate,
				"-c",
				"import sys\n"
				"try:\n"
				"    import cv2\n"
				"    from ultralytics import YOLO  # noqa: F401\n"
				"    import easyocr  # noqa: F401\n"
				"except Exception:\n"
				"    sys.exit(1)\n"
				"info = cv2.getBuildInformation()\n"
				"sys.exit(0 if any(line.strip().startswith('GUI:') and 'NO' not in line.upper() and 'NONE' not in line.upper() for line in info.splitlines()) else 1)",
			],
			stdout=subprocess.DEVNULL,
			stderr=subprocess.DEVNULL,
			check=False,
		)
		if probe.returncode == 0:
			print(f"[INFO] Switching to GUI-capable Python runtime: {candidate}")
			os.execv(candidate, [candidate, str(Path(__file__).resolve()), *sys.argv[1:]])

	raise RuntimeError(
		"OpenCV import failed in the current Python runtime. "
		"A compatible runtime is available at /home/jetson/yolov5_env/bin/python."
	) from current_exception


ensure_opencv_runtime()

import cv2
import easyocr
from ultralytics import YOLO


MODEL_PATH = Path("/home/jetson/codedemmo/view_identify/best.pt")
CAMERA_DEVICE = "/dev/camera_depth"
WINDOW_NAME = "OCR Camera Detection"
OCR_LANGS = ["ch_sim", "en"]
OCR_MIN_INTERVAL_SECONDS = 0.8
OCR_MIN_CONFIDENCE = 0.35
TEXT_CLASS_NAME = "text"
FRAME_CONFIGURATIONS = ((1280, 720, 30), (640, 480, 30), (1920, 1080, 30))
GUI_AVAILABLE = opencv_has_gui_support(cv2)


def create_ocr_reader():
	try:
		import torch
		use_gpu = torch.cuda.is_available()
	except Exception:
		use_gpu = False

	print(f"[INFO] Loading EasyOCR reader (gpu={use_gpu})")
	return easyocr.Reader(OCR_LANGS, gpu=use_gpu)


def open_camera(device_path=CAMERA_DEVICE, configurations=FRAME_CONFIGURATIONS):
	"""Open the configured camera device and confirm that it can return frames."""
	if not os.path.exists(device_path):
		raise RuntimeError(f"Camera device not found: {device_path}")

	for width, height, fps in configurations:
		cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
		if not cap.isOpened():
			cap.release()
			continue

		cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
		cap.set(cv2.CAP_PROP_FPS, fps)
		cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

		warmup_frame = None
		for _ in range(5):
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
		print(f"[INFO] Camera opened from {device_path}")
		print(f"[INFO] Resolution: {actual_width}x{actual_height} @ {actual_fps} FPS")
		return cap

	raise RuntimeError(f"Could not open a readable camera from {device_path}.")


def has_graphical_display():
	return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def load_model():
	if not MODEL_PATH.exists():
		raise FileNotFoundError(f"Model file not found: {MODEL_PATH}")
	return YOLO(str(MODEL_PATH))


def load_text_boxes(result, class_name=TEXT_CLASS_NAME, min_confidence=OCR_MIN_CONFIDENCE):
	boxes = getattr(result, "boxes", None)
	if boxes is None or len(boxes) == 0:
		return []

	names = getattr(result, "names", {}) or {}
	selected = []
	for box in boxes:
		confidence = float(box.conf.item())
		if confidence < min_confidence:
			continue

		class_id = int(box.cls.item())
		label = str(names.get(class_id, class_id)).lower()
		if label != class_name.lower():
			continue

		x1, y1, x2, y2 = box.xyxy[0].tolist()
		selected.append(
			{
				"box": (int(x1), int(y1), int(x2), int(y2)),
				"confidence": confidence,
				"label": label,
			}
		)

	selected.sort(key=lambda item: item["confidence"], reverse=True)
	return selected


def crop_with_padding(frame, box, padding_ratio=0.08):
	height, width = frame.shape[:2]
	x1, y1, x2, y2 = box
	box_width = max(1, x2 - x1)
	box_height = max(1, y2 - y1)
	pad_x = max(2, int(box_width * padding_ratio))
	pad_y = max(2, int(box_height * padding_ratio))
	left = max(0, x1 - pad_x)
	top = max(0, y1 - pad_y)
	right = min(width, x2 + pad_x)
	bottom = min(height, y2 + pad_y)
	return frame[top:bottom, left:right], (left, top, right, bottom)


def recognize_text(reader, crop):
	if crop is None or crop.size == 0:
		return []

	rgb_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
	results = reader.readtext(
		rgb_crop,
		detail=1,
		paragraph=False,
		batch_size=1,
		workers=0,
		allowlist=None,
		text_threshold=0.6,
		low_text=0.3,
		link_threshold=0.3,
		min_size=8,
		rotation_info=None,
	)
	texts = []
	for item in results:
		if len(item) < 2:
			continue
		text = str(item[1]).strip()
		if text:
			texts.append(text)
	return texts


def main():
	print("Initializing OCR camera viewer...")

	preview_enabled = has_graphical_display() and GUI_AVAILABLE
	if not preview_enabled:
		print("[WARN] Preview window disabled; OCR will run without GUI display.")

	try:
		cap = open_camera()
	except RuntimeError as exc:
		print(f"Error: {exc}")
		print("Please ensure:")
		print("1. Orbbec Astra camera is connected via USB")
		print("2. Astra drivers are installed")
		print("3. Camera is not being used by another application")
		sys.exit(1)

	try:
		model = load_model()
	except Exception as exc:
		print(f"Error loading model: {exc}")
		cap.release()
		sys.exit(1)

	try:
		reader = create_ocr_reader()
	except Exception as exc:
		print(f"Error loading EasyOCR: {exc}")
		cap.release()
		sys.exit(1)

	print("Press 'q' to quit")
	last_ocr_time = 0.0
	last_printed_text = ""

	try:
		while True:
			ret, frame = cap.read()
			if not ret or frame is None:
				print("Error: Could not read frame.")
				break

			results = model.predict(frame, verbose=False)
			result = results[0]
			detected_frame = result.plot()
			text_boxes = load_text_boxes(result)

			if text_boxes and (time.monotonic() - last_ocr_time) >= OCR_MIN_INTERVAL_SECONDS:
				best_box = text_boxes[0]
				crop, padded_box = crop_with_padding(frame, best_box["box"])
				recognized_texts = recognize_text(reader, crop)
				last_ocr_time = time.monotonic()
				if recognized_texts:
					recognized_text = " ".join(recognized_texts).strip()
					if recognized_text and recognized_text != last_printed_text:
						print(f"[OCR] {recognized_text}")
						last_printed_text = recognized_text
					x1, y1, x2, y2 = padded_box
					cv2.rectangle(detected_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
					cv2.putText(
						detected_frame,
						recognized_text[:40],
						(x1, max(20, y1 - 10)),
						cv2.FONT_HERSHEY_SIMPLEX,
						0.6,
						(0, 255, 255),
						2,
						cv2.LINE_AA,
					)
			if preview_enabled:
				cv2.imshow(WINDOW_NAME, detected_frame)

				if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
					break
	except KeyboardInterrupt:
			print("\nInterrupted by user.")
	finally:
		cap.release()
		if preview_enabled:
			cv2.destroyAllWindows()
		print("Viewer closed.")


if __name__ == '__main__':
	main()
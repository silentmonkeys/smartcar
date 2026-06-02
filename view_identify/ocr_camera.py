#!/usr/bin/env python3

import os
import time
import subprocess
import sys
from pathlib import Path

def opencv_has_gui_support(cv2_module, verbose=False):
	build_info = cv2_module.getBuildInformation()
	gui_line = None
	for line in build_info.splitlines():
		if line.strip().startswith("GUI:"):
			gui_line = line
			break
	
	if gui_line is None:
		if verbose:
			print("[DEBUG] No 'GUI:' line found in OpenCV build info")
		return False
	
	upper_line = gui_line.upper()
	has_gui_backend = any(backend in upper_line for backend in ["GTK", "QT", "WIN32UI", "COCOA"])
	is_disabled = "NO" in upper_line or "NONE" in upper_line
	result = has_gui_backend and not is_disabled
	
	if verbose:
		print(f"[DEBUG] OpenCV GUI line: {gui_line.strip()}")
		print(f"[DEBUG] Has GUI backend: {has_gui_backend}, Is disabled: {is_disabled}, Result: {result}")
	
	return result


def ensure_opencv_runtime():
	"""Switch to a known-good Python runtime if the current one cannot use OpenCV HighGUI."""
	try:
		import cv2

		if opencv_has_gui_support(cv2, verbose=True):
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
OCR_MIN_INTERVAL_SECONDS = 1.5
OCR_MIN_CONFIDENCE = 0.35
TEXT_CLASS_NAME = "text"
FRAME_CONFIGURATIONS = ((1280, 720, 30), (640, 480, 30), (1920, 1080, 30))
GUI_AVAILABLE = opencv_has_gui_support(cv2)

# OCR结果稳定性优化参数
RESULT_HISTORY_SIZE = 5  # 保留最近的识别结果数量
CONFIDENCE_THRESHOLD = 0.5  # OCR结果置信度阈值
SIMILARITY_THRESHOLD = 0.7  # 文本相似度阈值（用于去重）
STABILITY_THRESHOLD = 0.6  # 稳定性阈值（需要多少比例的结果一致）


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
		confidence = float(item[2]) if len(item) > 2 else 0.0
		if text and confidence >= CONFIDENCE_THRESHOLD:
			texts.append((text, confidence))
	return texts


def levenshtein_distance(s1, s2):
	"""计算两个字符串之间的编辑距离"""
	if len(s1) < len(s2):
		return levenshtein_distance(s2, s1)

	if len(s2) == 0:
		return len(s1)

	previous_row = range(len(s2) + 1)
	for i, c1 in enumerate(s1):
		current_row = [i + 1]
		for j, c2 in enumerate(s2):
			insertions = previous_row[j + 1] + 1
			deletions = current_row[j] + 1
			substitutions = previous_row[j] + (c1 != c2)
			current_row.append(min(insertions, deletions, substitutions))
		previous_row = current_row

	return previous_row[-1]


def text_similarity(s1, s2):
	"""计算两个文本的相似度（0-1之间）"""
	if not s1 or not s2:
		return 0.0
	max_len = max(len(s1), len(s2))
	if max_len == 0:
		return 1.0
	distance = levenshtein_distance(s1, s2)
	return 1.0 - (distance / max_len)


def is_stable_result(result_history, new_result):
	"""检查新识别结果是否与历史结果稳定一致"""
	if not result_history:
		return True

	similar_count = 0
	for hist_text, hist_conf in result_history:
		if text_similarity(new_result, hist_text) >= SIMILARITY_THRESHOLD:
			similar_count += 1

	return (similar_count / len(result_history)) >= STABILITY_THRESHOLD


def get_consensus_result(result_history):
	"""从历史结果中获取最稳定的共识结果"""
	if not result_history:
		return None

	# 按文本分组，计算每个文本的出现次数和平均置信度
	text_scores = {}
	for text, confidence in result_history:
		found = False
		for existing_text in list(text_scores.keys()):
			if text_similarity(text, existing_text) >= SIMILARITY_THRESHOLD:
				text_scores[existing_text]["count"] += 1
				text_scores[existing_text]["total_confidence"] += confidence
				found = True
				break
		if not found:
			text_scores[text] = {"count": 1, "total_confidence": confidence}

	# 找到出现次数最多且置信度最高的结果
	best_text = None
	best_score = 0.0
	for text, stats in text_scores.items():
		average_confidence = stats["total_confidence"] / stats["count"]
		score = stats["count"] * average_confidence
		if score > best_score:
			best_score = score
			best_text = text

	return best_text


def main():
	print("Initializing OCR camera viewer...")

	has_display = has_graphical_display()
	print(f"[DEBUG] DISPLAY env: {os.environ.get('DISPLAY', 'not set')}")
	print(f"[DEBUG] WAYLAND_DISPLAY env: {os.environ.get('WAYLAND_DISPLAY', 'not set')}")
	print(f"[DEBUG] has_graphical_display: {has_display}")
	print(f"[DEBUG] GUI_AVAILABLE: {GUI_AVAILABLE}")

	preview_enabled = has_display and GUI_AVAILABLE
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
	ocr_result_history = []  # 存储最近的OCR识别结果用于稳定性判断
	last_detection_box = None  # 上一次检测到的文本框位置
	BOX_CHANGE_THRESHOLD = 30  # 文本框位置变化阈值（像素）

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
			display_text = ""

			if text_boxes and (time.monotonic() - last_ocr_time) >= OCR_MIN_INTERVAL_SECONDS:
				best_box = text_boxes[0]
				
				# 检查文本框位置是否发生显著变化
				should_run_ocr = True
				if last_detection_box is not None:
					# 计算中心位置差异
					current_center = ((best_box["box"][0] + best_box["box"][2]) / 2, 
									 (best_box["box"][1] + best_box["box"][3]) / 2)
					last_center = ((last_detection_box[0] + last_detection_box[2]) / 2, 
								  (last_detection_box[1] + last_detection_box[3]) / 2)
					distance = ((current_center[0] - last_center[0])**2 + 
							   (current_center[1] - last_center[1])**2)**0.5
					if distance < BOX_CHANGE_THRESHOLD:
						should_run_ocr = False
				
				if should_run_ocr:
					last_detection_box = best_box["box"]
					crop, padded_box = crop_with_padding(frame, best_box["box"])
					recognized_texts = recognize_text(reader, crop)
					last_ocr_time = time.monotonic()
					
					if recognized_texts:
						# 获取当前识别结果（置信度最高的）
						recognized_texts.sort(key=lambda x: x[1], reverse=True)
						current_text = " ".join([t[0] for t in recognized_texts]).strip()
						current_confidence = recognized_texts[0][1]
						
						# 将当前结果加入历史记录
						ocr_result_history.append((current_text, current_confidence))
						
						# 保持历史记录在指定大小范围内
						if len(ocr_result_history) > RESULT_HISTORY_SIZE:
							ocr_result_history.pop(0)
						
						# 获取共识结果（稳定的识别结果）
						consensus_text = get_consensus_result(ocr_result_history)
						
						if consensus_text and consensus_text != last_printed_text:
							print(f"[OCR] {consensus_text}")
							last_printed_text = consensus_text
						
						# 使用共识结果作为显示文本
						display_text = consensus_text if consensus_text else current_text
						
						x1, y1, x2, y2 = padded_box
						cv2.rectangle(detected_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
						cv2.putText(
							detected_frame,
							display_text[:40],
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
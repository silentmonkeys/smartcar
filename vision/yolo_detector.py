from ultralytics import YOLO
from config.constants import MODEL_PATH, OBJECT_CLASSES, TEXT_CLASS_NAME, OCR_MIN_CONFIDENCE


def load_model():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"模型文件未找到: {MODEL_PATH}")
    return YOLO(str(MODEL_PATH))


def detect_boxes(result, target_labels=None, min_confidence=0.35):
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return []
    names = getattr(result, "names", {}) or {}
    detected = []
    for box in boxes:
        conf = float(box.conf.item())
        if conf < min_confidence:
            continue
        cls_id = int(box.cls.item())
        label = str(names.get(cls_id, cls_id)).lower()
        if target_labels and label not in target_labels:
            continue
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        detected.append({
            "box": (int(x1), int(y1), int(x2), int(y2)),
            "confidence": conf,
            "label": label,
            "x_center": (x1 + x2) / 2.0
        })
    detected.sort(key=lambda d: d["confidence"], reverse=True)
    return detected


def detect_objects(result):
    return detect_boxes(result, target_labels=OBJECT_CLASSES, min_confidence=0.35)


def detect_text_boxes(result):
    return detect_boxes(result, target_labels=[TEXT_CLASS_NAME.lower()],
                        min_confidence=OCR_MIN_CONFIDENCE)
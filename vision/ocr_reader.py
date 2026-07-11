import cv2
import easyocr
from config.constants import OCR_LANGS, CONFIDENCE_THRESHOLD, RESULT_HISTORY_SIZE, BOX_CHANGE_THRESHOLD
from utils.text_utils import text_similarity, get_consensus_result


def create_ocr_reader():
    try:
        import torch
        use_gpu = torch.cuda.is_available()
    except Exception:
        use_gpu = False
    print(f"[INFO] 加载EasyOCR (GPU={'启用' if use_gpu else '关闭'})")
    return easyocr.Reader(OCR_LANGS, gpu=use_gpu)


def crop_with_padding(frame, box, padding_ratio=0.08):
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = box
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    pad_x = max(2, int(bw * padding_ratio))
    pad_y = max(2, int(bh * padding_ratio))
    left = max(0, x1 - pad_x)
    top = max(0, y1 - pad_y)
    right = min(w, x2 + pad_x)
    bottom = min(h, y2 + pad_y)
    return frame[top:bottom, left:right], (left, top, right, bottom)


def recognize_text(reader, crop):
    if crop is None or crop.size == 0:
        return []
    rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
    results = reader.readtext(
        rgb, detail=1, paragraph=False, batch_size=1, workers=0,
        text_threshold=0.6, low_text=0.3, link_threshold=0.3,
        min_size=8, rotation_info=None
    )
    texts = []
    for item in results:
        if len(item) < 2:
            continue
        text = str(item[1]).strip()
        conf = float(item[2]) if len(item) > 2 else 0.0
        if text and conf >= CONFIDENCE_THRESHOLD:
            texts.append((text, conf))
    return texts


def detect_and_recognize_text(reader, frame, text_boxes, last_detection_box=None):
    if not text_boxes:
        return None, None, None

    best_box = text_boxes[0]

    do_ocr = True
    if last_detection_box is not None:
        cx = (best_box["box"][0] + best_box["box"][2]) / 2
        cy = (best_box["box"][1] + best_box["box"][3]) / 2
        lx = (last_detection_box[0] + last_detection_box[2]) / 2
        ly = (last_detection_box[1] + last_detection_box[3]) / 2
        if ((cx - lx)**2 + (cy - ly)**2)**0.5 < BOX_CHANGE_THRESHOLD:
            do_ocr = False

    if do_ocr:
        last_detection_box = best_box["box"]
        crop, _ = crop_with_padding(frame, best_box["box"])
        texts = recognize_text(reader, crop)
        return texts, last_detection_box, True

    return None, last_detection_box, False
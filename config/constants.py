import math
from pathlib import Path

MODEL_PATH = Path("/root/smartcar/best.pt")
ROS_IMAGE_TOPIC = "/usb_cam/image_raw"
OCR_LANGS = ["ch_sim", "en"]
OCR_MIN_CONFIDENCE = 0.35
TEXT_CLASS_NAME = "text"
OBJECT_CLASSES = ["cube", "sphere", "cylinder"]

RESULT_HISTORY_SIZE = 5
CONFIDENCE_THRESHOLD = 0.5
SIMILARITY_THRESHOLD = 0.7
STABILITY_THRESHOLD = 0.6
BOX_CHANGE_THRESHOLD = 30

YAW_TOL_HIGH = math.pi
YAW_TOL_LOW = 0.1
GOAL_TIMEOUT = 10.0

WHITELIST = ["正方体", "圆柱体", "球体"]

YOLO_TO_CHINESE = {
    "cube": "正方体",
    "sphere": "球体",
    "cylinder": "圆柱体"
}
CHINESE_TO_YOLO = {v: k for k, v in YOLO_TO_CHINESE.items()}

STOP_LOOP_TOPIC = "/stop_loop"

VIRTUAL_BOUNDARIES_TOPIC = "/virtual_obstacles"

VIRTUAL_BOUNDARIES = [
    (0.84, -0.26, 0.84, -1.0, 0.3),
    (0.84, 0.29, 0.85, 0.98, 0.3),
    (1.01, 1.11, 2.21, 1.17, 0.3),
    (0.99, -1.15, 2.13, -1.16, 0.3),
    (2.25, -0.99, 2.28, -0.25, 0.3),
    (2.31, 1.01, 2.32, 0.34, 0.3),
]
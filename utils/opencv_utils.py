import os
import sys
import subprocess
from pathlib import Path


def _has_gui(cv2_mod):
    info = cv2_mod.getBuildInformation()
    return any(
        line.strip().startswith("GUI:") and "NO" not in line.upper() and "NONE" not in line.upper()
        for line in info.splitlines()
    )


def ensure_opencv_runtime():
    try:
        import cv2
        if _has_gui(cv2):
            return
    except Exception:
        pass

    candidates = [
        "/home/jetson/yolov5_env/bin/python",
        "/home/jetson/miniconda3/envs/car1/bin/python",
    ]
    for candidate in candidates:
        if not (os.path.isfile(candidate) and os.access(candidate, os.X_OK)):
            continue
        probe_cmd = (
            f"{candidate} -c \""
            "import cv2, sys; "
            "from ultralytics import YOLO; "
            "import easyocr; "
            "info = cv2.getBuildInformation(); "
            "sys.exit(0 if any('GUI:' in l and 'NO' not in l.upper() for l in info.splitlines()) else 1)\""
        )
        if subprocess.run(probe_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
            print(f"[INFO] 切换到支持GUI的Python运行时: {candidate}")
            os.execv(candidate, [candidate, str(Path(__file__).resolve()), *sys.argv[1:]])

    raise RuntimeError("未找到支持GUI的OpenCV环境，请检查依赖")
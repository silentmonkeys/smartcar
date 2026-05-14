#!/usr/bin/env python3

import os
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

        current_runtime_lacks_gui = True
    except Exception as exc:
        current_runtime_lacks_gui = False
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
                "import cv2; import sys; info = cv2.getBuildInformation(); sys.exit(0 if any(line.strip().startswith('GUI:') and 'NO' not in line.upper() and 'NONE' not in line.upper() for line in info.splitlines()) else 1)",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if probe.returncode == 0:
            print(f"[INFO] Switching to GUI-capable Python runtime: {candidate}")
            os.execv(candidate, [candidate, str(Path(__file__).resolve()), *sys.argv[1:]])

    if current_runtime_lacks_gui:
        raise RuntimeError(
            "OpenCV is available, but the current Python runtime does not include GUI window support. "
            "A GUI-capable runtime was not found in the configured candidates."
        )

    raise RuntimeError(
        "OpenCV import failed in the current Python runtime. "
        "A compatible runtime is available at /home/jetson/yolov5_env/bin/python."
    ) from current_exception


ensure_opencv_runtime()

import cv2
import sys


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


def main():
    print("Initializing Orbbec Astra RGB Viewer...")

    try:
        cap = open_camera()
    except RuntimeError as exc:
        print(f"Error: {exc}")
        print("Please ensure:")
        print("1. Orbbec Astra camera is connected via USB")
        print("2. Astra drivers are installed")
        print("3. Camera is not being used by another application")
        sys.exit(1)

    if not has_graphical_display():
        print("Error: No graphical display was detected.")
        print("Run this script from a desktop session with DISPLAY or WAYLAND_DISPLAY set.")
        cap.release()
        sys.exit(1)

    print("Press 'q' to quit")

    try:
        while True:
            ret, frame = cap.read()

            if not ret or frame is None:
                print("Error: Could not read frame.")
                break

            cv2.imshow("Orbbec Astra RGB View", frame)

            if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Viewer closed.")

if __name__ == '__main__':
    main()

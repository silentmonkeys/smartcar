#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision test script (terminal-only, no GUI)

Workflow:
  1. Wait for first OCR label (正方体/圆柱体/球体)
     -> detect 3 objects -> build mapping -> freeze -> print command
  2. After that, keep recognizing OCR labels -> print command using frozen mapping
  3. Repeat forever until Ctrl+C

Run (inside container):
    cd /root/smartcar
    python3 test_vision.py
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Fix torch static TLS issue
if not os.environ.get('_LIBGOMP_PRELOADED'):
    os.environ['LD_PRELOAD'] = '/usr/lib/aarch64-linux-gnu/libgomp.so.1'
    os.environ['_LIBGOMP_PRELOADED'] = '1'
    os.execv(sys.executable, [sys.executable] + sys.argv)

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from config.constants import (
    ROS_IMAGE_TOPIC,
    WHITELIST,
    YOLO_TO_CHINESE,
    RESULT_HISTORY_SIZE,
    SIMILARITY_THRESHOLD,
)
from vision.yolo_detector import load_model, detect_objects, detect_text_boxes
from vision.ocr_reader import create_ocr_reader, detect_and_recognize_text
from utils.text_utils import text_similarity, get_consensus_result

POSITION_ACTION = {
    0: "向左走 (move_left)",
    1: "向前走 (move_forward)",
    2: "向右走 (move_right)",
}


class VisionTester:
    def __init__(self):
        rospy.init_node('vision_tester', anonymous=True)
        self.bridge = CvBridge()
        self.latest_frame = None

        rospy.loginfo("loading YOLO model...")
        self.model = load_model()
        rospy.loginfo("YOLO model loaded")

        rospy.loginfo("initializing OCR reader...")
        self.reader = create_ocr_reader()
        rospy.loginfo("OCR reader ready")

        self.ocr_history = []
        self.last_detection_box = None
        self.last_ocr_time = 0.0

        # frozen mapping: {chinese_label: action_name}
        self.frozen_mapping = None

        self.image_sub = rospy.Subscriber(ROS_IMAGE_TOPIC, Image, self._image_callback, queue_size=1)
        rospy.loginfo("subscribed to: %s", ROS_IMAGE_TOPIC)

    def _image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("image conversion failed: %s", e)

    def _match_whitelist(self, text):
        for item in WHITELIST:
            if text_similarity(text, item) >= SIMILARITY_THRESHOLD:
                return item
        return None

    def _build_mapping(self, object_boxes):
        mapping = {}
        order_cn = []
        for i, det in enumerate(object_boxes):
            label_en = det["label"]
            label_cn = YOLO_TO_CHINESE.get(label_en, label_en)
            action = POSITION_ACTION.get(i, f"未知动作{i}")
            mapping[label_cn] = action
            order_cn.append(label_cn)
        return mapping, order_cn

    def _print_mapping(self, mapping, order_cn):
        print("\n" + "=" * 60)
        print("  [映射建立] 物体顺序 (从左到右): " + " | ".join(order_cn))
        print("  " + "-" * 56)
        for label, action in mapping.items():
            print(f"  {label}  ->  {action}")
        print("  [映射已冻结]")
        print("=" * 60 + "\n")

    def _print_command(self, matched_label, mapping, round_num):
        action = mapping.get(matched_label, "未知动作")
        print("\n" + "*" * 60)
        print(f"  [第{round_num}轮] OCR识别标签: {matched_label}")
        print(f"  [执行指令] {matched_label}  ->  {action}")
        print("*" * 60 + "\n")

    def _get_frame(self):
        return self.latest_frame

    def _do_ocr(self, frame):
        """Run OCR, return matched whitelist label via consensus, or None."""
        results = self.model.predict(frame, verbose=False)
        text_boxes = detect_text_boxes(results[0])
        text_boxes.sort(key=lambda d: d["x_center"])

        now = time.monotonic()
        if text_boxes and (now - self.last_ocr_time) >= 1.0:
            ocr_texts, self.last_detection_box, did_ocr = detect_and_recognize_text(
                self.reader,
                frame,
                text_boxes,
                last_detection_box=self.last_detection_box,
                last_ocr_time=self.last_ocr_time,
            )
            if did_ocr:
                self.last_ocr_time = now

            if ocr_texts:
                for text, conf in ocr_texts:
                    matched = self._match_whitelist(text)
                    if matched:
                        self.ocr_history.append((matched, conf))
                        if len(self.ocr_history) > RESULT_HISTORY_SIZE:
                            self.ocr_history.pop(0)

                consensus = get_consensus_result(self.ocr_history)
                if consensus:
                    return consensus
        return None

    def run(self):
        rospy.loginfo("waiting for image...")
        wait_start = time.time()
        while self.latest_frame is None and not rospy.is_shutdown():
            if time.time() - wait_start > 10:
                rospy.logerr("image timeout, check: %s", ROS_IMAGE_TOPIC)
                return
            rospy.sleep(0.1)

        rospy.loginfo("image received, starting...\n")
        round_num = 0

        # ============================================================
        # Phase 1: Wait for first OCR label -> continuously detect
        #          objects until stable 3-object mapping is built
        # ============================================================
        rospy.loginfo("====== 阶段1: 等待第一个文字标签，建立映射 ======")

        first_label = None

        # Step 1a: Wait for first OCR label
        while not rospy.is_shutdown() and first_label is None:
            frame = self._get_frame()
            if frame is None:
                rospy.sleep(0.01)
                continue
            consensus = self._do_ocr(frame)
            if consensus:
                first_label = consensus
                rospy.loginfo("[阶段1] 第一个文字标签确认: %s", first_label)
                rospy.loginfo("[阶段1] 开始持续检测物体，建立完整映射...")
            rospy.sleep(0.05)

        # Step 1b: Continuously detect objects until stable mapping built
        stable_count = 0
        last_order = None
        stable_threshold = 3  # need 3 consecutive frames with same 3-object order

        while not rospy.is_shutdown() and self.frozen_mapping is None:
            frame = self._get_frame()
            if frame is None:
                rospy.sleep(0.01)
                continue

            results = self.model.predict(frame, verbose=False)
            object_boxes = detect_objects(results[0])
            object_boxes.sort(key=lambda d: d["x_center"])

            if len(object_boxes) == 3:
                current_order = tuple(d["label"] for d in object_boxes)
                if current_order == last_order:
                    stable_count += 1
                else:
                    last_order = current_order
                    stable_count = 1
                    order_cn = [YOLO_TO_CHINESE.get(d["label"], d["label"]) for d in object_boxes]
                    rospy.loginfo("[阶段1] 检测到3个物体: %s (稳定确认中 %d/%d)",
                                 " | ".join(order_cn), stable_count, stable_threshold)

                if stable_count >= stable_threshold:
                    self.frozen_mapping, order_cn = self._build_mapping(object_boxes)
                    self._print_mapping(self.frozen_mapping, order_cn)
                    round_num = 1
                    self._print_command(first_label, self.frozen_mapping, round_num)
                    self.ocr_history.clear()
                    rospy.loginfo("[阶段1完成] 映射已冻结，进入循环识别阶段")
            else:
                stable_count = 0
                last_order = None
                rospy.loginfo("[阶段1] 检测到%d个物体，等待3个物体同时出现...", len(object_boxes))

            rospy.sleep(0.05)

        # ============================================================
        # Phase 2: Keep recognizing labels -> execute via frozen mapping
        # ============================================================
        rospy.loginfo("====== 阶段2: 循环识别标签，按冻结映射执行 ======")

        while not rospy.is_shutdown():
            frame = self._get_frame()
            if frame is None:
                rospy.sleep(0.01)
                continue

            consensus = self._do_ocr(frame)
            if consensus:
                round_num += 1
                self._print_command(consensus, self.frozen_mapping, round_num)
                self.ocr_history.clear()
                rospy.loginfo("[第%d轮完成] 等待下一个标签...", round_num)

            rospy.sleep(0.05)

        rospy.loginfo("test finished")


if __name__ == '__main__':
    try:
        tester = VisionTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()



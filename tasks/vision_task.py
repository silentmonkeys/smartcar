import time
import rospy
import traceback
from voice import speak_cube, speak_cylinder, speak_sphere
from config.constants import (
    WHITELIST, YOLO_TO_CHINESE, RESULT_HISTORY_SIZE,
    SIMILARITY_THRESHOLD
)
from utils.stop_signal import init_stop_subscriber, should_stop
from utils.text_utils import text_similarity, get_consensus_result
from vision.image_subscriber import ROSImageSubscriber
from vision.yolo_detector import detect_objects, detect_text_boxes
from vision.ocr_reader import detect_and_recognize_text


TEXT_TO_SPEECH = {
    "\u6b63\u65b9\u4f53": speak_cube,
    "\u5706\u67f1\u4f53": speak_cylinder,
    "\u7403\u4f53": speak_sphere,
}

# OCR recognition interval (seconds)
OCR_INTERVAL = 1.0

# Number of consecutive frames with same 3-object order to confirm stable mapping
STABLE_THRESHOLD = 1


def _match_whitelist(text):
    """Match text against whitelist, return matched label or None."""
    for wl_item in WHITELIST:
        if text_similarity(text, wl_item) >= SIMILARITY_THRESHOLD:
            return wl_item
    return None


def _build_label_to_action(object_boxes, action_functions):
    """Build mapping from object position to action function.

    Position 0 (left)   -> action_functions[1] (move_left)
    Position 1 (center) -> action_functions[0] (move_forward)
    Position 2 (right)  -> action_functions[2] (move_right)
    """
    label_to_action = {}
    chinese_labels = []
    for i, det in enumerate(object_boxes):
        label_en = det["label"]
        label_cn = YOLO_TO_CHINESE.get(label_en, label_en)
        chinese_labels.append(label_cn)
        if i == 0:
            label_to_action[label_cn] = action_functions[1]
        elif i == 1:
            label_to_action[label_cn] = action_functions[0]
        elif i == 2:
            label_to_action[label_cn] = action_functions[2]
    return label_to_action, chinese_labels


def wait_for_first_ocr_label(model, reader, subscriber):
    """Step 1a: Wait for first OCR label that matches whitelist.

    Only runs OCR detection, no object detection.
    Returns the matched whitelist label, or None if stopped.
    """
    rospy.loginfo("====== \u9636\u6bb51a: \u7b49\u5f85\u7b2c\u4e00\u4e2a\u6587\u5b57\u6807\u7b7e\u786e\u8ba4 ======")

    ocr_history = []
    last_ocr_time = 0.0
    last_detection_box = None

    while not should_stop():
        frame = subscriber.get_frame()
        if frame is None:
            rospy.sleep(0.05)
            continue

        results = model.predict(frame, verbose=False)
        text_boxes = detect_text_boxes(results[0])
        now = time.monotonic()

        if text_boxes and (now - last_ocr_time) >= OCR_INTERVAL:
            texts, last_detection_box, did_ocr = detect_and_recognize_text(
                reader, frame, text_boxes,
                last_detection_box=last_detection_box,
                last_ocr_time=last_ocr_time,
            )
            if did_ocr:
                last_ocr_time = now

            if texts:
                for text, conf in texts:
                    matched = _match_whitelist(text)
                    if matched:
                        ocr_history.append((matched, conf))
                        if len(ocr_history) > RESULT_HISTORY_SIZE:
                            ocr_history.pop(0)

                consensus = get_consensus_result(ocr_history)
                if consensus:
                    rospy.loginfo("[\u9636\u6bb51a] \u7b2c\u4e00\u4e2a\u6587\u5b57\u6807\u7b7e\u786e\u8ba4: %s", consensus)
                    return consensus

        rospy.sleep(0.05)

    return None


def build_stable_mapping(model, subscriber, action_functions):
    """Step 1b: Continuously detect objects until stable mapping.

    Primary: 3 objects for STABLE_THRESHOLD consecutive frames.
    Fallback: 2 objects for TWO_OBJ_ACCEPT consecutive frames,
              third object position and label inferred automatically.
    Returns (label_to_action, chinese_labels), or (None, None) if stopped.
    """
    rospy.loginfo("====== 阶段1b: 持续检测物体，建立稳定映射 ======")

    stable_count = 0
    last_order = None

    # 2-object fallback
    TWO_OBJ_ACCEPT = 10
    two_obj_count = 0
    two_obj_last_order = None
    two_obj_boxes = None
    ALL_CN_LABELS = set(WHITELIST)

    while not should_stop():
        frame = subscriber.get_frame()
        if frame is None:
            rospy.sleep(0.05)
            continue

        results = model.predict(frame, verbose=False)
        object_boxes = detect_objects(results[0])
        object_boxes.sort(key=lambda d: d["x_center"])

        if len(object_boxes) == 3:
            two_obj_count = 0
            two_obj_last_order = None

            current_order = tuple(d["label"] for d in object_boxes)
            if current_order == last_order:
                stable_count += 1
            else:
                last_order = current_order
                stable_count = 1
                order_cn = [YOLO_TO_CHINESE.get(d["label"], d["label"]) for d in object_boxes]
                rospy.loginfo("[阶段1b] 检测到3个物体: %s (稳定确认 %d/%d)",
                             " | ".join(order_cn), stable_count, STABLE_THRESHOLD)

            if stable_count >= STABLE_THRESHOLD:
                label_to_action, order_cn = _build_label_to_action(object_boxes, action_functions)
                rospy.loginfo("[阶段1b] 映射建立完成:")
                for ch, act in label_to_action.items():
                    rospy.loginfo("  %s -> %s", ch, act.__name__)
                rospy.loginfo("[映射已冻结]")
                return label_to_action, order_cn

        elif len(object_boxes) == 2:
            stable_count = 0
            last_order = None

            current_2_order = tuple(d["label"] for d in object_boxes)
            if current_2_order == two_obj_last_order:
                two_obj_count += 1
            else:
                two_obj_last_order = current_2_order
                two_obj_count = 1
            two_obj_boxes = object_boxes

            order_cn = [YOLO_TO_CHINESE.get(d["label"], d["label"]) for d in object_boxes]
            rospy.loginfo("[阶段1b] 检测到2个物体: %s (连纭%d帧, 阈值%d)",
                         " | ".join(order_cn), two_obj_count, TWO_OBJ_ACCEPT)

            if two_obj_count >= TWO_OBJ_ACCEPT:
                frame_w = frame.shape[1]
                detected_cn = set(YOLO_TO_CHINESE.get(d["label"], d["label"]) for d in two_obj_boxes)
                missing_cn = ALL_CN_LABELS - detected_cn

                if len(missing_cn) == 1:
                    missing_label_cn = missing_cn.pop()

                    x_vals = [d["x_center"] for d in two_obj_boxes]
                    x_right = max(x_vals)

                    if missing_label_cn == "正方体":
                        # 正方体缺失时，直接补到最右边
                        missing_x = x_right + max(x_right - min(x_vals), 50)
                        rospy.loginfo("[阶段1b] 缺失正方体，自动补到最右边")
                    else:
                        x_left = min(x_vals)
                        if x_right < frame_w * 0.45:
                            missing_x = x_right + max(x_right - x_left, 50)
                        elif x_left > frame_w * 0.55:
                            missing_x = x_left - max(x_right - x_left, 50)
                        else:
                            missing_x = (x_left + x_right) / 2

                    virtual_entry = {"label": missing_label_cn, "x_center": missing_x}
                    full_boxes = list(two_obj_boxes) + [virtual_entry]
                    full_boxes.sort(key=lambda d: d["x_center"])

                    order_cn_full = [YOLO_TO_CHINESE.get(d["label"], d["label"]) for d in full_boxes]

                    rospy.loginfo("[阶段1b] 2物体稳定, 自动推断第三个:")
                    rospy.loginfo("  已检测: %s", " | ".join(order_cn))
                    rospy.loginfo("  缺失: %s", missing_label_cn)
                    rospy.loginfo("  完整顺序: %s", " | ".join(order_cn_full))

                    label_to_action, order_cn = _build_label_to_action(full_boxes, action_functions)
                    rospy.loginfo("[阶段1b] 映射建立完成(2+推断):")
                    for ch, act in label_to_action.items():
                        rospy.loginfo("  %s -> %s", ch, act.__name__)
                    rospy.loginfo("[映射已冻结]")
                    return label_to_action, order_cn
                else:
                    rospy.logwarn("[阶段1b] 无法确定缺失物体, 继续检测...")
                    two_obj_count = 0

        else:
            stable_count = 0
            last_order = None
            two_obj_count = 0
            two_obj_last_order = None
            rospy.loginfo("[阶段1b] 检测到%d个物体，等待2-3个物体稳定...", len(object_boxes))

        rospy.sleep(0.05)

    return None, None

def execute_action(matched_label, label_to_action, round_idx):
    """Execute the action for a matched label and announce via voice."""
    rospy.loginfo("[\u7b2c%d\u8f6e] \u6267\u884c\u52a8\u4f5c: %s", round_idx, matched_label)

    if matched_label in label_to_action:
        try:
            label_to_action[matched_label]()
            rospy.loginfo("[\u7b2c%d\u8f6e] \u52a8\u4f5c\u51fd\u6570\u6267\u884c\u5b8c\u6bd5", round_idx)
        except Exception as e:
            rospy.logerr("[\u7b2c%d\u8f6e] \u52a8\u4f5c\u51fd\u6570\u5f02\u5e38: %s", round_idx, e)
    else:
        rospy.logwarn("[\u7b2c%d\u8f6e] \u65e0\u6b64\u6620\u5c04: %s", round_idx, matched_label)

    if matched_label in TEXT_TO_SPEECH:
        speak_fn = TEXT_TO_SPEECH[matched_label]
        rospy.loginfo("[\u7b2c%d\u8f6e] \u8bed\u97f3\u64ad\u62a5: %s", round_idx, matched_label)
        for _ in range(3):
            if should_stop():
                break
            speak_fn()
            time.sleep(0.5)
    else:
        rospy.logwarn("[\u7b2c%d\u8f6e] \u65e0\u8bed\u97f3\u6620\u5c04: %s", round_idx, matched_label)


def process_last_waypoint(model, reader, action_functions):
    """Main vision task entry point.

    Phase 1: Wait for first OCR label -> build stable mapping -> execute first action
    Phase 2: Loop recognizing OCR labels -> execute via frozen mapping -> repeat
    """
    init_stop_subscriber()
    subscriber = ROSImageSubscriber()
    rospy.sleep(0.5)

    round_idx = 0

    try:
        # ============================================================
        # Phase 1: Wait for first OCR label -> build stable mapping -> execute
        # ============================================================
        rospy.loginfo("====== \u9636\u6bb51: \u7b49\u5f85\u7b2c\u4e00\u4e2a\u6587\u5b57\u6807\u7b7e\uff0c\u5efa\u7acb\u6620\u5c04 ======")

        # Step 1a: Wait for first OCR label
        first_label = wait_for_first_ocr_label(model, reader, subscriber)

        if should_stop():
            rospy.logwarn("\u9636\u6bb51\u88ab\u505c\u6b62\u4fe1\u53f7\u4e2d\u65ad")
            return

        if first_label is None:
            rospy.logerr("\u672a\u80fd\u786e\u8ba4\u7b2c\u4e00\u4e2a\u6587\u5b57\u6807\u7b7e\uff0c\u9000\u51fa")
            return

        # Step 1b: Build stable mapping (continuously detect until 3 consecutive frames)
        label_to_action, order_cn = build_stable_mapping(model, subscriber, action_functions)

        if should_stop():
            rospy.logwarn("\u6620\u5c04\u5efa\u7acb\u88ab\u505c\u6b62\u4fe1\u53f7\u4e2d\u65ad")
            return

        if label_to_action is None:
            rospy.logerr("\u672a\u80fd\u5efa\u7acb\u6709\u6548\u7269\u4f53\u6620\u5c04\uff0c\u9000\u51fa")
            return

        rospy.loginfo("====== \u6620\u5c04\u5df2\u51bb\u7ed3\uff0c\u5f00\u59cb\u5faa\u73af\u6267\u884c ======")

        # Execute first label's action
        round_idx = 1
        execute_action(first_label, label_to_action, round_idx)

        # ============================================================
        # Phase 2: Keep recognizing labels -> execute via frozen mapping
        # ============================================================
        rospy.loginfo("====== \u9636\u6bb52: \u5faa\u73af\u8bc6\u522b\u6807\u7b7e\uff0c\u6309\u6620\u5c04\u6267\u884c ======")

        ocr_history = []
        last_ocr_time = 0.0
        last_detection_box = None

        while not should_stop():
            frame = subscriber.get_frame()
            if frame is None:
                rospy.sleep(0.05)
                continue

            results = model.predict(frame, verbose=False)
            text_boxes = detect_text_boxes(results[0])
            now = time.monotonic()

            if text_boxes and (now - last_ocr_time) >= OCR_INTERVAL:
                texts, last_detection_box, did_ocr = detect_and_recognize_text(
                    reader, frame, text_boxes,
                    last_detection_box=last_detection_box,
                    last_ocr_time=last_ocr_time,
                )
                if did_ocr:
                    last_ocr_time = now

                if texts:
                    for text, conf in texts:
                        matched = _match_whitelist(text)
                        if matched:
                            ocr_history.append((matched, conf))
                            if len(ocr_history) > RESULT_HISTORY_SIZE:
                                ocr_history.pop(0)

                    consensus = get_consensus_result(ocr_history)
                    if consensus:
                        round_idx += 1
                        rospy.loginfo("[\u7b2c%d\u8f6e] OCR\u8bc6\u522b\u6807\u7b7e: %s", round_idx, consensus)
                        execute_action(consensus, label_to_action, round_idx)
                        # Clear history so next label can reach consensus quickly
                        ocr_history.clear()
                        last_ocr_time = 0.0
                        last_detection_box = None
                        rospy.loginfo("[\u7b2c%d\u8f6e] \u5b8c\u6210\uff0c\u7b49\u5f85\u4e0b\u4e00\u4e2a\u6807\u7b7e...", round_idx)

            rospy.sleep(0.05)

    except Exception as e:
        rospy.logerr("\u89c6\u89c9\u4efb\u52a1\u51fa\u73b0\u5f02\u5e38: %s", e)
        rospy.logerr(traceback.format_exc())
    finally:
        subscriber.shutdown()
        if rospy.is_shutdown():
            rospy.loginfo("ROS \u5df2\u5173\u95ed\uff0c\u89c6\u89c9\u4efb\u52a1\u6b63\u5e38\u9000\u51fa")
        rospy.loginfo("\u89c6\u89c9\u4efb\u52a1\u7ed3\u675f\uff0c\u5171\u6267\u884c %d \u8f6e", round_idx)

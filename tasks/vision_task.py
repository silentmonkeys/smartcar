import time
import rospy
import traceback
from TTL import speak_cube, speak_cylinder, speak_sphere
from config.constants import (
    WHITELIST, YOLO_TO_CHINESE, RESULT_HISTORY_SIZE, BOX_CHANGE_THRESHOLD,
    SIMILARITY_THRESHOLD
)
from utils.stop_signal import init_stop_subscriber, should_stop
from utils.text_utils import text_similarity, get_consensus_result
from vision.image_subscriber import ROSImageSubscriber
from vision.yolo_detector import detect_objects, detect_text_boxes
from vision.ocr_reader import detect_and_recognize_text


TEXT_TO_SPEECH = {
    "正方体": speak_cube,
    "圆柱体": speak_cylinder,
    "球体": speak_sphere,
}


def detect_objects_and_build_mapping(model, reader, subscriber, action_functions):
    rospy.loginfo("=== 开始实时检测物体并建立映射 ===")

    ocr_history = []
    last_ocr_time = 0.0
    last_detection_box = None
    last_text_time = time.monotonic()

    current_detected_order = None

    last_complete_order = None
    last_complete_label_to_action = None

    while not should_stop():
        frame = subscriber.get_frame()
        if frame is None:
            rospy.sleep(0.05)
            continue

        results = model.predict(frame, verbose=False)
        now = time.monotonic()

        object_boxes = detect_objects(results[0])
        text_boxes = detect_text_boxes(results[0])

        if object_boxes:
            object_boxes.sort(key=lambda d: d["x_center"])
            detected_order = [obj["label"] for obj in object_boxes]

            if detected_order != current_detected_order:
                current_detected_order = detected_order
                order_cn = [YOLO_TO_CHINESE.get(l, l) for l in detected_order]

                if len(detected_order) == 3:
                    chinese_labels = [YOLO_TO_CHINESE.get(l, l) for l in detected_order]

                    last_complete_label_to_action = {}
                    for i, ch in enumerate(chinese_labels):
                        if i == 0:
                            last_complete_label_to_action[ch] = action_functions[1]
                        elif i == 1:
                            last_complete_label_to_action[ch] = action_functions[0]
                        elif i == 2:
                            last_complete_label_to_action[ch] = action_functions[2]

                    last_complete_order = detected_order
                    rospy.loginfo("[物体检测] 完整检测到3个物体（左→右）: %s (已保存)", order_cn)
                    rospy.loginfo("[物体检测] 当前映射关系:")
                    for ch, act in last_complete_label_to_action.items():
                        rospy.loginfo("  %s → %s", ch, act.__name__)
                else:
                    rospy.loginfo("[物体检测] 检测到 %d 个物体（左→右）: %s", len(detected_order), order_cn)
                    if last_complete_order:
                        rospy.loginfo("[物体检测] 检测不完整，将使用最近一次完整检测: %s",
                                     [YOLO_TO_CHINESE.get(l, l) for l in last_complete_order])

        if text_boxes and (now - last_ocr_time) >= 1.5:
            texts, last_detection_box, did_ocr = detect_and_recognize_text(
                reader,
                frame,
                text_boxes,
                last_detection_box=last_detection_box,
                last_ocr_time=last_ocr_time,
            )
            if did_ocr:
                last_ocr_time = now

            if texts:
                texts.sort(key=lambda x: x[1], reverse=True)
                current_text = " ".join([t[0] for t in texts])
                current_conf = texts[0][1]
                ocr_history.append((current_text, current_conf))
                if len(ocr_history) > RESULT_HISTORY_SIZE:
                    ocr_history.pop(0)

                consensus = get_consensus_result(ocr_history)
                if consensus:
                    rospy.loginfo("[OCR检测] 共识文本: %s", consensus)
                    for wl_item in WHITELIST:
                        if text_similarity(consensus, wl_item) >= SIMILARITY_THRESHOLD:
                            matched_label = wl_item
                            rospy.loginfo("[OCR检测] 匹配到物体: %s，立即冻结映射", matched_label)

                            rospy.loginfo("[映射冻结] 文本标签已确认，冻结映射")

                            if last_complete_order and last_complete_label_to_action:
                                rospy.loginfo("[映射冻结] 使用最近一次完整检测的映射")
                                rospy.loginfo("[映射冻结] 完整顺序（左→右）: %s",
                                             [YOLO_TO_CHINESE.get(l, l) for l in last_complete_order])
                                rospy.loginfo("[映射冻结] 最终映射关系:")
                                for ch, act in last_complete_label_to_action.items():
                                    rospy.loginfo("  %s → %s", ch, act.__name__)
                                return last_complete_order, last_complete_label_to_action

                            elif current_detected_order:
                                rospy.logwarn("[映射冻结] 没有完整检测记录，使用当前检测结果: %s",
                                             [YOLO_TO_CHINESE.get(l, l) for l in current_detected_order])
                                chinese_labels = [YOLO_TO_CHINESE.get(l, l) for l in current_detected_order]
                                label_to_action = {}
                                for i, ch in enumerate(chinese_labels):
                                    if i == 0:
                                        label_to_action[ch] = action_functions[1]
                                    elif i == 1:
                                        label_to_action[ch] = action_functions[0]
                                    elif i == 2:
                                        label_to_action[ch] = action_functions[2]
                                return current_detected_order, label_to_action

                            else:
                                rospy.logwarn("[映射冻结] 使用默认顺序")
                                default_order = ["cube", "sphere", "cylinder"]
                                chinese_labels = [YOLO_TO_CHINESE.get(l, l) for l in default_order]
                                label_to_action = {}
                                for i, ch in enumerate(chinese_labels):
                                    if i == 0:
                                        label_to_action[ch] = action_functions[1]
                                    elif i == 1:
                                        label_to_action[ch] = action_functions[0]
                                    elif i == 2:
                                        label_to_action[ch] = action_functions[2]
                                return default_order, label_to_action

        rospy.sleep(0.05)

    return None, None


def process_last_waypoint(model, reader, action_functions):
    init_stop_subscriber()
    subscriber = ROSImageSubscriber()
    rospy.sleep(0.5)

    round_idx = 0

    try:
        rospy.loginfo("====== 阶段1: 实时检测物体，等待文本标签确认 ======")
        complete_order, label_to_action = detect_objects_and_build_mapping(
            model, reader, subscriber, action_functions
        )

        if should_stop():
            rospy.logwarn("映射建立过程中收到停止信号")
            return

        if complete_order is None or label_to_action is None:
            rospy.logerr("未能建立有效的物体映射，退出")
            return

        rospy.loginfo("====== 映射已冻结，开始循环导航 ======")

        while not should_stop():
            round_idx += 1
            rospy.loginfo("====== 第 %d 轮 OCR 识别-导航循环开始 ======", round_idx)

            ocr_history = []
            last_ocr_time = 0.0
            last_detection_box = None
            has_executed = False
            matched_label = None

            while not should_stop():
                frame = subscriber.get_frame()
                if frame is None:
                    rospy.sleep(0.05)
                    continue

                results = model.predict(frame, verbose=False)
                text_boxes = detect_text_boxes(results[0])
                now = time.monotonic()

                if text_boxes and (now - last_ocr_time) >= 1.5:
                    texts, last_detection_box, did_ocr = detect_and_recognize_text(
                        reader,
                        frame,
                        text_boxes,
                        last_detection_box=last_detection_box,
                        last_ocr_time=last_ocr_time,
                    )
                    if did_ocr:
                        last_ocr_time = now
                    if texts:
                        texts.sort(key=lambda x: x[1], reverse=True)
                        current_text = " ".join([t[0] for t in texts])
                        current_conf = texts[0][1]
                        ocr_history.append((current_text, current_conf))
                        if len(ocr_history) > RESULT_HISTORY_SIZE:
                            ocr_history.pop(0)

                        consensus = get_consensus_result(ocr_history)
                        if consensus:
                            rospy.loginfo("[第%d轮] OCR共识文本: %s", round_idx, consensus)
                            for wl_item in WHITELIST:
                                if text_similarity(consensus, wl_item) >= SIMILARITY_THRESHOLD:
                                    matched_label = wl_item
                                    rospy.loginfo("[第%d轮] 匹配到物体: %s，立即执行动作",
                                                  round_idx, matched_label)
                                    break

                if matched_label and not has_executed:
                    rospy.loginfo("[第%d轮] 执行动作: %s", round_idx, matched_label)
                    if matched_label in label_to_action:
                        try:
                            label_to_action[matched_label]()
                            rospy.loginfo("[第%d轮] 导航动作执行完成", round_idx)
                        except Exception as e:
                            rospy.logerr("[第%d轮] 导航动作异常: %s", round_idx, e)
                    else:
                        rospy.logwarn("[第%d轮] 无动作映射: %s", round_idx, matched_label)

                    if matched_label in TEXT_TO_SPEECH:
                        speak_fn = TEXT_TO_SPEECH[matched_label]
                        rospy.loginfo("[第%d轮] 播报语音: %s", round_idx, matched_label)
                        for _ in range(3):
                            if should_stop():
                                break
                            speak_fn()
                            time.sleep(1.4)
                    else:
                        rospy.logwarn("[第%d轮] 无语音映射: %s", round_idx, matched_label)

                    has_executed = True
                    break

                rospy.sleep(0.05)

            if should_stop():
                break

            rospy.loginfo("[第%d轮] 完成，等待环境稳定后进入下一轮 ...", round_idx)
            stable_until = time.monotonic() + 1.5
            while not should_stop() and time.monotonic() < stable_until:
                subscriber.get_frame()
                rospy.sleep(0.05)

    except Exception as e:
        rospy.logerr("最后路径点任务异常: %s", e)
        rospy.logerr(traceback.format_exc())
    finally:
        subscriber.shutdown()
        if rospy.is_shutdown():
            rospy.loginfo("ROS 已关闭（Ctrl+C），最后路径点任务退出")
        rospy.loginfo("最后路径点任务结束（共完成 %d 轮）", round_idx)
#!/usr/bin/env python3
import os
import sys
import time
import math
import subprocess
import threading
from pathlib import Path
from functools import lru_cache

# ========== OpenCV/GUI环境检查（简化版）==========
def _has_gui(cv2_mod):
    """检查OpenCV编译是否包含GUI支持"""
    info = cv2_mod.getBuildInformation()
    return any(
        line.strip().startswith("GUI:") and "NO" not in line.upper() and "NONE" not in line.upper()
        for line in info.splitlines()
    )

def ensure_opencv_runtime():
    """确保导入的OpenCV版本支持GUI，否则尝试切换Python解释器"""
    try:
        import cv2
        if _has_gui(cv2):
            return
    except Exception:
        pass

    # 尝试使用预置的带GUI的环境
    candidates = [
        "/home/jetson/yolov5_env/bin/python",
        "/home/jetson/miniconda3/envs/car1/bin/python",
    ]
    for candidate in candidates:
        if not (os.path.isfile(candidate) and os.access(candidate, os.X_OK)):
            continue
        # 探测候选解释器是否具有完整环境
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

ensure_opencv_runtime()

import cv2
import easyocr
import rospy
import tf
import actionlib
from ultralytics import YOLO
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dynamic_reconfigure.client import Client as DynClient
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

# ========== 参数配置 ==========
MODEL_PATH = Path("best.pt")
ROS_IMAGE_TOPIC = "/usb_cam/image_raw"
OCR_LANGS = ["ch_sim", "en"]
OCR_MIN_CONFIDENCE = 0.35
TEXT_CLASS_NAME = "text"
OBJECT_CLASSES = ["cube", "sphere", "cylinder"]  # 物体类别

# OCR结果稳定性参数
RESULT_HISTORY_SIZE = 5
CONFIDENCE_THRESHOLD = 0.5
SIMILARITY_THRESHOLD = 0.7
STABILITY_THRESHOLD = 0.6
BOX_CHANGE_THRESHOLD = 30

# 路径点相关
YAW_TOL_HIGH = math.pi
YAW_TOL_LOW = 0.1
GOAL_TIMEOUT = rospy.Duration(10.0)

# 目标物体白名单（顺序固定，用于文本匹配）
WHITELIST = ["正方体", "圆柱体", "球体"]

# YOLO标签与中文名称互转
YOLO_TO_CHINESE = {
    "cube": "正方体",
    "sphere": "球体",
    "cylinder": "圆柱体"
}
CHINESE_TO_YOLO = {v: k for k, v in YOLO_TO_CHINESE.items()}

# 语音播报函数（直接从 TTL 导入，避免二次包装）
from TTL import speak_cube, speak_cylinder, speak_sphere
TEXT_TO_SPEECH = {
    "正方体": speak_cube,
    "圆柱体": speak_cylinder,
    "球体": speak_sphere,
}

# 全局 TF 监听器（单例）
_tf_listener = None
_current_yaw_tol = None
_tf_lock = threading.Lock()

# ========== 循环停止标志（Ctrl+C 与 /stop_loop topic 双通道）==========
_stop_requested = threading.Event()
_stop_sub = None
STOP_LOOP_TOPIC = "/stop_loop"

def _stop_loop_callback(msg):
    """订阅 /stop_loop (std_msgs/Bool)，收到 data=True 即请求停止循环"""
    if bool(msg.data):
        rospy.logwarn("收到 %s 停止信号，准备退出循环识别-导航流程", STOP_LOOP_TOPIC)
        _stop_requested.set()

def init_stop_subscriber():
    """初始化 /stop_loop 订阅器（仅一次）"""
    global _stop_sub
    if _stop_sub is None:
        _stop_sub = rospy.Subscriber(STOP_LOOP_TOPIC, Bool, _stop_loop_callback, queue_size=1)
        rospy.loginfo("已订阅停止话题: %s (发送 std_msgs/Bool data:true 可停止循环)", STOP_LOOP_TOPIC)

def should_stop():
    """循环退出条件：Ctrl+C 关闭 ROS 或 /stop_loop 收到 True"""
    return rospy.is_shutdown() or _stop_requested.is_set()

# ========== 图像订阅器（线程安全优化）==========
class ROSImageSubscriber:
    """持续接收ROS图像话题的最新帧"""
    def __init__(self, topic_name=ROS_IMAGE_TOPIC):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame = None
        self._new_frame = False
        self.sub = rospy.Subscriber(topic_name, Image, self._callback)

    def _callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.latest_frame = frame
                self._new_frame = True
        except Exception as e:
            rospy.logerr("图像转换失败: %s", e)

    def get_frame(self):
        """非阻塞获取最新帧，若没有新帧则返回None"""
        with self.lock:
            if self._new_frame:
                self._new_frame = False
                return self.latest_frame
            return None

    def shutdown(self):
        self.sub.unregister()

def capture_ros_image(timeout=5.0):
    """从ROS话题捕获单帧图像（阻塞）"""
    try:
        img_msg = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout=timeout)
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    except rospy.ROSException:
        rospy.logerr("获取图像超时")
        return None

# ========== YOLO与OCR功能函数 ==========
def load_model():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"模型文件未找到: {MODEL_PATH}")
    return YOLO(str(MODEL_PATH))

def create_ocr_reader():
    try:
        import torch
        use_gpu = torch.cuda.is_available()
    except Exception:
        use_gpu = False
    print(f"[INFO] 加载EasyOCR (GPU={'启用' if use_gpu else '关闭'})")
    return easyocr.Reader(OCR_LANGS, gpu=use_gpu)

def detect_boxes(result, target_labels=None, min_confidence=0.35):
    """
    从YOLO结果中提取符合标签的检测框，返回列表。
    每个元素包含: box, confidence, label, x_center
    结果按置信度降序排列。
    """
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

def crop_with_padding(frame, box, padding_ratio=0.08):
    """按比例扩大裁剪区域"""
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
    """对裁剪区域进行OCR，返回满足置信度的文本列表"""
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

@lru_cache(maxsize=128)
def levenshtein_distance(s1: str, s2: str) -> int:
    """计算编辑距离（缓存以提高重复比较效率）"""
    if len(s1) < len(s2):
        return levenshtein_distance(s2, s1)
    if len(s2) == 0:
        return len(s1)
    prev_row = range(len(s2) + 1)
    for i, c1 in enumerate(s1):
        curr_row = [i + 1]
        for j, c2 in enumerate(s2):
            insertions = prev_row[j + 1] + 1
            deletions = curr_row[j] + 1
            substitutions = prev_row[j] + (c1 != c2)
            curr_row.append(min(insertions, deletions, substitutions))
        prev_row = curr_row
    return prev_row[-1]

def text_similarity(s1, s2):
    """计算两个文本的相似度（0~1）"""
    if not s1 or not s2:
        return 0.0
    max_len = max(len(s1), len(s2))
    if max_len == 0:
        return 1.0
    return 1.0 - levenshtein_distance(s1, s2) / max_len

def get_consensus_result(history):
    """根据历史OCR结果计算一致性最佳文本"""
    if not history:
        return None
    # 聚类相似文本
    clusters = {}
    for text, conf in history:
        merged = False
        for key in list(clusters.keys()):
            if text_similarity(text, key) >= SIMILARITY_THRESHOLD:
                clusters[key]["count"] += 1
                clusters[key]["total_conf"] += conf
                merged = True
                break
        if not merged:
            clusters[text] = {"count": 1, "total_conf": conf}

    # 选择综合得分最高的
    best_text, best_score = None, 0.0
    total = len(history)
    for text, data in clusters.items():
        stability = data["count"] / total
        avg_conf = data["total_conf"] / data["count"]
        score = stability * 0.6 + avg_conf * 0.4
        if score > best_score:
            best_score = score
            best_text = text
    return best_text

# ========== 导航辅助函数 ==========
def get_tf_listener():
    global _tf_listener
    with _tf_lock:
        if _tf_listener is None:
            _tf_listener = tf.TransformListener()
            rospy.sleep(0.5)  # 等待监听器就绪
        return _tf_listener

def set_yaw_tolerance(tol):
    global _current_yaw_tol
    if _current_yaw_tol == tol:
        return True
    planner_names = [
        "/move_base/TebLocalPlannerROS",
        "/move_base/DWAPlannerROS",
        "/move_base/TrajectoryPlannerROS",
    ]
    for name in planner_names:
        try:
            client = DynClient(name, timeout=2.0)
            client.update_configuration({"yaw_goal_tolerance": tol})
            rospy.loginfo("已将 %s 的 yaw_goal_tolerance 设为 %.4f", name, tol)
            _current_yaw_tol = tol
            return True
        except Exception:
            continue
    rospy.logwarn("无法通过 dynamic_reconfigure 设置 yaw_goal_tolerance")
    return False

def get_current_yaw():
    """获取机器人当前朝向的四元数（仅z和w分量）"""
    listener = get_tf_listener()
    try:
        listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))
        _, rot = listener.lookupTransform("map", "base_link", rospy.Time(0))
        return rot[2], rot[3]
    except Exception as e:
        rospy.logwarn("获取当前朝向失败，使用默认值: %s", e)
        return 0.0, 1.0

def send_goal(action_client, x, y, oz, ow):
    """发送导航目标并等待结果"""
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = oz
    goal.target_pose.pose.orientation.w = ow
    action_client.send_goal(goal)
    finished = action_client.wait_for_result(GOAL_TIMEOUT)
    if not finished:
        action_client.cancel_goal()
        rospy.logwarn("目标超时，已取消")
        return False
    return action_client.get_state() == actionlib.GoalStatus.SUCCEEDED

# ========== 视觉任务核心函数 ==========
def detect_objects_and_build_mapping(model, reader, subscriber, action_functions):
    """
    在最后一个点位实时检测物体并建立映射。
    持续检测直到同时满足两个条件：
    1) 检测到物体（cube/sphere/cylinder）
    2) 识别到文本标签（与白名单匹配）
    
    一旦检测到文本标签，立即冻结映射并返回。
    如果此时检测不完整（被文本板遮挡），使用最近一次完整检测的映射。
    
    返回: (complete_order_list, label_to_action_dict) 或 None表示失败
    """
    rospy.loginfo("=== 开始实时检测物体并建立映射 ===")
    
    # OCR历史记录
    ocr_history = []
    last_ocr_time = 0.0
    last_detection_box = None
    last_text_time = time.monotonic()
    
    # 当前的物体检测结果（持续更新直到文本确认）
    current_detected_order = None
    
    # 保存最后一次完整检测（3个物体）的映射
    last_complete_order = None
    last_complete_label_to_action = None
    
    while not should_stop():
        frame = subscriber.get_frame()
        if frame is None:
            rospy.sleep(0.05)
            continue
        
        results = model.predict(frame, verbose=False)
        now = time.monotonic()
        
        # 1) 检测物体（cube/sphere/cylinder）
        object_boxes = detect_boxes(results[0], target_labels=OBJECT_CLASSES, min_confidence=0.35)
        
        # 2) 检测文本框
        text_boxes = detect_boxes(results[0], target_labels=[TEXT_CLASS_NAME.lower()],
                                  min_confidence=OCR_MIN_CONFIDENCE)
        
        # ===== 更新物体检测结果 =====
        if object_boxes:
            # 按 X 轴位置从左到右排序
            object_boxes.sort(key=lambda d: d["x_center"])
            detected_order = [obj["label"] for obj in object_boxes]
            
            # 记录检测到的物体
            if detected_order != current_detected_order:
                current_detected_order = detected_order
                order_cn = [YOLO_TO_CHINESE.get(l, l) for l in detected_order]
                
                if len(detected_order) == 3:
                    # 检测到完整的3个物体，更新"最后完整映射"
                    chinese_labels = [YOLO_TO_CHINESE.get(l, l) for l in detected_order]
                    
                    # 创建完整映射
                    last_complete_label_to_action = {}
                    for i, ch in enumerate(chinese_labels):
                        if i == 0:
                            last_complete_label_to_action[ch] = action_functions[1]  # 最左 → 左转
                        elif i == 1:
                            last_complete_label_to_action[ch] = action_functions[0]  # 中间 → 前进
                        elif i == 2:
                            last_complete_label_to_action[ch] = action_functions[2]  # 最右 → 右转
                    
                    last_complete_order = detected_order
                    rospy.loginfo("[物体检测] 完整检测到3个物体（左→右）: %s (已保存)", order_cn)
                    rospy.loginfo("[物体检测] 当前映射关系:")
                    for ch, act in last_complete_label_to_action.items():
                        rospy.loginfo("  %s → %s", ch, act.__name__)
                else:
                    # 检测不完整（可能被文本板遮挡）
                    rospy.loginfo("[物体检测] 检测到 %d 个物体（左→右）: %s", len(detected_order), order_cn)
                    if last_complete_order:
                        rospy.loginfo("[物体检测] 检测不完整，将使用最近一次完整检测: %s", 
                                     [YOLO_TO_CHINESE.get(l, l) for l in last_complete_order])
        
        # ===== OCR文本识别 =====
        if text_boxes and (now - last_ocr_time) >= 1.5:
            best_box = text_boxes[0]
            # 判断文本框是否移动，避免重复OCR
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
                        # 匹配白名单
                        for wl_item in WHITELIST:
                            if text_similarity(consensus, wl_item) >= SIMILARITY_THRESHOLD:
                                matched_label = wl_item
                                rospy.loginfo("[OCR检测] 匹配到物体: %s，等待3秒确认...", matched_label)
                                last_text_time = now
                                
                                # 等待3秒无新文本确认
                                while not should_stop():
                                    if time.monotonic() - last_text_time >= 3.0:
                                        # 确认！冻结映射
                                        rospy.loginfo("[映射冻结] 文本标签已确认，冻结映射")
                                        
                                        # 优先使用完整检测的映射
                                        if last_complete_order and last_complete_label_to_action:
                                            rospy.loginfo("[映射冻结] 使用最近一次完整检测的映射")
                                            rospy.loginfo("[映射冻结] 完整顺序（左→右）: %s", 
                                                         [YOLO_TO_CHINESE.get(l, l) for l in last_complete_order])
                                            rospy.loginfo("[映射冻结] 最终映射关系:")
                                            for ch, act in last_complete_label_to_action.items():
                                                rospy.loginfo("  %s → %s", ch, act.__name__)
                                            return last_complete_order, last_complete_label_to_action
                                        
                                        # 如果没有完整检测记录，使用当前检测结果
                                        elif current_detected_order:
                                            rospy.logwarn("[映射冻结] 没有完整检测记录，使用当前检测结果: %s", 
                                                         [YOLO_TO_CHINESE.get(l, l) for l in current_detected_order])
                                            # 创建当前映射
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
                                        
                                        # 完全没有检测到物体
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
                                    
                                    rospy.sleep(0.1)
                                break
        
        rospy.sleep(0.05)
    
    # 被停止信号中断
    return None, None

def process_last_waypoint(model, reader, action_functions):
    """
    最后一个路径点任务：
    1) 实时检测物体并建立映射（持续更新直到文本标签确认）
    2) 映射冻结后，循环执行 [实时 OCR 识别 → 导航到对应点位 → 语音播报]
    """
    # 初始化停止订阅器
    init_stop_subscriber()
    subscriber = ROSImageSubscriber()
    rospy.sleep(0.5)  # 等待订阅就绪
    
    round_idx = 0
    
    try:
        # ===== 阶段1: 实时检测物体并建立映射 =====
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
        
        # ===== 阶段2: 循环OCR识别与导航 =====
        while not should_stop():
            round_idx += 1
            rospy.loginfo("====== 第 %d 轮 OCR 识别-导航循环开始 ======", round_idx)
            
            # 每轮重置 OCR 状态
            ocr_history = []
            last_ocr_time = 0.0
            last_detection_box = None
            last_text_time = time.monotonic()
            has_executed = False
            matched_label = None
            
            # —— 内层：实时 OCR 识别循环 ——
            while not should_stop():
                frame = subscriber.get_frame()
                if frame is None:
                    rospy.sleep(0.05)
                    continue
                
                results = model.predict(frame, verbose=False)
                text_boxes = detect_boxes(results[0], target_labels=[TEXT_CLASS_NAME.lower()],
                                          min_confidence=OCR_MIN_CONFIDENCE)
                now = time.monotonic()
                
                if text_boxes and (now - last_ocr_time) >= 1.5:
                    best_box = text_boxes[0]
                    # 判断文本框是否移动，避免重复OCR
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
                                # 匹配白名单
                                for wl_item in WHITELIST:
                                    if text_similarity(consensus, wl_item) >= SIMILARITY_THRESHOLD:
                                        matched_label = wl_item
                                        rospy.loginfo("[第%d轮] 匹配到物体: %s，3秒无新文本后执行",
                                                      round_idx, matched_label)
                                        last_text_time = now
                                        break
                
                # 执行动作逻辑（每轮只执行一次，执行完后跳出内层循环进入下一轮）
                if matched_label and not has_executed:
                    if now - last_text_time >= 3.0:
                        rospy.loginfo("[第%d轮] 3秒无新文本，执行动作: %s", round_idx, matched_label)
                        # 1) 动作（导航到对应点位）—— 使用冻结的映射
                        if matched_label in label_to_action:
                            try:
                                label_to_action[matched_label]()
                                rospy.loginfo("[第%d轮] 导航动作执行完成", round_idx)
                            except Exception as e:
                                rospy.logerr("[第%d轮] 导航动作异常: %s", round_idx, e)
                        else:
                            rospy.logwarn("[第%d轮] 无动作映射: %s", round_idx, matched_label)
                        
                        # 2) 语音播报（与动作解耦）
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
                        # 跳出内层循环，进入下一轮（在新位置继续识别）
                        break
                
                rospy.sleep(0.05)
            
            # —— 内层结束：本轮已执行动作或被停止 ——
            if should_stop():
                break
            
            # 给图像帧、TF、OCR 一点缓冲时间
            rospy.loginfo("[第%d轮] 完成，等待环境稳定后进入下一轮 ...", round_idx)
            stable_until = time.monotonic() + 1.5
            while not should_stop() and time.monotonic() < stable_until:
                # 丢弃缓冲中的旧帧
                subscriber.get_frame()
                rospy.sleep(0.05)
    
    except Exception as e:
        rospy.logerr("最后路径点任务异常: %s", e)
        import traceback
        rospy.logerr(traceback.format_exc())
    finally:
        subscriber.shutdown()
        if _stop_requested.is_set():
            rospy.loginfo("最后路径点任务已收到停止信号，正常退出")
        elif rospy.is_shutdown():
            rospy.loginfo("ROS 已关闭（Ctrl+C），最后路径点任务退出")
        rospy.loginfo("最后路径点任务结束（共完成 %d 轮）", round_idx)

# ========== 主程序 ==========
def main():
    rospy.init_node('waypoint_publisher_with_vision', anonymous=True)

    # 导入小车控制
    # from TTS_RUN import move_forward, move_left, move_right,car
    from fc_point import go_to_left_waypoint, go_to_middle_waypoint, go_to_right_waypoint

    # 加载模型
    rospy.loginfo("加载YOLO模型...")
    model = load_model()
    rospy.loginfo("加载EasyOCR...")
    reader = create_ocr_reader()

    # 连接move_base
    action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待 move_base action 服务器...")
    if not action_client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("无法连接 move_base action 服务器！")
        return
    rospy.loginfo("已连接 move_base")

    # 定义动作函数（需要在 action_client 创建之后）
    def move_forward_action():
        # move_forward()
        go_to_middle_waypoint(action_client)

    def move_left_action():
        # move_left()
        go_to_left_waypoint(action_client)

    def move_right_action():
        # move_right()
        go_to_right_waypoint(action_client)

    action_functions = [move_forward_action, move_left_action, move_right_action]

    # 路径点 (x, y, oz, ow, keep_orientation)
    waypoints = [
        (1.3077, 0.0001, -0.0052, 0.9999, False),
        (1.3077, 0.0001, -0.6975, 0.7165, True),
        (1.3987, -0.7606, 0.0075, 0.9999, False),
        (1.8880, -0.6118, 0.6870, 0.7267, False),
        (1.6597, -0.0051, 0.9999, 0.0141, False),
        (1.1910, 0.2825, 0.6558, 0.7549, False),
        (1.4101, 0.8273, 0.0207, 0.9998, True),
        
        (1.8631, 0.5052, -0.7167, 0.6974, False),
        (1.8290, 0.0106, -0.0118, 0.9999, False),
        (1.8290, 0.0106, 0.0029, 0.9999, True),
        (2.5226, 0.0001, -0.0008, 0.9999, True),
        (3.1629, 0.0001, 0.0001, 0.9999, True)
    ]

    for idx, (x, y, oz, ow, keep_ori) in enumerate(waypoints):
        is_last = (idx == len(waypoints) - 1)

        # 设置朝向容差
        if keep_ori:
            set_yaw_tolerance(YAW_TOL_LOW)
        else:
            set_yaw_tolerance(YAW_TOL_HIGH)
            oz, ow = get_current_yaw()

        rospy.loginfo("导航至 %d/%d: (%.2f, %.2f) %s",
                      idx+1, len(waypoints), x, y,
                      "保持朝向" if keep_ori else "忽略朝向")
        success = send_goal(action_client, x, y, oz, ow)

        if not success:
            rospy.logwarn("目标 %d 失败，继续下一个", idx+1)
            continue

        rospy.loginfo("目标 %d 到达 ✓", idx+1)

        # 最后一个点：执行完整流程（实时检测+建映射+循环导航）
        if is_last and success:
            rospy.loginfo("到达最后一个路径点，启动完整视觉任务")
            process_last_waypoint(model, reader, action_functions)

    rospy.loginfo("所有路径点执行完毕。")

if __name__ == '__main__':
    main()
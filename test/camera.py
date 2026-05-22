import cv2
import sys

def capture_depth_camera_frame(device_path="/dev/camera_depth", save_path="depth_frame.jpg"):
    """
    从深度相机设备捕获一帧画面并保存
    :param device_path: 相机设备路径，默认 /dev/camera_depth
    :param save_path: 保存图片路径
    """
    # 打开相机设备
    cap = cv2.VideoCapture(device_path)

    # 检查设备是否成功打开
    if not cap.isOpened():
        print(f"❌ 无法打开相机设备: {device_path}")
        print("可能原因：设备不存在、权限不足、驱动未加载")
        return

    print(f"✅ 成功打开相机设备: {device_path}")

    # 读取一帧画面
    ret, frame = cap.read()

    if ret:
        # 保存帧到文件
        cv2.imwrite(save_path, frame)
        print(f"✅ 成功捕获并保存一帧画面: {save_path}")
        
    else:
        print("❌ 读取帧失败")

    # 释放相机资源
    cap.release()

if __name__ == "__main__":
    # 运行捕获
    capture_depth_camera_frame()
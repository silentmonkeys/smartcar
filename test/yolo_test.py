from ultralytics import YOLO
import cv2

model = YOLO('/home/jetson/ultralytics/ultralytics/yolo11n.pt')  # 首次运行会自动下载

# 打开默认摄像头
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

ret, frame = cap.read()
if not ret:
    print("读取摄像头图像失败")
    cap.release()
    exit()

# 使用 YOLO 模型进行推理
results = model(frame)

# 在原图上绘制检测结果（YOLO 自动完成标注）
annotated_frame = results[0].plot()

# 保存带检测框的图像
output_path = "yolo_test.jpg"
cv2.imwrite(output_path, annotated_frame)
print(f"已保存带识别框的图片: {output_path}")

cap.release()

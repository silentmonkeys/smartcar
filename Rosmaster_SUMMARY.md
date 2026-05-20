# Rosmaster 目录功能汇总

本文档为 /home/jetson/Rosmaster 目录下主要程序与子模块的功能说明与快速使用提示，方便快速定位和二次开发。

## 项目概览
- 该项目面向 Yahboom Rosmaster 系列教育/实验型小车，包含小车底层控制库、摄像头与 WiFi 支持、自动驾驶（基于 YOLOv5 + 路跟随模型）、固件与示例笔记本。
- 运行平台：NVIDIA Jetson（含 TensorRT/pycuda、torch2trt 环境），需要相机设备、部分 GPIO 控制（LED/按键）以及对应的模型文件。

## 主要子目录与说明
- `rosmaster/`：主控脚本与服务，包含 web/控制接口、摄像头与 WiFi 辅助脚本、启动/停止脚本。
- `auto_drive/`：自动驾驶相关（数据采集、路跟随模型、YOLOv5 推理封装、数据集工具）。
- `board/`：STM32 等板级固件工程（多个子工程，如 Motor、CAN、Encoder 等），用于嵌入式固件开发与烧录。
- `Sample/`：示例 Jupyter 笔记本，演示蜂鸣、舵机、PWM、电机等功能。

## 重点脚本（摘要）
- `rosmaster/rosmaster_main.py`：主服务入口，基于 Flask（和 gevent）提供控制接口；负责串口与底层 `Rosmaster` 库交互、命令解析、摄像头/ WiFi 状态管理以及响应上位机协议。
- `rosmaster/camera_rosmaster.py`：封装摄像头（Depth/USB）的读取、配置与 JPG 编码方法，提供 `get_frame()` / `get_frame_jpg()`。
- `rosmaster/wifi_rosmaster.py`：WiFi 配网辅助（二维码识别通过摄像头读取 SSID/PASSWD），包含 LED/按键 GPIO 操作与 nmcli 调用实现配网逻辑。
- `rosmaster/start_app.sh`：启动脚本（示例：在终端中启动 `rosmaster_main.py`），常用于系统开机自启。
- `rosmaster/kill_rosmaster.sh`：查找并强杀 `rosmaster_main.py` 进程的脚本。

## 自动驾驶相关（`auto_drive/`）
- `road_following.py`：加载 TRT（TensorRT）路跟随模型（`road_following_model_trt.pth`），读取 USB 摄像头，输出转向角并控制底层 `Rosmaster` 小车接口（`set_akm_steering_angle`）。
- `yolov5_auto.py`：集成 YOLOv5 与路跟随，使用 `YoLov5TRT` 推理检测交通标志并做策略（停车、减速、转向等），同时调用路跟随模型修正方向。
- `yolov5_trt.py`：YOLOv5 的 TensorRT 封装类（加载 engine、预处理、后处理、推理接口 `infer()`）。
- `data_collection.py`：用于采集训练数据的 GUI 工具，借助 `xy_dataset.py` 保存标注图片（坐标 x,y 与类别）。
- `xy_dataset.py`：数据集管理类（保存、解析文件名得到标注、返回样本），并包含 Heatmap 生成辅助类。
- `utils.py`：图像预处理（normalize、to tensor），用于训练/推理流水线。

## 运行/依赖要点
- 依赖（高层）：Python3、OpenCV、PyTorch、torch2trt、TensorRT、pycuda、cvui、pyzbar、RPi.GPIO（或在非树莓平台上需要替代或 mock）。
- 硬件依赖：Jetson 平台、USB 或深度相机、Rosmaster 硬件（串口/底盘控制）、按键与 LED（GPIO）。
- 模型文件：`auto_drive/road_following_model_trt.pth`（用于路跟随），`auto_drive/yolov5/<device>/yolov5s.engine`（YOLO TensorRT engine）。

## 快速启动（示例）
1. 启动主服务（在 Jetson 上运行）：
   - `bash /home/jetson/Rosmaster/rosmaster/start_app.sh` 或 `python3 /home/jetson/Rosmaster/rosmaster/rosmaster_main.py`
2. 手动停止：
   - `bash /home/jetson/Rosmaster/rosmaster/kill_rosmaster.sh`
3. 启动自动驾驶示例：
   - 确保 TensorRT engine 与 `road_following_model_trt.pth` 可用，然后运行 `python3 /home/jetson/Rosmaster/auto_drive/road_following.py` 或运行 `yolov5_auto.py` 进行联合检测+控制。

## 推荐的下一步（可选）
- 在虚拟环境或 conda 环境中记录并安装依赖（生成 `requirements.txt` 或 conda env）。
- 若在非树莓/Jetson 开发机上测试，需对 `RPi.GPIO`、`Rosmaster_Lib` 做 mock 或封装替代层。
- 可将 `rosmaster_main.py` 中的部分协议解析提取成文档或接口定义，方便上位机集成。

——
文件自动生成于 /home/jetson/codedemmo/Rosmaster_SUMMARY.md

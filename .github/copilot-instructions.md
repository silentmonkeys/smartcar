# Copilot 指令

## 构建、测试、Lint
| 任务 | 命令 |
| --- | --- |
| OCR 单项测试 | `python3 test/ocr_test.py` |
| YOLO 摄像头单项测试 | `python3 test/yolo_test.py` |
| 相机快照单项测试 | `python3 test/camera_test.py` |
| Numpy 导入检查 | `python3 test/numpy_test.py` |

## 高层架构
- `car_running/` 对 Rosmaster_Lib 的 `run` 进行封装，并提供全局 `car` 实例用于运动控制。
- `view_identify/` 放置 Astra RGB 相机工具；`astra_rgb_viewer.py` 负责视频流查看，`ocr_camera.py` 载入 `view_identify/best.pt` 的本地 ultralytics YOLO 模型并渲染检测结果。
- `TTL/` 提供串口 TTS 触发封装（`ShapeVoiceTrigger`, `speak_*`），`call_tts_demo.py` 演示调用方式。

## 关键约定
- 视觉脚本共用 `ensure_opencv_runtime()`，用于切换到具备 GUI 的 Python 运行时：`/home/jetson/yolov5_env/bin/python` 或 `/home/jetson/miniconda3/envs/car1/bin/python`；这些路径需与实际环境保持一致。
- Astra 相机采集使用 V4L2 与 MJPG，按 0-4 索引探测，成功后会打印选中的索引与分辨率。
- 小车控制需要显式释放资源：使用完调用 `car.close()` 或 `del car`（见 `car_running/running.py` 与 `car_running/test.py`）。
- TTS 串口固定使用 `/dev/ttyUSB2`、115200，发送 5 字节帧 `AA 55 FF <cmd> FB` 用于形状播报（`sphere`, `cube`, `cylinder`）。

# smartcar

项目基于yahboom x3 plus开发，项目仍在开发中，功能不够完善，谨慎使用，如若有侵权，请及时联系作者删除。

## 快速启动

```bash
python main.py
```

## 项目结构

```
smartcar/
├── main.py                    # 主程序入口
├── best.pt                    # YOLO模型文件
├── fc_point.py                # 导航点配置
├── config/                    # 配置模块
│   ├── __init__.py
│   └── constants.py           # 参数配置（模型路径、阈值、类别映射等）
├── utils/                     # 工具模块
│   ├── __init__.py
│   ├── opencv_utils.py        # OpenCV GUI环境检查
│   ├── stop_signal.py         # 停止信号处理（Ctrl+C 和 ROS话题）
│   └── text_utils.py          # 文本相似度计算（Levenshtein距离、共识结果）
├── vision/                    # 视觉模块
│   ├── __init__.py
│   ├── image_subscriber.py    # ROS图像订阅器
│   ├── yolo_detector.py       # YOLO模型加载和物体检测
│   └── ocr_reader.py          # EasyOCR文本识别
├── navigation/                # 导航模块
│   ├── __init__.py
│   ├── tf_utils.py            # TF变换监听和朝向获取
│   ├── yaw_controller.py      # 动态配置yaw容差
│   └── goal_sender.py         # 发送导航目标
├── tasks/                     # 任务模块
│   ├── __init__.py
│   └── vision_task.py         # 视觉任务核心（物体检测+建映射+循环导航）
├── TTL/                       # 语音播报模块
│   ├── __init__.py
│   └── tts_demo.py            # 串口语音播报封装
├── describe/                  # 运行说明文档
│   ├── RUN.md                 # 运行命令
│   └── topic_summary.md       # 话题汇总
└── Abandon/                   # 废弃文件目录
```

## 模块说明

### config/
集中管理所有配置参数，包括：
- 模型路径和话题名称
- 检测阈值（置信度、相似度等）
- 类别映射（YOLO标签与中文名称）
- 导航参数

### utils/
通用工具函数：
- `opencv_utils.py`: 检测并切换到支持GUI的OpenCV环境
- `stop_signal.py`: 处理停止信号（支持Ctrl+C和ROS话题）
- `text_utils.py`: 文本相似度计算和OCR结果共识算法

### vision/
视觉处理相关：
- `image_subscriber.py`: ROS图像话题订阅器，支持线程安全的帧获取
- `yolo_detector.py`: YOLO模型加载和物体检测，支持按标签过滤
- `ocr_reader.py`: EasyOCR文本识别，包含裁剪和识别功能

### navigation/
ROS导航相关：
- `tf_utils.py`: TF变换监听和当前朝向获取
- `yaw_controller.py`: 通过dynamic_reconfigure动态设置yaw容差
- `goal_sender.py`: 发送导航目标到move_base

### tasks/
业务逻辑任务：
- `vision_task.py`: 视觉识别-导航循环任务，包含：
  - 实时检测物体并建立映射
  - 循环OCR识别与导航
  - 语音播报

### TTL/
语音播报模块：

| 函数             | 语音       |
| ---------------- | ---------- |
| speak_sphere()   | 这是球体   |
| speak_cube()     | 这是正方体 |
| speak_cylinder() | 这是圆柱体 |

## 运行流程

1. 启动ROS环境和move_base
2. 运行 `python main.py`
3. 小车按预设路径点导航
4. 到达最后一个路径点后，启动视觉识别任务：
   - 实时检测物体位置并建立映射
   - 识别文本标签后冻结映射
   - 循环执行：OCR识别 → 导航到对应点位 → 语音播报

## 停止方式

- **Ctrl+C**: 直接退出
- **ROS话题**: 发送 `std_msgs/Bool data:true` 到 `/stop_loop` 话题

## 注意事项

- 运行前请确保已安装依赖：OpenCV、EasyOCR、ultralytics、ROS相关包
- 模型文件 `best.pt` 需放在项目根目录
- 建议在Jetson设备上运行以获得最佳性能
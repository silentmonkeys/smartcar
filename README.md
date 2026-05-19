# smartcar

`call_tts_demo.py`为通过串口向ttyUSB2发送对应协议以达到播报相对应语音的效果，具体实现在文件夹 `TTL`下

| 函数             | 语音       | 串口内容       |
| ---------------- | ---------- | -------------- |
| speak_sphere()   | 这是球体   | AA 55 FF 3D FB |
| speak_cube()     | 这是正方体 | AA 55 FF 3E FB |
| speak_cylinder() | 这是圆柱体 | AA 55 FF 3F FB |

`test`文件夹请忽略

`car_running`文件夹下将rosmaster库二次封装，做了基本的运动控制，速度发布为0.5，自旋为0.6

| 函数    | 动作 |     | 函数           | 动作 |
| ------- | ---- | --- | -------------- | ---- |
| ahead() | 前进 |     | right()        | 右移 |
| back()  | 后退 |     | left_rotate()  | 左旋 |
| left()  | 左移 |     | right_rotate() | 右旋 |

`view_identify`该文件夹内实现了一个调用astra深度相机的画面的程序，同时实现了一个调用相机实现ocr文本识别的程序

> [!WARNING]
> ocr识别的部分在测试存在极高的延迟，等待后续上传优化中

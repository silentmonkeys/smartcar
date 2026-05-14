import easyocr
import time

# 初始化：简体中文 + 英文
print("正在加载 OCR 模型...")
reader = easyocr.Reader(['ch_sim', 'en'], gpu=True)  # Jetson 自动开 GPU

# 换成你自己的图片路径
img_path = "codedemmo/test/png/font_img.png"

start = time.time()
print("开始识别...")

# 执行 OCR
result = reader.readtext(img_path)

# 输出结果
print("\n识别结果：")
for line in result:
    print(line[1])

print(f"\n耗时：{time.time() - start:.2f}s")
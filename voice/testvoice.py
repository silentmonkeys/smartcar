import subprocess
import os
import re


def find_usb_audio_card():
    """自动检测 USB 声卡号，避免因重启导致卡号变化"""
    try:
        result = subprocess.run(["aplay", "-l"], capture_output=True, text=True)
        for line in result.stdout.split("\n"):
            # 匹配: card 0: Device [USB Audio Device], device 0: ...
            match = re.match(r'card\s+(\d+):\s*\w+\s+\[.*USB Audio.*\]', line)
            if match:
                return int(match.group(1))
    except Exception:
        pass

    # 回退：尝试 amixer 检测含 PCM 控制的卡
    for fallback in [0, 4]:
        test_result = subprocess.run(
            ["amixer", "-c", str(fallback), "scontrols"],
            capture_output=True, text=True
        )
        if "'PCM'" in test_result.stdout:
            return fallback
    return 0


CARD_NUM = find_usb_audio_card()
AUDIO_DEV = f"hw:{CARD_NUM},0"

print(f"[声音] 检测到 USB 声卡: card {CARD_NUM}, 设备 {AUDIO_DEV}", flush=True)


def set_volume(percent=90):
    subprocess.run(
        ["amixer", "-c", str(CARD_NUM), "set", "PCM", f"{percent}%"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )


def play_mp3(mp3_file):
    if not os.path.exists(mp3_file):
        print(f"文件不存在：{mp3_file}")
        return False

    set_volume(100)

    ffmpeg_proc = subprocess.Popen(
        ["ffmpeg", "-y", "-i", mp3_file, "-acodec", "pcm_s16le",
         "-ar", "44100", "-ac", "2", "-f", "s16le", "-"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    aplay_proc = subprocess.Popen(
        ["aplay", "-D", AUDIO_DEV, "-r", "44100", "-f", "S16_LE", "-c", "2"],
        stdin=ffmpeg_proc.stdout, stderr=subprocess.DEVNULL)

    ffmpeg_proc.stdout.close()
    aplay_proc.wait()
    print("播放完成", flush=True)
    return True


if __name__ == "__main__":
    play_mp3("/root/smartcar/voice/cube.mp3")

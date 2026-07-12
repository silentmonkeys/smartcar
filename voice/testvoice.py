import subprocess
import os

AUDIO_DEV = "hw:4,0"
CARD_NUM = 4  # USB声卡card号

def set_volume(percent=90):
    # 设置USB声卡PCM音量
    subprocess.run(
        ["amixer", "-c", str(CARD_NUM), "set", "PCM", f"{percent}%"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

def play_mp3(mp3_file):
    if not os.path.exists(mp3_file):
        print(f"文件不存在：{mp3_file}")
        return False
    # 播放前调高音量
    set_volume(100)

    ffmpeg_args = [
        "ffmpeg", "-y", "-i", mp3_file,
        "-acodec", "pcm_s16le", "-ar", "44100", "-ac", "2", "-f", "s16le", "-"
    ]
    aplay_args = [
        "aplay", "-D", AUDIO_DEV, "-r", "44100", "-f", "S16_LE", "-c", "2"
    ]

    ffmpeg_proc = subprocess.Popen(ffmpeg_args, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    aplay_proc = subprocess.Popen(aplay_args, stdin=ffmpeg_proc.stdout, stderr=subprocess.DEVNULL)
    ffmpeg_proc.stdout.close()
    aplay_proc.wait()
    print("播放完成")
    return True

if __name__ == "__main__":
    play_mp3("/root/smartcar/voice/cube.mp3")
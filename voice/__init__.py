"""voice 语音播放封装。"""

from pathlib import Path

from .testvoice import play_mp3


BASE_DIR = Path(__file__).resolve().parent


def speak_cube():
    return play_mp3(str(BASE_DIR / "cube.mp3"))


def speak_sphere():
    return play_mp3(str(BASE_DIR / "sphere.mp3"))


def speak_cylinder():
    return play_mp3(str(BASE_DIR / "cylinder.mp3"))


# 保留旧拼写，避免外部代码暂时断掉。
speackcube = speak_cube
speacksphere = speak_sphere
speackcylinder = speak_cylinder


__all__ = [
    "play_mp3",
    "speak_cube",
    "speak_sphere",
    "speak_cylinder",
    "speackcube",
    "speacksphere",
    "speackcylinder",
]

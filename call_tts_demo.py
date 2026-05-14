#!/usr/bin/env python3
# coding: utf-8

"""最简单的外部调用示例。

直接把你要播报的函数放到 main 里即可。
"""

from TTL import speak_cube, speak_cylinder, speak_sphere


def main():
    # 只保留一行调用，按需把这里改成 speak_cube() 或 speak_cylinder()
    speak_sphere()

    # 如果你想一次测试三个，就改成下面三行：
    # speak_sphere()
    # speak_cube()
    # speak_cylinder()


if __name__ == "__main__":
    main()
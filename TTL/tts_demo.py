#!/usr/bin/env python3
# coding: utf-8

"""TTYUSB2 语音播报串口封装与演示。"""

import argparse
import time

try:
	import serial
except ImportError as exc:  # pragma: no cover - 运行环境缺少 pyserial 时给出明确提示
	serial = None
	_serial_import_error = exc


__all__ = ["ShapeVoiceTrigger", "send_shape_trigger", "run_demo"]


class ShapeVoiceTrigger:
	"""封装串口播报触发帧。

	协议帧固定为: AA 55 FF <命令字> FB
	"""

	DEFAULT_PORT = "/dev/ttyUSB2"
	DEFAULT_BAUDRATE = 115200

	SHAPE_CODES = {
		"sphere": 0x3D,
		"cube": 0x3E,
		"cylinder": 0x3F,
		"球体": 0x3D,
		"正方体": 0x3E,
		"圆柱体": 0x3F,
	}

	def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE, timeout=1):
		if serial is None:
			raise ImportError("pyserial is required") from _serial_import_error

		self.port = port
		self.baudrate = baudrate
		self.timeout = timeout
		self._serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)

	@property
	def is_open(self):
		return self._serial.is_open

	def open(self):
		if not self._serial.is_open:
			self._serial.open()

	def close(self):
		if self._serial.is_open:
			self._serial.close()

	def __enter__(self):
		return self

	def __exit__(self, exc_type, exc, tb):
		self.close()

	def _build_frame(self, command_byte):
		return bytes([0xAA, 0x55, 0xFF, command_byte, 0xFB])

	def send_command(self, command_byte):
		frame = self._build_frame(command_byte)
		self._serial.write(frame)
		self._serial.flush()
		time.sleep(0.005)
		return frame

	def trigger_shape(self, shape_name):
		if shape_name not in self.SHAPE_CODES:
			valid = ", ".join(sorted(self.SHAPE_CODES))
			raise ValueError(f"Unsupported shape: {shape_name}. Valid values: {valid}")
		return self.send_command(self.SHAPE_CODES[shape_name])

	def trigger_sphere(self):
		return self.trigger_shape("sphere")

	def trigger_cube(self):
		return self.trigger_shape("cube")

	def trigger_cylinder(self):
		return self.trigger_shape("cylinder")


def send_shape_trigger(shape_name, port=ShapeVoiceTrigger.DEFAULT_PORT, baudrate=ShapeVoiceTrigger.DEFAULT_BAUDRATE, timeout=1):
	"""单次发送一个播报触发，适合在其他代码里直接调用。"""
	with ShapeVoiceTrigger(port=port, baudrate=baudrate, timeout=timeout) as trigger:
		return trigger.trigger_shape(shape_name)


def run_demo(shape_name=None):
	with ShapeVoiceTrigger() as trigger:
		print(f"Speech serial opened on {trigger.port} at {trigger.baudrate}")

		if shape_name:
			frame = trigger.trigger_shape(shape_name)
			print(f"Sent {shape_name}: {frame.hex(' ')}")
			return

		demo_sequence = [
			("球体", trigger.trigger_sphere),
			("正方体", trigger.trigger_cube),
			("圆柱体", trigger.trigger_cylinder),
		]

		for label, action in demo_sequence:
			frame = action()
			print(f"Sent {label}: {frame.hex(' ')}")
			time.sleep(1)


def parse_args():
	parser = argparse.ArgumentParser(description="TTS 串口触发演示")
	parser.add_argument(
		"shape",
		nargs="?",
		choices=["sphere", "cube", "cylinder", "球体", "正方体", "圆柱体"],
		help="可选：只发送某一个播报触发",
	)
	return parser.parse_args()


if __name__ == "__main__":
	args = parse_args()
	run_demo(args.shape)

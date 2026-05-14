"""TTL 串口播报封装。"""

from .tts_demo import ShapeVoiceTrigger, run_demo, send_shape_trigger



def speak_sphere():
	return send_shape_trigger("sphere")


def speak_cube():
	return send_shape_trigger("cube")


def speak_cylinder():
	return send_shape_trigger("cylinder")


__all__ = [
	"ShapeVoiceTrigger",
	"run_demo",
	"send_shape_trigger",
	"speak_sphere",
	"speak_cube",
	"speak_cylinder",
]
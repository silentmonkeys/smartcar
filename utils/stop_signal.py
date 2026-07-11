import threading
import rospy
from std_msgs.msg import Bool
from config.constants import STOP_LOOP_TOPIC


_stop_requested = threading.Event()
_stop_sub = None


def _stop_loop_callback(msg):
    if bool(msg.data):
        rospy.logwarn("收到 %s 停止信号，准备退出循环识别-导航流程", STOP_LOOP_TOPIC)
        _stop_requested.set()


def init_stop_subscriber():
    global _stop_sub
    if _stop_sub is None:
        _stop_sub = rospy.Subscriber(STOP_LOOP_TOPIC, Bool, _stop_loop_callback, queue_size=1)
        rospy.loginfo("已订阅停止话题: %s (发送 std_msgs/Bool data:true 可停止循环)", STOP_LOOP_TOPIC)


def should_stop():
    return rospy.is_shutdown() or _stop_requested.is_set()
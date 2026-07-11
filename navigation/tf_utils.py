import threading
import rospy
import tf


_tf_listener = None
_tf_lock = threading.Lock()


def get_tf_listener():
    global _tf_listener
    with _tf_lock:
        if _tf_listener is None:
            _tf_listener = tf.TransformListener()
            rospy.sleep(0.5)
        return _tf_listener


def get_current_yaw():
    listener = get_tf_listener()
    try:
        listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))
        _, rot = listener.lookupTransform("map", "base_link", rospy.Time(0))
        return rot[2], rot[3]
    except Exception as e:
        rospy.logwarn("获取当前朝向失败，使用默认值: %s", e)
        return 0.0, 1.0
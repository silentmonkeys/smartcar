import threading
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from config.constants import ROS_IMAGE_TOPIC


class ROSImageSubscriber:
    def __init__(self, topic_name=ROS_IMAGE_TOPIC):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame = None
        self._new_frame = False
        self.sub = rospy.Subscriber(topic_name, Image, self._callback)

    def _callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.latest_frame = frame
                self._new_frame = True
        except Exception as e:
            rospy.logerr("图像转换失败: %s", e)

    def get_frame(self):
        with self.lock:
            if self._new_frame:
                self._new_frame = False
                return self.latest_frame
            return None

    def shutdown(self):
        self.sub.unregister()


def capture_ros_image(timeout=5.0):
    try:
        img_msg = rospy.wait_for_message(ROS_IMAGE_TOPIC, Image, timeout=timeout)
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    except rospy.ROSException:
        rospy.logerr("获取图像超时")
        return None
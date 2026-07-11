#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from utils.opencv_utils import ensure_opencv_runtime
from config.constants import YAW_TOL_HIGH, YAW_TOL_LOW
from vision.yolo_detector import load_model
from vision.ocr_reader import create_ocr_reader
from navigation.tf_utils import get_current_yaw
from navigation.yaw_controller import set_yaw_tolerance
from navigation.goal_sender import send_goal
from tasks.vision_task import process_last_waypoint


ensure_opencv_runtime()


def main():
    rospy.init_node('waypoint_publisher_with_vision', anonymous=True)

    from fc_point import go_to_left_waypoint, go_to_middle_waypoint, go_to_right_waypoint

    rospy.loginfo("加载YOLO模型...")
    model = load_model()
    rospy.loginfo("加载EasyOCR...")
    reader = create_ocr_reader()

    action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待 move_base action 服务器...")
    if not action_client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("无法连接 move_base action 服务器！")
        return
    rospy.loginfo("已连接 move_base")

    def move_forward_action():
        go_to_middle_waypoint(action_client)

    def move_left_action():
        go_to_left_waypoint(action_client)

    def move_right_action():
        go_to_right_waypoint(action_client)

    action_functions = [move_forward_action, move_left_action, move_right_action]

    waypoints = [
        (1.3077, 0.0001, -0.0052, 0.9999, False),
        (1.3077, 0.0001, -0.6975, 0.7165, True),
        (1.3987, -0.7606, 0.0075, 0.9999, False),
        (1.8880, -0.6118, 0.6870, 0.7267, False),
        (1.6597, -0.0051, 0.9999, 0.0141, False),
        (1.1910, 0.2825, 0.6558, 0.7549, False),
        (1.4101, 0.8273, 0.0207, 0.9998, True),

        (1.8631, 0.5052, -0.7167, 0.6974, False),
        (1.8290, 0.0106, -0.0118, 0.9999, False),
        (1.8290, 0.0106, 0.0029, 0.9999, True),
        (2.5226, 0.0001, -0.0008, 0.9999, True),
        (3.1629, 0.0001, 0.0001, 0.9999, True)
    ]

    for idx, (x, y, oz, ow, keep_ori) in enumerate(waypoints):
        is_last = (idx == len(waypoints) - 1)

        if keep_ori:
            set_yaw_tolerance(YAW_TOL_LOW)
        else:
            set_yaw_tolerance(YAW_TOL_HIGH)
            oz, ow = get_current_yaw()

        rospy.loginfo("导航至 %d/%d: (%.2f, %.2f) %s",
                      idx+1, len(waypoints), x, y,
                      "保持朝向" if keep_ori else "忽略朝向")
        success = send_goal(action_client, x, y, oz, ow)

        if not success:
            rospy.logwarn("目标 %d 失败，继续下一个", idx+1)
            continue

        rospy.loginfo("目标 %d 到达 ✓", idx+1)

        if is_last and success:
            rospy.loginfo("到达最后一个路径点，启动完整视觉任务")
            process_last_waypoint(model, reader, action_functions)

    rospy.loginfo("所有路径点执行完毕。")


if __name__ == '__main__':
    main()
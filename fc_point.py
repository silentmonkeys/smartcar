#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# 导航点配置
GOAL_TIMEOUT = rospy.Duration(10.0)

def send_goal_to_move_base(action_client, x, y, oz, ow, goal_name="目标"):
    """
    通过 actionlib 发送目标到 move_base 并等待完成
    
    Args:
        action_client: move_base 的 SimpleActionClient
        x: 目标位置 x 坐标
        y: 目标位置 y 坐标
        oz: 四元数 z 分量
        ow: 四元数 w 分量
        goal_name: 目标名称，用于日志输出
    
    Returns:
        bool: 目标是否成功到达
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = oz
    goal.target_pose.pose.orientation.w = ow

    rospy.loginfo("发送 %s: (%.4f, %.4f), 朝向: (z=%.6f, w=%.6f)", 
                  goal_name, x, y, oz, ow)
    
    action_client.send_goal(goal)
    finished = action_client.wait_for_result(GOAL_TIMEOUT)
    
    if not finished:
        rospy.logwarn("%s 超时，取消当前目标", goal_name)
        action_client.cancel_goal()
        return False
    
    success = action_client.get_state() == actionlib.GoalStatus.SUCCEEDED
    if success:
        rospy.loginfo("%s 到达 ✓", goal_name)
    else:
        rospy.logwarn("%s 失败", goal_name)
    
    return success

def go_to_left_waypoint(action_client):
    """
    前往左导航点
    
    Args:
        action_client: move_base 的 SimpleActionClient
    
    Returns:
        bool: 目标是否成功到达
    """
    # 左导航点
    x = 3.515286922454834
    y = 0.5222519040107727
    oz = 0.018816358961906178
    ow = 0.9998229566455337
    
    return send_goal_to_move_base(action_client, x, y, oz, ow, "左导航点")

def go_to_middle_waypoint(action_client):
    """
    前往中导航点
    
    Args:
        action_client: move_base 的 SimpleActionClient
    
    Returns:
        bool: 目标是否成功到达
    """
    # 中导航点
    x = 3.6280109882354736
    y = -0.0030737072229385376
    oz = 0.01225676058075669
    ow = 0.9999248830887578
    
    return send_goal_to_move_base(action_client, x, y, oz, ow, "中导航点")

def go_to_right_waypoint(action_client):
    """
    前往右导航点
    
    Args:
        action_client: move_base 的 SimpleActionClient
    
    Returns:
        bool: 目标是否成功到达
    """
    # 右导航点
    x = 3.6280109882354736
    y = -0.8028337359428406
    oz = 6.34556199230947e-05
    ow = 0.9999999979866921
    
    return send_goal_to_move_base(action_client, x, y, oz, ow, "右导航点")

def main():
    """
    主函数：演示如何使用这三个导航点函数
    """
    rospy.init_node('waypoint_publisher_three_points', anonymous=True)
    
    # 连接到 move_base 的 action 服务器
    action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("等待 move_base action 服务器...")
    if not action_client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr("无法连接到 move_base action 服务器！")
        return
    rospy.loginfo("已连接 move_base。")

    # 演示调用三个导航点函数
    rospy.loginfo("=== 开始执行导航点序列 ===")
    
    go_to_left_waypoint(action_client)
    go_to_middle_waypoint(action_client)
    go_to_right_waypoint(action_client)
    
    rospy.loginfo("=== 所有导航点执行完毕 ===")

if __name__ == '__main__':
    main()
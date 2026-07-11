import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from config.constants import GOAL_TIMEOUT


def send_goal(action_client, x, y, oz, ow):
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = oz
    goal.target_pose.pose.orientation.w = ow
    action_client.send_goal(goal)
    finished = action_client.wait_for_result(rospy.Duration(GOAL_TIMEOUT))
    if not finished:
        action_client.cancel_goal()
        rospy.logwarn("目标超时，已取消")
        return False
    return action_client.get_state() == actionlib.GoalStatus.SUCCEEDED
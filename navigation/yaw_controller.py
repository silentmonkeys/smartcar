from dynamic_reconfigure.client import Client as DynClient
import rospy


_current_yaw_tol = None


def set_yaw_tolerance(tol):
    global _current_yaw_tol
    if _current_yaw_tol == tol:
        return True
    planner_names = [
        "/move_base/TebLocalPlannerROS",
        "/move_base/DWAPlannerROS",
        "/move_base/TrajectoryPlannerROS",
    ]
    for name in planner_names:
        try:
            client = DynClient(name, timeout=2.0)
            client.update_configuration({"yaw_goal_tolerance": tol})
            rospy.loginfo("已将 %s 的 yaw_goal_tolerance 设为 %.4f", name, tol)
            _current_yaw_tol = tol
            return True
        except Exception:
            continue
    rospy.logwarn("无法通过 dynamic_reconfigure 设置 yaw_goal_tolerance")
    return False
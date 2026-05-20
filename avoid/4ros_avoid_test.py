import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


RAD2DEG = 180.0 / math.pi


class LaserAvoidSimple(Node):
	def __init__(self):
		super().__init__("laser_avoid_simple")

		self.sub_laser = self.create_subscription(
			LaserScan, "/scan", self.register_scan, 10
		)
		self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)

		self.declare_parameter("linear", 0.15)
		self.declare_parameter("angular", 0.8)
		self.declare_parameter("laser_angle", 40.0)
		self.declare_parameter("response_dist", 0.55)

		self.linear = self.get_parameter("linear").value
		self.angular = self.get_parameter("angular").value
		self.laser_angle = self.get_parameter("laser_angle").value
		self.response_dist = self.get_parameter("response_dist").value

		# 三个计数参数：大于10即认为该方向有障碍
		self.front_warning = 0
		self.Left_warning = 0
		self.Right_warning = 0

		self.get_logger().info("laser avoid node started")

	def register_scan(self, scan_data: LaserScan) -> None:
		ranges = np.array(scan_data.ranges)

		self.front_warning = 0
		self.Left_warning = 0
		self.Right_warning = 0

		for i, r in enumerate(ranges):
			if not np.isfinite(r):
				continue

			angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG

			if -10.0 - self.laser_angle < angle < -10.0:
				if r < self.response_dist:
					self.Right_warning += 1

			if 10.0 < angle < 10.0 + self.laser_angle:
				if r < self.response_dist:
					self.Left_warning += 1

			if abs(angle) <= 10.0:
				if r < self.response_dist:
					self.front_warning += 1

		self.avoidance_control()

	def avoidance_control(self) -> None:
		front_blocked = self.front_warning > 10
		left_blocked = self.Left_warning > 10
		right_blocked = self.Right_warning > 10

		cmd = Twist()

		if front_blocked:
			# 前方有障碍：优先往障碍少的一侧转
			cmd.linear.x = 0.05
			if left_blocked and not right_blocked:
				cmd.angular.z = -self.angular
			elif right_blocked and not left_blocked:
				cmd.angular.z = self.angular
			else:
				cmd.angular.z = self.angular
		elif left_blocked and not right_blocked:
			cmd.linear.x = self.linear
			cmd.angular.z = -self.angular * 0.6
		elif right_blocked and not left_blocked:
			cmd.linear.x = self.linear
			cmd.angular.z = self.angular * 0.6
		elif left_blocked and right_blocked:
			cmd.linear.x = 0.05
			cmd.angular.z = self.angular
		else:
			cmd.linear.x = self.linear
			cmd.angular.z = 0.0

		self.pub_vel.publish(cmd)


def main() -> None:
	rclpy.init()
	node = LaserAvoidSimple()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()

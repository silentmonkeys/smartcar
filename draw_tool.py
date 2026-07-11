#!/usr/bin/env python3
import rospy
import sys
from geometry_msgs.msg import PointStamped


class DrawTool:
    def __init__(self):
        self.publisher = rospy.Publisher('/draw_point', PointStamped, queue_size=10)
        self.points = []
        self.lines = []
        
        rospy.init_node('draw_tool', anonymous=True)
        rospy.loginfo("=== 边线绘制工具 ===")
        rospy.loginfo("自定义话题: /draw_point")
        rospy.loginfo("不会影响move_base导航")
        rospy.loginfo("")

    def publish_point(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self.publisher.publish(msg)
        rospy.loginfo(f"已发布点: ({x:.2f}, {y:.2f})")
        return msg

    def draw_line(self, x1, y1, x2, y2):
        self.publish_point(x1, y1)
        rospy.sleep(0.1)
        self.publish_point(x2, y2)
        
        self.lines.append({
            'x1': x1, 'y1': y1,
            'x2': x2, 'y2': y2
        })
        rospy.loginfo(f"已绘制边线: ({x1:.2f},{y1:.2f}) -> ({x2:.2f},{y2:.2f})")

    def run(self):
        print("命令列表:")
        print("  line x1 y1 x2 y2   - 绘制一条边线")
        print("  point x y          - 发布单个点")
        print("  list               - 列出已绘制的边线")
        print("  clear              - 清除所有")
        print("  quit / q           - 退出")
        print("")
        
        while not rospy.is_shutdown():
            try:
                cmd = input("> ").strip()
                
                if not cmd:
                    continue
                
                parts = cmd.split()
                command = parts[0].lower()
                
                if command in ['quit', 'q']:
                    break
                
                elif command == 'line':
                    if len(parts) != 5:
                        print("用法: line x1 y1 x2 y2")
                        print("示例: line 1.0 0.0 3.0 0.0")
                        continue
                    try:
                        x1 = float(parts[1])
                        y1 = float(parts[2])
                        x2 = float(parts[3])
                        y2 = float(parts[4])
                        self.draw_line(x1, y1, x2, y2)
                    except ValueError:
                        print("请输入数字坐标")
                
                elif command == 'point':
                    if len(parts) != 3:
                        print("用法: point x y")
                        continue
                    try:
                        x = float(parts[1])
                        y = float(parts[2])
                        self.publish_point(x, y)
                    except ValueError:
                        print("请输入数字坐标")
                
                elif command == 'list':
                    print(f"已绘制 {len(self.lines)} 条边线:")
                    for i, line in enumerate(self.lines):
                        print(f"  {i+1}. ({line['x1']:.2f},{line['y1']:.2f}) -> ({line['x2']:.2f},{line['y2']:.2f})")
                
                elif command == 'clear':
                    self.lines.clear()
                    print("已清除所有边线")
                
                else:
                    print(f"未知命令: {command}")
                    print("可用命令: line, point, list, clear, quit")
            
            except EOFError:
                break
        
        rospy.loginfo("绘制工具退出")


if __name__ == '__main__':
    try:
        tool = DrawTool()
        tool.run()
    except rospy.ROSInterruptException:
        pass

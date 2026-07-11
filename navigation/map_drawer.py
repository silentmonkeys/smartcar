import rospy
import threading
import sys
from geometry_msgs.msg import PointStamped, Point, PolygonStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
import struct


class MapDrawer:
    def __init__(self, obstacle_publisher=None, topic_name='/draw_point'):
        self.click_sub = rospy.Subscriber(topic_name, PointStamped, self._click_callback)
        self.polygon_sub = rospy.Subscriber('/rviz_polygon', PolygonStamped, self._polygon_callback)
        self.draw_pub = rospy.Publisher(topic_name, PointStamped, queue_size=10)
        self.marker_pub = rospy.Publisher('/map_drawer_markers', MarkerArray, queue_size=10)
        self.obstacle_publisher = obstacle_publisher
        
        self.points = []
        self.lines = []
        self.draw_mode = 'line'
        self.next_marker_id = 0
        
        self.lock = threading.Lock()
        rospy.loginfo(f"[地图绘制器] 已启动，监听自定义话题: {topic_name}")
        rospy.loginfo("[地图绘制器] 使用 draw_tool.py 发布点来绘制边线")
        rospy.loginfo("[地图绘制器] 支持RViz Polygon工具，话题: /rviz_polygon")

    def set_draw_mode(self, mode):
        if mode in ['line', 'rectangle']:
            self.draw_mode = mode
            rospy.loginfo(f"[地图绘制器] 切换到{mode}模式")

    def _click_callback(self, msg):
        with self.lock:
            if msg.header.frame_id != 'map':
                rospy.logwarn("[地图绘制器] 点击点不在map坐标系下，忽略")
                return

            x = msg.point.x
            y = msg.point.y
            rospy.loginfo(f"[地图绘制器] 收到点: ({x:.2f}, {y:.2f})")

            if self.draw_mode == 'line':
                self.points.append((x, y))
                
                if len(self.points) >= 2:
                    start = self.points[-2]
                    end = self.points[-1]
                    self._add_line(start[0], start[1], end[0], end[1])
                    self.points.clear()
                else:
                    self._publish_marker(x, y, 'point', self.next_marker_id)
                    self.next_marker_id += 1
                    rospy.loginfo(f"[地图绘制器] 请选择第二个点...")

            elif self.draw_mode == 'rectangle':
                self.points.append((x, y))
                
                if len(self.points) >= 2:
                    x1, y1 = self.points[-2]
                    x2, y2 = self.points[-1]
                    self._add_rectangle(x1, y1, x2, y2)
                    self.points.clear()
                else:
                    self._publish_marker(x, y, 'point', self.next_marker_id)
                    self.next_marker_id += 1

    def add_point(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self._click_callback(msg)

    def _add_line(self, x1, y1, x2, y2, width=0.15):
        self.lines.append({
            'type': 'line',
            'x1': x1, 'y1': y1,
            'x2': x2, 'y2': y2,
            'width': width
        })
        
        self._publish_line_marker(x1, y1, x2, y2)
        
        if self.obstacle_publisher:
            self.obstacle_publisher.add_line(x1, y1, x2, y2, width)
            self.obstacle_publisher.publish_once()
        
        rospy.loginfo(f"[地图绘制器] 已绘制边线: ({x1:.2f},{y1:.2f}) -> ({x2:.2f},{y2:.2f})")

    def _add_rectangle(self, x1, y1, x2, y2):
        self.lines.append({
            'type': 'rectangle',
            'x1': min(x1, x2), 'y1': min(y1, y2),
            'x2': max(x1, x2), 'y2': max(y1, y2)
        })
        
        self._publish_rectangle_marker(min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
        
        if self.obstacle_publisher:
            self.obstacle_publisher.add_rectangle(min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
            self.obstacle_publisher.publish_once()
        
        rospy.loginfo(f"[地图绘制器] 已绘制矩形: ({x1:.2f},{y1:.2f}) -> ({x2:.2f},{y2:.2f})")

    def _polygon_callback(self, msg):
        with self.lock:
            if msg.header.frame_id != 'map':
                rospy.logwarn("[地图绘制器] 多边形不在map坐标系下，忽略")
                return

            polygon_points = msg.polygon.points
            if len(polygon_points) < 3:
                rospy.logwarn("[地图绘制器] 多边形至少需要3个点")
                return

            rospy.loginfo(f"[地图绘制器] 收到多边形，{len(polygon_points)} 个点")

            self._publish_polygon_marker(polygon_points)

            if self.obstacle_publisher:
                for i in range(len(polygon_points)):
                    p1 = polygon_points[i]
                    p2 = polygon_points[(i + 1) % len(polygon_points)]
                    self.obstacle_publisher.add_line(p1.x, p1.y, p2.x, p2.y, width=0.15)
                self.obstacle_publisher.publish_once()

            points_list = [(p.x, p.y) for p in polygon_points]
            self.lines.append({
                'type': 'polygon',
                'points': points_list
            })
            rospy.loginfo(f"[地图绘制器] 已绘制多边形")

    def _publish_polygon_marker(self, polygon_points):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.id = self.next_marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        for p in polygon_points:
            marker.points.append(Point(p.x, p.y, 0.05))
        
        if polygon_points:
            marker.points.append(Point(polygon_points[0].x, polygon_points[0].y, 0.05))
        
        marker.scale.x = 0.15
        
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration(0)
        
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        
        self.next_marker_id += 1

    def _publish_marker(self, x, y, marker_type, marker_id):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration(10.0)
        
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def _publish_line_marker(self, x1, y1, x2, y2):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.id = self.next_marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        p1 = Point()
        p1.x = x1; p1.y = y1; p1.z = 0.05
        p2 = Point()
        p2.x = x2; p2.y = y2; p2.z = 0.05
        
        marker.points.append(p1)
        marker.points.append(p2)
        
        marker.scale.x = 0.1
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration(0)
        
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        
        self.next_marker_id += 1

    def _publish_rectangle_marker(self, x1, y1, x2, y2):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.id = self.next_marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.points.append(Point(x1, y1, 0.0))
        marker.points.append(Point(x2, y1, 0.0))
        marker.points.append(Point(x2, y2, 0.0))
        marker.points.append(Point(x1, y2, 0.0))
        marker.points.append(Point(x1, y1, 0.0))
        
        marker.scale.x = 0.1
        
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration(0)
        
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        
        self.next_marker_id += 1

    def clear_all(self):
        with self.lock:
            marker_array = MarkerArray()
            
            for i in range(self.next_marker_id):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = rospy.Time.now()
                marker.id = i
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
            
            self.marker_pub.publish(marker_array)
            
            self.points.clear()
            self.lines.clear()
            
            if self.obstacle_publisher:
                self.obstacle_publisher.clear()
            
            rospy.loginfo("[地图绘制器] 已清除所有绘制内容")

    def get_lines(self):
        with self.lock:
            return list(self.lines)

    def save_lines_to_config(self, file_path):
        with self.lock:
            lines_config = []
            for line in self.lines:
                if line['type'] == 'line':
                    lines_config.append(
                        (line['x1'], line['y1'], line['x2'], line['y2'], line.get('width', 0.15))
                    )
                elif line['type'] == 'rectangle':
                    lines_config.append(
                        (line['x1'], line['y1'], line['x2'], line['y1'], 0.15)
                    )
                    lines_config.append(
                        (line['x2'], line['y1'], line['x2'], line['y2'], 0.15)
                    )
                    lines_config.append(
                        (line['x2'], line['y2'], line['x1'], line['y2'], 0.15)
                    )
                    lines_config.append(
                        (line['x1'], line['y2'], line['x1'], line['y1'], 0.15)
                    )
                elif line['type'] == 'polygon':
                    points = line['points']
                    for i in range(len(points)):
                        x1, y1 = points[i]
                        x2, y2 = points[(i + 1) % len(points)]
                        lines_config.append((x1, y1, x2, y2, 0.15))
            
            with open(file_path, 'w') as f:
                f.write("VIRTUAL_BOUNDARIES = [\n")
                for l in lines_config:
                    f.write(f"    ({l[0]:.4f}, {l[1]:.4f}, {l[2]:.4f}, {l[3]:.4f}, {l[4]:.2f}),\n")
                f.write("]\n")
            
            rospy.loginfo(f"[地图绘制器] 已保存 {len(lines_config)} 条边线到 {file_path}")
            return lines_config


def draw_lines_interactive():
    rospy.init_node('map_drawer', anonymous=True)
    
    obstacle_publisher = None
    
    try:
        from navigation.obstacle_publisher import VirtualLinePublisher
        obstacle_publisher = VirtualLinePublisher("/virtual_obstacles")
        obstacle_publisher.start()
        rospy.loginfo("[地图绘制器] 已连接障碍物发布器")
    except ImportError:
        rospy.logwarn("[地图绘制器] 未找到障碍物发布器，仅显示绘制结果")
    
    drawer = MapDrawer(obstacle_publisher)
    
    rospy.loginfo("=== 地图绘制器使用说明 ===")
    rospy.loginfo("方法1: 在RViz中使用 'InteractiveMarkers' 或发布话题")
    rospy.loginfo("方法2: 使用命令行发布点:")
    rospy.loginfo("  rostopic pub /draw_point geometry_msgs/PointStamped")
    rospy.loginfo("  --once -- \"header: frame_id: 'map'\" \"point: {x: 1.0, y: 0.0}\"")
    rospy.loginfo("方法3: 使用键盘输入坐标")
    rospy.loginfo("按 Ctrl+C 退出")
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


def draw_lines_with_keyboard():
    rospy.init_node('map_drawer', anonymous=True)
    
    obstacle_publisher = None
    
    try:
        from navigation.obstacle_publisher import VirtualLinePublisher
        obstacle_publisher = VirtualLinePublisher("/virtual_obstacles")
        obstacle_publisher.start()
        rospy.loginfo("[地图绘制器] 已连接障碍物发布器")
    except ImportError:
        rospy.logwarn("[地图绘制器] 未找到障碍物发布器，仅显示绘制结果")
    
    drawer = MapDrawer(obstacle_publisher)
    
    rospy.loginfo("=== 键盘输入模式 ===")
    rospy.loginfo("请依次输入两个点的坐标来绘制边线")
    rospy.loginfo("格式: x y (如: 1.0 0.5)")
    rospy.loginfo("输入 'q' 退出，输入 'c' 清除")
    rospy.loginfo("输入 's' 保存配置")
    
    while not rospy.is_shutdown():
        try:
            line = input("> ").strip()
            
            if line.lower() == 'q':
                break
            elif line.lower() == 'c':
                drawer.clear_all()
                continue
            elif line.lower() == 's':
                drawer.save_lines_to_config('config/virtual_boundaries.py')
                continue
            
            parts = line.split()
            if len(parts) != 2:
                print("格式错误，请输入: x y")
                continue
            
            try:
                x = float(parts[0])
                y = float(parts[1])
                drawer.add_point(x, y)
            except ValueError:
                print("请输入数字坐标")
                
        except EOFError:
            break
    
    rospy.loginfo("地图绘制器退出")


if __name__ == '__main__':
    if '--keyboard' in sys.argv:
        draw_lines_with_keyboard()
    else:
        draw_lines_interactive()

<<<<<<< HEAD
import rospy
import threading
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32
import struct


class VirtualLinePublisher:
    def __init__(self, topic_name="/virtual_obstacles"):
        self.publisher = rospy.Publisher(
            topic_name,
            PointCloud2,
            queue_size=10
        )
        self.polygon_publisher = rospy.Publisher(
            '/move_base/local_costmap/footprint',
            PolygonStamped,
            queue_size=10
        )
        self.obstacle_points = []
        self.publish_rate = rospy.Rate(10)
        self.is_running = False
        self.publish_thread = None

    def add_line(self, start_x, start_y, end_x, end_y, width=0.2, resolution=0.02):
        dx = end_x - start_x
        dy = end_y - start_y
        length = (dx**2 + dy**2)**0.5
        
        if length < 0.01:
            return

        nx = dx / length
        ny = dy / length

        perp_x = -ny * width / 2
        perp_y = nx * width / 2

        num_length_points = max(int(length / resolution), 2)
        num_width_points = max(int(width / resolution), 2)

        for w_i in range(num_width_points):
            w_t = w_i / (num_width_points - 1) if num_width_points > 1 else 0.5
            offset_ratio = w_t * 2 - 1
            sx = start_x + perp_x * offset_ratio
            sy = start_y + perp_y * offset_ratio
            for l_i in range(num_length_points + 1):
                l_t = l_i / num_length_points
                x = sx + dx * l_t
                y = sy + dy * l_t
                self.obstacle_points.append((x, y, 0.0))

        total_points = num_length_points * num_width_points
        rospy.loginfo(f"[虚拟边线] 添加边线: ({start_x:.2f},{start_y:.2f}) -> ({end_x:.2f},{end_y:.2f}), 宽度: {width}, 点数: {total_points}")

    def add_rectangle(self, x1, y1, x2, y2, resolution=0.05):
        min_x, max_x = sorted([x1, x2])
        min_y, max_y = sorted([y1, y2])

        for x in self._linspace(min_x, max_x, resolution):
            for y in self._linspace(min_y, max_y, resolution):
                self.obstacle_points.append((x, y, 0.0))

        rospy.loginfo(f"[虚拟边线] 添加矩形障碍物: ({x1:.2f},{y1:.2f}) -> ({x2:.2f},{y2:.2f})")

    def _linspace(self, start, end, step):
        result = []
        current = start
        while current <= end:
            result.append(current)
            current += step
        return result

    def clear(self):
        self.obstacle_points = []
        rospy.loginfo("[虚拟边线] 已清除所有虚拟障碍物")

    def _create_pointcloud(self):
        if not self.obstacle_points:
            return None

        cloud = PointCloud2()
        cloud.header.frame_id = "map"
        cloud.header.stamp = rospy.Time.now()
        cloud.width = len(self.obstacle_points)
        cloud.height = 1
        cloud.is_dense = True

        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width

        data = b""
        for x, y, z in self.obstacle_points:
            data += struct.pack('fff', x, y, z)
        cloud.data = data

        return cloud

    def _publish_loop(self):
        while self.is_running and not rospy.is_shutdown():
            cloud = self._create_pointcloud()
            if cloud:
                self.publisher.publish(cloud)
            self.publish_rate.sleep()

    def start(self):
        if self.is_running:
            return
        
        self._configure_costmap()
        
        self.is_running = True
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        rospy.loginfo("[虚拟边线] 启动虚拟障碍物发布器")

    def _configure_costmap(self):
        try:
            local_obs_sources = rospy.get_param('/move_base/local_costmap/obstacle_layer/observation_sources', '')
            global_obs_sources = rospy.get_param('/move_base/global_costmap/obstacle_layer/observation_sources', '')
            
            if 'virtual_obstacles' not in local_obs_sources:
                new_local_sources = local_obs_sources + ' virtual_obstacles' if local_obs_sources else 'virtual_obstacles'
                rospy.set_param('/move_base/local_costmap/obstacle_layer/observation_sources', new_local_sources)
                rospy.loginfo(f"[虚拟边线] 配置local_costmap: {new_local_sources}")
                
                rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/topic', '/virtual_obstacles')
                rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/data_type', 'PointCloud2')
                rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/marking', True)
                rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/clearing', False)
            
            if 'virtual_obstacles' not in global_obs_sources:
                new_global_sources = global_obs_sources + ' virtual_obstacles' if global_obs_sources else 'virtual_obstacles'
                rospy.set_param('/move_base/global_costmap/obstacle_layer/observation_sources', new_global_sources)
                rospy.loginfo(f"[虚拟边线] 配置global_costmap: {new_global_sources}")
                
                rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/topic', '/virtual_obstacles')
                rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/data_type', 'PointCloud2')
                rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/marking', True)
                rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/clearing', False)
            
            rospy.loginfo("[虚拟边线] costmap配置完成")
        except Exception as e:
            rospy.logwarn(f"[虚拟边线] 自动配置costmap失败: {e}")
            rospy.logwarn("[虚拟边线] 请手动配置costmap:")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/observation_sources \"scan virtual_obstacles\"")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/virtual_obstacles/topic /virtual_obstacles")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/virtual_obstacles/data_type PointCloud2")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/virtual_obstacles/marking true")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/virtual_obstacles/clearing false")

    def stop(self):
        self.is_running = False
        if self.publish_thread:
            self.publish_thread.join(timeout=1.0)
        rospy.loginfo("[虚拟边线] 停止虚拟障碍物发布器")

    def publish_once(self):
        cloud = self._create_pointcloud()
        if cloud:
            self.publisher.publish(cloud)


def create_virtual_boundaries(publisher, lines_config):
    for i, line in enumerate(lines_config):
        start_x, start_y, end_x, end_y, width = line
        publisher.add_line(start_x, start_y, end_x, end_y, width)
    
    rospy.loginfo(f"[虚拟边线] 已加载 {len(lines_config)} 条虚拟边线")
    publisher.publish_once()
=======
import rospy
import threading
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32
import struct


class VirtualLinePublisher:
    def __init__(self, topic_name="/virtual_obstacles"):
        self.publisher = rospy.Publisher(
            topic_name,
            PointCloud2,
            queue_size=10
        )
        self.polygon_publisher = rospy.Publisher(
            '/move_base/local_costmap/footprint',
            PolygonStamped,
            queue_size=10
        )
        self.obstacle_points = []
        self.publish_rate = rospy.Rate(10)
        self.is_running = False
        self.publish_thread = None

    def add_line(self, start_x, start_y, end_x, end_y, width=0.2, resolution=0.02):
        dx = end_x - start_x
        dy = end_y - start_y
        length = (dx**2 + dy**2)**0.5
        
        if length < 0.01:
            return

        nx = dx / length
        ny = dy / length

        perp_x = -ny * width / 2
        perp_y = nx * width / 2

        num_length_points = max(int(length / resolution), 2)
        num_width_points = max(int(width / resolution), 2)

        for w_i in range(num_width_points):
            w_t = w_i / (num_width_points - 1) if num_width_points > 1 else 0.5
            offset_ratio = w_t * 2 - 1
            sx = start_x + perp_x * offset_ratio
            sy = start_y + perp_y * offset_ratio
            for l_i in range(num_length_points + 1):
                l_t = l_i / num_length_points
                x = sx + dx * l_t
                y = sy + dy * l_t
                self.obstacle_points.append((x, y, 0.0))

        total_points = num_length_points * num_width_points
        rospy.loginfo(f"[虚拟边线] 添加边线: ({start_x:.2f},{start_y:.2f}) -> ({end_x:.2f},{end_y:.2f}), 宽度: {width}, 点数: {total_points}")

    def add_rectangle(self, x1, y1, x2, y2, resolution=0.05):
        min_x, max_x = sorted([x1, x2])
        min_y, max_y = sorted([y1, y2])

        for x in self._linspace(min_x, max_x, resolution):
            for y in self._linspace(min_y, max_y, resolution):
                self.obstacle_points.append((x, y, 0.0))

        rospy.loginfo(f"[虚拟边线] 添加矩形障碍物: ({x1:.2f},{y1:.2f}) -> ({x2:.2f},{y2:.2f})")

    def _linspace(self, start, end, step):
        result = []
        current = start
        while current <= end:
            result.append(current)
            current += step
        return result

    def clear(self):
        self.obstacle_points = []
        rospy.loginfo("[虚拟边线] 已清除所有虚拟障碍物")

    def _create_pointcloud(self):
        if not self.obstacle_points:
            return None

        cloud = PointCloud2()
        cloud.header.frame_id = "map"
        cloud.header.stamp = rospy.Time.now()
        cloud.width = len(self.obstacle_points)
        cloud.height = 1
        cloud.is_dense = True

        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width

        data = b""
        for x, y, z in self.obstacle_points:
            data += struct.pack('fff', x, y, z)
        cloud.data = data

        return cloud

    def _publish_loop(self):
        while self.is_running and not rospy.is_shutdown():
            cloud = self._create_pointcloud()
            if cloud:
                self.publisher.publish(cloud)
            self.publish_rate.sleep()

    def start(self):
        if self.is_running:
            return
        
        self._configure_costmap()
        
        self.is_running = True
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        rospy.loginfo("[虚拟边线] 启动虚拟障碍物发布器")

    def _configure_costmap(self):
        try:
            local_obs_sources = rospy.get_param('/move_base/local_costmap/obstacle_layer/observation_sources', '')
            global_obs_sources = rospy.get_param('/move_base/global_costmap/obstacle_layer/observation_sources', '')
            
            if 'virtual_obstacles' not in local_obs_sources:
                new_local_sources = local_obs_sources + ' virtual_obstacles' if local_obs_sources else 'virtual_obstacles'
                rospy.set_param('/move_base/local_costmap/obstacle_layer/observation_sources', new_local_sources)
                rospy.loginfo(f"[虚拟边线] 配置local_costmap: {new_local_sources}")
                
                rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/topic', '/virtual_obstacles')
                rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/data_type', 'PointCloud2')
                rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/marking', True)
                rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/clearing', False)
            
            if 'virtual_obstacles' not in global_obs_sources:
                new_global_sources = global_obs_sources + ' virtual_obstacles' if global_obs_sources else 'virtual_obstacles'
                rospy.set_param('/move_base/global_costmap/obstacle_layer/observation_sources', new_global_sources)
                rospy.loginfo(f"[虚拟边线] 配置global_costmap: {new_global_sources}")
                
                rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/topic', '/virtual_obstacles')
                rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/data_type', 'PointCloud2')
                rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/marking', True)
                rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/clearing', False)
            
            rospy.loginfo("[虚拟边线] costmap配置完成")
        except Exception as e:
            rospy.logwarn(f"[虚拟边线] 自动配置costmap失败: {e}")
            rospy.logwarn("[虚拟边线] 请手动配置costmap:")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/observation_sources \"scan virtual_obstacles\"")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/virtual_obstacles/topic /virtual_obstacles")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/virtual_obstacles/data_type PointCloud2")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/virtual_obstacles/marking true")
            rospy.logwarn("  rosparam set /move_base/local_costmap/obstacle_layer/virtual_obstacles/clearing false")

    def stop(self):
        self.is_running = False
        if self.publish_thread:
            self.publish_thread.join(timeout=1.0)
        rospy.loginfo("[虚拟边线] 停止虚拟障碍物发布器")

    def publish_once(self):
        cloud = self._create_pointcloud()
        if cloud:
            self.publisher.publish(cloud)


def create_virtual_boundaries(publisher, lines_config):
    for i, line in enumerate(lines_config):
        start_x, start_y, end_x, end_y, width = line
        publisher.add_line(start_x, start_y, end_x, end_y, width)
    
    rospy.loginfo(f"[虚拟边线] 已加载 {len(lines_config)} 条虚拟边线")
    publisher.publish_once()
>>>>>>> 133ba2b449166ed3f5b46c800f6c83a16cbc979b

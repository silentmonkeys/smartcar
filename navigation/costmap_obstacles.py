import rospy
import threading
import dynamic_reconfigure.client
from sensor_msgs.msg import PointCloud2, PointField
import struct


class CostmapObstacles:
    def __init__(self):
        self.obstacle_points = []
        self.lock = threading.Lock()
        self.is_running = False
        
        self.publisher = rospy.Publisher(
            '/virtual_obstacles',
            PointCloud2,
            queue_size=10
        )
        
        self._configure_costmap()
    
    def _configure_costmap(self):
        try:
            rospy.loginfo("[Costmap障碍物] 尝试配置costmap...")
            
            local_client = dynamic_reconfigure.client.Client(
                '/move_base/local_costmap/obstacle_layer',
                timeout=5.0
            )
            
            global_client = dynamic_reconfigure.client.Client(
                '/move_base/global_costmap/obstacle_layer',
                timeout=5.0
            )
            
            local_config = local_client.get_configuration()
            global_config = global_client.get_configuration()
            
            if 'observation_sources' in local_config:
                sources = local_config['observation_sources']
                if 'virtual_obstacles' not in sources:
                    new_sources = sources + ' virtual_obstacles' if sources else 'virtual_obstacles'
                    local_client.update_configuration({'observation_sources': new_sources})
                    rospy.loginfo(f"[Costmap障碍物] 更新local_costmap observation_sources: {new_sources}")
            
            if 'observation_sources' in global_config:
                sources = global_config['observation_sources']
                if 'virtual_obstacles' not in sources:
                    new_sources = sources + ' virtual_obstacles' if sources else 'virtual_obstacles'
                    global_client.update_configuration({'observation_sources': new_sources})
                    rospy.loginfo(f"[Costmap障碍物] 更新global_costmap observation_sources: {new_sources}")
            
            rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/topic', '/virtual_obstacles')
            rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/data_type', 'PointCloud2')
            rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/marking', True)
            rospy.set_param('/move_base/local_costmap/obstacle_layer/virtual_obstacles/clearing', False)
            
            rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/topic', '/virtual_obstacles')
            rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/data_type', 'PointCloud2')
            rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/marking', True)
            rospy.set_param('/move_base/global_costmap/obstacle_layer/virtual_obstacles/clearing', False)
            
            rospy.loginfo("[Costmap障碍物] costmap配置完成")
            
        except Exception as e:
            rospy.logwarn(f"[Costmap障碍物] 动态配置失败: {e}")
            rospy.logwarn("[Costmap障碍物] 尝试直接设置参数...")
            self._set_params_directly()
    
    def _set_params_directly(self):
        try:
            local_sources = rospy.get_param('/move_base/local_costmap/obstacle_layer/observation_sources', 'scan')
            global_sources = rospy.get_param('/move_base/global_costmap/obstacle_layer/observation_sources', 'scan')
            
            if 'virtual_obstacles' not in local_sources:
                rospy.set_param('/move_base/local_costmap/obstacle_layer/observation_sources', local_sources + ' virtual_obstacles')
            
            if 'virtual_obstacles' not in global_sources:
                rospy.set_param('/move_base/global_costmap/obstacle_layer/observation_sources', global_sources + ' virtual_obstacles')
            
            params = {
                'topic': '/virtual_obstacles',
                'data_type': 'PointCloud2',
                'marking': True,
                'clearing': False
            }
            
            for key, value in params.items():
                rospy.set_param(f'/move_base/local_costmap/obstacle_layer/virtual_obstacles/{key}', value)
                rospy.set_param(f'/move_base/global_costmap/obstacle_layer/virtual_obstacles/{key}', value)
            
            rospy.loginfo("[Costmap障碍物] 参数设置完成")
        except Exception as e:
            rospy.logerr(f"[Costmap障碍物] 参数设置失败: {e}")
    
    def add_line(self, x1, y1, x2, y2, width=0.3, resolution=0.05):
        dx = x2 - x1
        dy = y2 - y1
        length = (dx**2 + dy**2)**0.5
        
        if length < 0.01:
            return
        
        nx = dx / length
        ny = dy / length
        
        perp_x = -ny * width / 2
        perp_y = nx * width / 2
        
        avg_y = (y1 + y2) / 2
        if avg_y < 0:
            offset_dir = -1
            side = "下边线"
        else:
            offset_dir = 1
            side = "上边线"
        
        num_points = max(int(length / resolution), 10)
        
        new_points = []
        
        for i in range(num_points + 1):
            t = i / num_points
            x = x1 + dx * t
            y = y1 + dy * t
            
            new_points.append((x + perp_x * offset_dir, y + perp_y * offset_dir, 0.0))
        
        with self.lock:
            self.obstacle_points.extend(new_points)
        
        rospy.loginfo(f"[Costmap障碍物] 添加{side}: ({x1:.2f},{y1:.2f}) -> ({x2:.2f},{y2:.2f}), 点数: {len(new_points)}")
    
    def _create_pointcloud(self):
        with self.lock:
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
    
    def publish(self):
        cloud = self._create_pointcloud()
        if cloud:
            self.publisher.publish(cloud)
    
    def start(self):
        if self.is_running:
            return
        
        self.is_running = True
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        rospy.loginfo("[Costmap障碍物] 启动发布线程")
    
    def _publish_loop(self):
        rate = rospy.Rate(10)
        while self.is_running and not rospy.is_shutdown():
            self.publish()
            rate.sleep()
    
    def stop(self):
        self.is_running = False
        if hasattr(self, 'publish_thread') and self.publish_thread:
            self.publish_thread.join(timeout=1.0)
        rospy.loginfo("[Costmap障碍物] 停止发布")
    
    def clear(self):
        with self.lock:
            self.obstacle_points = []
        rospy.loginfo("[Costmap障碍物] 已清除所有障碍物")
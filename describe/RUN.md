~/run_docker.sh
启动ASTER + laser + yahboomcar
roslaunch yahboomcar_nav laser_usb_bringup.launch

docker ps -a

docker exec -it 编号 /bin/bash

roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=map2

map路径：~/yahboomcar_ws/src/yahboomcar_nav/maps/
cp -r /root/temp/\* ~/yahboomcar_ws/src/yahboomcar_nav/maps/

gmapping建图
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=gmapping

保存地图
rosrun map_server map_saver -f ~/yahboomcar_ws/src/yahboomcar_nav/maps/name

rrt_exploration建图
roslaunch yahboomcar_nav rrt_exploration.launch use_rviz:=false
自动保存

键盘移动
roslaunch yahboomcar_ctrl yahboom_keyboard.launch

b5d9b57a4ac5
e16553ebf884
36082cc3b06a

docker exec -it 36082cc3b06a /bin/bash

# 主程序启动（在docker容器内运行）
cd /root/smartcar
python main.py

# 停止程序（方式一：Ctrl+C）
# 停止程序（方式二：发送ROS话题）
rostopic pub /stop_loop std_msgs/Bool "data: true"
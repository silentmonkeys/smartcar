~/run_docker.sh
启动ASTER + laser + yahboomcar
roslaunch yahboomcar_nav laser_usb_bringup.launch

docker ps -a

docker exec -it 编号 /bin/bash

roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=false map:=map2

map路径：~/yahboomcar_ws/src/yahboomcar_nav/maps/

cp -r /root/temp/\* ~/yahboomcar_ws/src/yahboomcar_nav/maps/

建图
roslaunch yahboomcar_nav yahboomcar_map.launch use_rviz:=false map_type:=gmapping

保存地图
rosrun map_server map_saver -f ~/yahboomcar_ws/src/yahboomcar_nav/maps/name

键盘移动
roslaunch yahboomcar_ctrl yahboom_keyboard.launch

# gmsl_dw_camera

cd catkin_ws

# For ROS

catkin_make
rosrun gmsl_n_cameras gmsl_n_cameras_node --selector-mask=0111

# For TCP
cd catkin_ws/src/gmsl_driver/build
cmake -DDEFINE_ENABLE_TCP_SOCKET=ON ..
make

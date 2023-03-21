##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch px4 posix_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_cmd set_command; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_cmd set_mode; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_cmd send_command; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_cmd recieve_video /iris/usb_cam/image_raw; exec bash"' \

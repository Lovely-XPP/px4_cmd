##sitl_gazebo
gnome-terminal --window -e 'bash -c "sleep 1; roslaunch px4_cmd sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_cmd set_mode; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_cmd set_cmd; exec bash"' \
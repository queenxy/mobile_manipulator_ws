# gnome-terminal --tab -- sh -c ". install/setup.sh; ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true depth_module.profile:=848x480x30 rgb_camera.profile:=848x480x30"
gnome-terminal --tab -- sh -c ". install/setup.sh; ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true rgb_camera.color_profile:=1280x720x15 depth_module.depth_profile:=1280x720x15"

sleep 10

gnome-terminal --tab -- sh -c ". install/setup.sh; ros2 run object_detection service"

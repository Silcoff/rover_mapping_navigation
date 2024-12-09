#!/bin/bash

/entrypoint.sh &
sleep 5
ntpdate -q pool.ntp.org
# source /opt/ros/humble/setup.bash
# source /root/orin/ros2_zed_ws/install/setup.bash
# source /root/orin/ros_ws/install/setup.bash
# source /root/.bashrc


/bin/bash -c "source /opt/ros/humble/setup.bash && source /root/orin/ros2_zed_ws/install/setup.bash && source /root/orin/ros_ws/install/setup.bash && source /root/.bashrc && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i publish_tf:=false"

# Keep the container running
exec "$@"
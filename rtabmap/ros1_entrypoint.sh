#!/bin/bash
set -e

# Source ROS1 environment
source "/opt/ros/noetic/setup.bash"
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source "/ros1_ws/devel/setup.bash"

# Setup custom environment variables
export ROS_HOSTNAME=$(hostname)
export ROS_MASTER_URI=${ROS_MASTER_URI:-"http://localhost:11311"}

# Setup any additional environment variables from env file if it exists
if [ -f "/ros1_ws/env.sh" ]; then
    source "/ros1_ws/env.sh"
fi

# Print important environment info for debugging
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_HOSTNAME=$ROS_HOSTNAME"
echo "PYTHONPATH=$PYTHONPATH"

# roscore &
# # Wait for roscore to start
# while ! rostopic list &> /dev/null; do
#     echo "Waiting for roscore to start..."
#     sleep 1
# done
# rosrun traversability_mapping steepness_mapping_node

export ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}]: ${message}'
# roslaunch rtabmap_demos demo_stereo_outdoor.launch
rosbag play --clock /ros1_ws/src/traversability_mapping/data/stereo_outdoorB.bag -r 0.5 &
roslaunch traversability_mapping steepness_mapping.launch

# Handle special cases
case $1 in
    "bash" | "shell")
        exec bash
        ;;
    *)
        # Execute the command passed to the container
        exec "$@"
        ;;
esac

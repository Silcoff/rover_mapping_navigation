#!/bin/bash

/entrypoint.sh &
ntpdate -q pool.ntp.org
sleep 15
# source /opt/ros/humble/setup.bash
# source /root/orin/ros2_zed_ws/install/setup.bash
# source /root/orin/ros_ws/install/setup.bash
# source /root/.bashrc

ros2 topic list

/bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /root/orin/ros2_zed_ws/install/setup.bash && \
    source /root/orin/ros_ws/install/setup.bash && \
    source /root/.bashrc && \
    ros2 topic list && \
    ros2 launch rtabmap_launch rtabmap.launch.py \
        compressed:=true \
        rtabmap_args:="--delete_db_on_start" \
        rgb_topic:=/zed/zed_node/rgb/image_rect_color \
        depth_topic:=/zed/zed_node/depth/depth_registered \
        camera_info_topic:=/zed/zed_node/rgb/camera_info \
        frame_id:=zed_camera_link \
        approx_sync:=true \
        wait_imu_to_init:=true \
        imu_topic:=/zed/zed_node/imu/data \
        qos:=1 \
        topic_queue_size:=100 \
        sync_queue_size:=300 \
        rviz:=false"

# Keep the container running
exec "$@"
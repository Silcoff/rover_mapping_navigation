# FROM ros:noetic-ros-base as builder
FROM ros:noetic-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y python3-catkin-tools && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y ros-noetic-grid-map && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y ros-noetic-rtabmap && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y ros-noetic-rtabmap-ros && rm -rf /var/lib/apt/lists/*
# Install ntpdate to sync time
RUN apt-get update && apt-get install -y ntpdate && rm -rf /var/lib/apt/lists/*
# Install image processing packages
RUN apt-get update && apt-get install -y ros-noetic-image-transport-plugins && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y ros-noetic-image-proc && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y ros-noetic-stereo-image-proc && rm -rf /var/lib/apt/lists/*


# Install any additional dependencies from requirements.txt
COPY docker/ros1/requirements.txt /requirements.txt
RUN pip3 install -r /requirements.txt

# Create catkin workspace
WORKDIR /ros1_ws
COPY ros1_ws/src ./src

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin init && \
    catkin build -DCMAKE_BUILD_TYPE=Release"

# Synchronize time
RUN ntpdate -q pool.ntp.org

# Runtime stage
# FROM ros:noetic-ros-base

# # Install runtime dependencies
# RUN apt-get update && apt-get install -y \
#     python3-pip \
#     ros-noetic-roscpp-tutorials \
#     && rm -rf /var/lib/apt/lists/*

# Copy built workspace from builder stage
# COPY --from=builder /ros1_ws/devel /ros1_ws/devel
# COPY --from=builder /ros1_ws/install /ros1_ws/install

# Copy any additional runtime scripts
COPY docker/ros1/ros1_entrypoint.sh /
RUN chmod +x /ros1_entrypoint.sh

# Setup environment
ENV PYTHONPATH=/ros1_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
ENV LD_LIBRARY_PATH=/ros1_ws/devel/lib:${LD_LIBRARY_PATH}

ENTRYPOINT ["/ros1_entrypoint.sh"]
# CMD ["rosrun", "roscpp_tutorials", "talker"]

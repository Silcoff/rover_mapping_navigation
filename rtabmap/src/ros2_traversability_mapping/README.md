# a. Porting 'traversability_mapping' to ROS2 
> ###### based on: https://github.com/NorbertPap/traversability_mapping.git

### Build guide 

#### 0. Packages needed ( use appropriate ROS2 version )
```
apt-get install ros-humble-grid-map-msgs 
apt-get install ros-humble-grid-map-ros

apt-get install ros-humble-rtabmap
# or > apt-get install ros-humble-rtabmap*

apt-get install ros-humble-rtabmap_ros
```

#### 1. Build package
```
cd your_ros2_ws
git clone https://github.com/GiannisSp/ros2_traversability_mapping.git src/ros2_traversability_mapping
colcon build --symlink-install
```


#### Issue : Can't rtabmap_ros detected bu can't find rtabmap
Locate rtabmap `which rtabmap` : /opt/ros/humble/bin/rtabmap
Launch file tried to find it under /opt/ros/humble/lib/rtabmap_ros/rtabmap
Create link to the right directory 
```
sudo mkdir -p /opt/ros/humble/lib/rtabmap_ros
sudo ln -s /opt/ros/humble/bin/rtabmap /opt/ros/humble/lib/rtabmap_ros/rtabmap
```


# b. traversability_mapping
> https://github.com/NorbertPap/traversability_mapping/blob/master/README.md
> 
This package contains a node responsible for transforming the output of RTABMap (`/elevation_map` of type `GridMap`) into a Nav2 compatible format (`/steepness_map_occupancy_grid` of type `OccupancyGrid`). It also includes launch files that can be used to launch the necessary nodes to run a RealSense camera and RTABMap in either mapping or localization mode. 

## Usage

Example usage for mapping:
```sh
roslaunch traversability_mapping rtabmap_outdoors_mapping.launch
```
Example usage for localization:
```sh
roslaunch traversability_mapping rtabmap_outdoors_localization.launch
```

Parameters for the `steepness_mapping` node:

- `steepness_max`: The maximum allowable steepness for the terrain. Steepness values above this will be set to the maximum occupancy value, corresponding to a lethal obstacle. Default value is `1.0`, that is the equivalent of a 45° incline.

- `smoothing_window_size`: The size of the window used for smoothing the terrain data. It acts as a noise filter. A larger window size results in more smoothing, but also a loss of terrain information. Default value is `3`.

- `smoothing_threshold`: The threshold value that specifies the percentage of neighboring cells that have to be present in order for a cell to be considered for smoothing. Default value is `0.5`, which means 50% of the neighboring cells have to exist, otherwise the smoothed value at this cell will be `NaN`.

- `filling_window_size`: The size of the window used for filling gaps in the terrain data. This helps in creating a more continuous map by filling small gaps. Default value is `5`.

- `filling_window_stride`: The stride or step size used when moving the filling window across the terrain data. This parameter controls how frequently the filling operation is applied. Default value is `5`.

- `filling_threshold`: The threshold value that specifies the percentage of neighboring cells that have to be present inside the window in order for empty cells to be filled. Default value is `0.5`, which means 50% of the values have to be non-`NaN` for all the missing values inside the window to be filled up with the average value of the existing cells.

### Prerequisites

Ensure you have the necessary dependencies installed and sourced, i.e: `rtabmap`, `rtabmap_ros`, `grid_map`, `grid_map_ros`.

### Files
   - `launch/rtabmap_outdoors_mapping.launch`: The main launch file that includes the optimized parameters for RTABMap, the RealSense camera nodes, and the `steepness_mapping` node. On launch, it deletes the last map database file RTABMap created. On exit (Ctrl+C), RTABMap saves the map it built into a database.
   - `launch/rtabmap_outdoors_localization.launch`: This launch file is almost the same as the mapping one, except that it runs RTABMap in localization mode. It uses the database file created in the last run of RTABMap. This provides an estimated pose of the robot, while not updating the map itself.
   - `src/steepness_mapping_node.cpp`: Contains the source code for transforming the RTABMap output into a Nav2 compatible format for path planning and navigation.

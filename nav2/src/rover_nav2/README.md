# rover_nav2

This package contains a launch file that links the parameter file to the proper `nav2` launch file required to run in SLAM mode. It also includes a few rosbags used for testing.

## Usage

### Prerequisites

Ensure you have the necessary dependencies installed and sourced.

### Running the Launch File

1. **Start the rosbag:**

   In the first terminal, navigate to the `bags` directory and start the rosbag:

   ```sh
   cd bags
   ros2 bag play merged_bag.db3
   ```

2. **Launch Nav2 in SLAM mode:**

   In a second terminal, run:

   ```sh
   <!-- ros2 launch rover_nav2 nav2_launch.py -->
   ros2 launch nav2_bringup navigation_launch.py slam:=true params_file:=nav2_params.yaml
   ```

3. **Visualize with Rviz:**

   In a third terminal, run:

   ```sh
   rviz2
   ```

### Files
   - `launch/nav2_launch.py`: The main launch file that includes the necessary parameters and links to the nav2 launch file.
   - `params/nav2_params.yaml`: The parameter file used by the launch file.
   - `bags/`: Directory containing the rosbags used for testing.
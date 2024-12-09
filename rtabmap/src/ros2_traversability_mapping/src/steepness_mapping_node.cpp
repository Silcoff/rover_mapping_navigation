#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.h>
// #include <nav_msgs/msg/occupancy_grid.hpp>
#include <chrono>
#include <cmath>

class SteepnessMappingNode : public rclcpp::Node
{
public:
    SteepnessMappingNode() : Node("steepness_mapping_node")
    {
        // Declare parameters
        this->declare_parameter("steepness_max", 1.0);
        this->declare_parameter("smoothing_window_size", 3);
        this->declare_parameter("smoothing_threshold", 0.5);
        this->declare_parameter("filling_window_size", 5);
        this->declare_parameter("filling_window_stride", 5);
        this->declare_parameter("filling_threshold", 0.5);

        // Get parameters
        steepness_max_ = this->get_parameter("steepness_max").as_double();
        smoothing_window_size_ = this->get_parameter("smoothing_window_size").as_int();
        smoothing_threshold_ = this->get_parameter("smoothing_threshold").as_double();
        filling_window_size_ = this->get_parameter("filling_window_size").as_int();
        filling_window_stride_ = this->get_parameter("filling_window_stride").as_int();
        filling_threshold_ = this->get_parameter("filling_threshold").as_double();

        RCLCPP_INFO(this->get_logger(), "Steepness max: %f", steepness_max_);
        RCLCPP_INFO(this->get_logger(), "Smoothing window size: %d", smoothing_window_size_);
        RCLCPP_INFO(this->get_logger(), "Smoothing threshold: %f", smoothing_threshold_);
        RCLCPP_INFO(this->get_logger(), "Filling window size: %d", filling_window_size_);
        RCLCPP_INFO(this->get_logger(), "Filling window stride: %d", filling_window_stride_);
        RCLCPP_INFO(this->get_logger(), "Filling threshold: %f", filling_threshold_);

        // Publisher and subscriber
        publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/steepness_map", 10);
        occupancyGridPublisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/steepness_map_occupancy_grid", 10);
        subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "/rtabmap/elevation_map", 10,
            std::bind(&SteepnessMappingNode::callback, this, std::placeholders::_1));
    }
    
    void fillElevationData(const grid_map::Matrix& elevationData, grid_map::Matrix& filledElevationData) {
      // First, copy all original data
      filledElevationData = elevationData;
      
      int radius = filling_window_size_ / 2;
      // Stride through the map in steps of `filling_window_stride_`
      for (int i = 0; i < elevationData.rows(); i += filling_window_stride_) {
          for (int j = 0; j < elevationData.cols(); j += filling_window_stride_) {
              // First pass: count valid values in the window and calculate their sum
              double sum = 0.0;
              int count = 0;
              
              // Check each cell in the window
              for (int k = 0; k < filling_window_size_; ++k) {
                  for (int l = 0; l < filling_window_size_; ++l) {
                      if (i + k >= elevationData.rows() || j + l >= elevationData.cols()) {
                          continue;
                      }

                      int row = i + k;
                      int col = j + l;
                      
                      if (!std::isnan(elevationData(row, col))) {
                          sum += elevationData(row, col);
                          ++count;
                      }
                  }
              }
              
              // If we have enough valid values in the window
              if (count >= filling_threshold_ * std::pow(filling_window_size_, 2)) {
                  double average = sum / count;
                  
                  // Second pass: fill in NaN values in the window
                  for (int k = 0; k < filling_window_size_; ++k) {
                      for (int l = 0; l < filling_window_size_; ++l) {
                          int row = i + k;
                          int col = j + l;
                          
                          if (row >= elevationData.rows() || col >= elevationData.cols()) {
                              continue;
                          }
                          
                          if (std::isnan(elevationData(row, col))) {
                              filledElevationData(row, col) = average;
                          }
                      }
                  }
              }
          }
      }
    }
    
    void smoothElevationData(const grid_map::Matrix& elevationData, grid_map::Matrix& smoothedElevationData) {
        // Create the Gaussian kernel
        double kernel[smoothing_window_size_][smoothing_window_size_];
        for (int i = 0; i < smoothing_window_size_; ++i) {
            for (int j = 0; j < smoothing_window_size_; ++j) {
                // Calculate Gaussian kernel values
                kernel[i][j] = std::exp(-((i - smoothing_window_size_ / 2) * (i - smoothing_window_size_ / 2) + 
                                          (j - smoothing_window_size_ / 2) * (j - smoothing_window_size_ / 2)) / 2);
            }
        }

        // Calculate the radius of the smoothing window (half the window size)
        int radius = smoothing_window_size_ / 2;

        // Loop over the matrix and apply the smoothing filter
        for (int i = radius; i < elevationData.rows() - radius; ++i) {
            for (int j = radius; j < elevationData.cols() - radius; ++j) {
                if (std::isnan(elevationData(i, j))) {
                    smoothedElevationData(i, j) = std::numeric_limits<double>::quiet_NaN();
                    continue;
                }

                double sum = 0.0;
                double normalization = 0.0;

                int count = 0;
                // Apply the Gaussian kernel to the surrounding neighbors
                for (int k = -radius; k <= radius; ++k) {
                    for (int l = -radius; l <= radius; ++l) {
                        if (std::isnan(elevationData(i + k, j + l))) {
                            continue;
                        }
                        sum += kernel[k + radius][l + radius] * elevationData(i + k, j + l);
                        normalization += kernel[k + radius][l + radius];
                        ++count;
                    }
                }

                // If we have enough valid neighbors, apply the smoothing, otherwise leave as NaN
                smoothedElevationData(i, j) = count >= smoothing_threshold_ * std::pow(smoothing_window_size_, 2)
                    ? sum / normalization
                    : std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
    
    void computeSteepness(const grid_map::Matrix& smoothedElevationData, grid_map::Matrix& steepnessData, double resolution) {
        // Loop through the elevation data, skipping the boundaries
        for (int i = 1; i < smoothedElevationData.rows() - 1; ++i) {
            for (int j = 1; j < smoothedElevationData.cols() - 1; ++j) {
                // If the center point is NaN, skip to the next point
                if (std::isnan(smoothedElevationData(i, j))) {
                    steepnessData(i, j) = std::numeric_limits<double>::quiet_NaN();
                    continue;
                }

                double dx, dy;

                // Calculate dx (gradient in the x direction)
                if (std::isnan(smoothedElevationData(i, j + 1)) && std::isnan(smoothedElevationData(i, j - 1))) {
                    dx = std::numeric_limits<double>::quiet_NaN();
                } else if (std::isnan(smoothedElevationData(i, j + 1))) {
                    dx = (smoothedElevationData(i, j) - smoothedElevationData(i, j - 1)) / resolution;
                } else if (std::isnan(smoothedElevationData(i, j - 1))) {
                    dx = (smoothedElevationData(i, j + 1) - smoothedElevationData(i, j)) / resolution;
                } else {
                    dx = (smoothedElevationData(i, j + 1) - smoothedElevationData(i, j - 1)) / (2 * resolution);
                }

                // Calculate dy (gradient in the y direction)
                if (std::isnan(smoothedElevationData(i + 1, j)) && std::isnan(smoothedElevationData(i - 1, j))) {
                    dy = std::numeric_limits<double>::quiet_NaN();
                } else if (std::isnan(smoothedElevationData(i + 1, j))) {
                    dy = (smoothedElevationData(i, j) - smoothedElevationData(i - 1, j)) / resolution;
                } else if (std::isnan(smoothedElevationData(i - 1, j))) {
                    dy = (smoothedElevationData(i + 1, j) - smoothedElevationData(i, j)) / resolution;
                } else {
                    dy = (smoothedElevationData(i + 1, j) - smoothedElevationData(i - 1, j)) / (2 * resolution);
                }

                // Compute steepness as the magnitude of the gradient
                if (!std::isnan(dx) && !std::isnan(dy)) {
                    steepnessData(i, j) = std::sqrt(dx * dx + dy * dy);
                } else {
                    steepnessData(i, j) = std::isnan(dx) ? std::abs(dy) : std::abs(dx);
                }
            }
        }
    }


    void callback(const grid_map_msgs::msg::GridMap &message)
    {
        auto start = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO(this->get_logger(), "Received a new elevation map");

        // Convert ROS message to grid_map::GridMap
        grid_map::GridMap inputMap;
        grid_map::GridMapRosConverter::fromMessage(message, inputMap);
        inputMap.setBasicLayers({});

        // Get the elevation layer
        const grid_map::Matrix& elevationData = inputMap["elevation"];

        // Fill in small gaps in the elevation data
        grid_map::Matrix filledElevationData(elevationData.rows(), elevationData.cols());
        fillElevationData(elevationData, filledElevationData);
        inputMap.add("filled_elevation", filledElevationData);

        // Smooth the elevation data
        grid_map::Matrix smoothedElevationData(elevationData.rows(), elevationData.cols());
        smoothedElevationData.setConstant(std::numeric_limits<double>::quiet_NaN());
        smoothElevationData(filledElevationData, smoothedElevationData);
        inputMap.add("smoothed_elevation", smoothedElevationData);

        // Compute steepness
        grid_map::Matrix steepnessData(elevationData.rows(), elevationData.cols());
        steepnessData.setConstant(std::numeric_limits<double>::quiet_NaN());
        computeSteepness(smoothedElevationData, steepnessData, inputMap.getResolution());
        inputMap.add("steepness", steepnessData);

        // Convert back to ROS message
        auto outputMsg = grid_map::GridMapRosConverter::toMessage(inputMap, {"elevation", "filled_elevation", "smoothed_elevation", "steepness"});
        if (outputMsg) {
            auto sharedMsg = std::shared_ptr<grid_map_msgs::msg::GridMap>(std::move(outputMsg));
            publisher_->publish(*sharedMsg);  // Dereference shared pointer to pass the raw message
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert GridMap to ROS message.");
        }

        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        RCLCPP_INFO(this->get_logger(), "Steepness mapping took %f seconds", elapsed.count());

        // Convert GridMap to OccupancyGrid message and send it to another topic
        start = std::chrono::high_resolution_clock::now();
        nav_msgs::msg::OccupancyGrid occupancyGrid;
        grid_map::GridMapRosConverter::toOccupancyGrid(inputMap, "steepness", 0.0, steepness_max_, occupancyGrid);
        finish = std::chrono::high_resolution_clock::now();
        elapsed = finish - start;
        RCLCPP_INFO(this->get_logger(), "OccupancyGrid conversion took %f seconds", elapsed.count());
        occupancyGridPublisher_->publish(occupancyGrid);
    }



    private:
        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancyGridPublisher_;
        rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber_;
        double steepness_max_;
        int smoothing_window_size_;
        double smoothing_threshold_;
        int filling_window_size_;
        int filling_window_stride_;
        double filling_threshold_;
    };

    int main(int argc, char** argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<SteepnessMappingNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }

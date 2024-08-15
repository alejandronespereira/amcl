#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "amcl/math.h"

class AMCLNode : public rclcpp::Node
{
  public:
    AMCLNode() : Node("sensor_subscriber_node")
    {
        // Subscribe to /map topic
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AMCLNode::mapCallback, this, std::placeholders::_1));

        // Subscribe to /scan topic
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&AMCLNode::scanCallback, this, std::placeholders::_1));

        // Subscribe to /odom topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AMCLNode::odomCallback, this, std::placeholders::_1));
    }

  private:
    // Callback function for /map topic
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received /map data");
        // Process map data here
    }

    // Callback function for /scan topic
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received /scan data");
        // Process scan data here
    }

    // Callback function for /odom topic
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received /odom data");
        // Process odometry data here
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AMCLNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
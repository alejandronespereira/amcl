#pragma once

#include <Eigen/Core>

#include <nav_msgs/msg/occupancy_grid.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "amcl/math.h"

namespace amcl
{

enum class CellValue : int8_t
{
    OCCUPIED = 100,
    FREE_SPACE = 0,
    UNKNOWN = -1
};

class OccupancyGrid2D
{
  public:
    // Constructor from a nav_msgs::msg::OccupancyGrid message
    OccupancyGrid2D(const nav_msgs::msg::OccupancyGrid &ros_grid);

    // Constructor from a sensor_msgs::msg::LaserScan message
    OccupancyGrid2D(const sensor_msgs::msg::LaserScan &ros_scan, double resolution);

    size_t width() const;

    size_t height() const;

    void writeMapToFile(const std::string &filePath) const;

    int8_t getCellValue(const Eigen::Vector2d &cellCoordinates) const;

    int8_t getCellValueAtLocalCoordinates(const Eigen::Vector2d &coordinates) const;

    Eigen::Vector2d getLocalCoordinatesForCell(const Eigen::Vector2d &coordinates) const;

  private:
    bool isValidCell(int x, int y) const;
    Eigen::Matrix<double, 3, 3> m_transform; /** Coordinates to cell transformation */
    Eigen::Matrix<double, 3, 3> m_inverse;   /** Cell to local coordinates transform */
    size_t m_width;
    size_t m_height;
    float m_resolution;
    std::vector<std::vector<int8_t>> m_grid;

    Eigen::Matrix<double, 3, 3> computeCoordinatesToCellTransform(double resolution, double cx, double cy,
                                                                  double yaw) const;
};

}  // namespace amcl

#include "amcl/occupancy_grid_2d.hpp"
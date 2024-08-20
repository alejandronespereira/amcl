#pragma once
#include <math.h>
#include <algorithm>
#include <cmath>
#include <fstream>

#include <geometry_msgs/msg/quaternion.hpp>

#include "occupancy_grid_2d.h"

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quat)
{
    // Convert quaternion to yaw angle
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Matrix<double, 3, 3> amcl::OccupancyGrid2D::computeCoordinatesToCellTransform(double resolution, double cx,
                                                                                     double cy, double yaw) const
{
    /**
     * resolution is in meters/cell, as given by the ros occupancy grid message
     */

    Eigen::Matrix<double, 3, 3> scaleMatrix = Eigen::Matrix<double, 3, 3>::Identity();
    scaleMatrix(0, 0) = 1 / resolution;
    scaleMatrix(1, 1) = 1 / resolution;

    auto M = amcl::math::Transform2D<double>(Eigen::Matrix<double, 2, 1>(cx, cy), Eigen::Rotation2D<double>(yaw));

    Eigen::Matrix<double, 3, 3> ret = M.homogeneous() * scaleMatrix;
    return ret;
}

amcl::OccupancyGrid2D::OccupancyGrid2D(const nav_msgs::msg::OccupancyGrid &ros_grid)
{
    this->m_width = ros_grid.info.width;
    this->m_height = ros_grid.info.height;
    this->m_resolution = ros_grid.info.resolution;

    // Resize the internal grid to match the dimensions from the ROS message
    this->m_grid.resize(this->m_width, std::vector<int8_t>(this->m_height, -1));

    this->m_transform = this->computeCoordinatesToCellTransform(this->m_resolution, ros_grid.info.origin.position.x,
                                                                ros_grid.info.origin.position.y,
                                                                getYawFromQuaternion(ros_grid.info.origin.orientation));
    this->m_inverse = this->m_transform.inverse();

    // Fill the grid with data from the ROS message
    for (size_t y = 0; y < this->m_height; ++y)
    {
        for (size_t x = 0; x < this->m_width; ++x)
        {
            size_t index = y * this->m_width + x;
            this->m_grid[x][y] = int8_t(ros_grid.data[index]);
        }
    }
}

amcl::OccupancyGrid2D::OccupancyGrid2D(const sensor_msgs::msg::LaserScan &ros_scan, double resolution)
{
    double max_range = *std::max_element(ros_scan.ranges.begin(), ros_scan.ranges.end(), [](double a, double b) {
        if (!std::isfinite(a))
        {
            return true;
        }
        if (!std::isfinite(b))
        {
            return false;
        }
        return a < b;
    });

    this->m_width = static_cast<size_t>(floor((2.5 * max_range / resolution) / 2) * 2 + 1);
    this->m_height = this->m_width;
    this->m_resolution = resolution;

    // Scans are always XY aligned
    double yaw = 0;

    this->m_grid.resize(this->m_width, std::vector<int8_t>(this->m_height, -1));

    auto x = this->m_width / 2.0;
    auto y = this->m_height / 2.0;

    this->m_transform = this->computeCoordinatesToCellTransform(this->m_resolution, x, y, 0.0);
    this->m_inverse = this->m_transform.inverse();

    for (size_t row = 0; row < this->m_height; row++)
    {
        for (size_t col = 0; col < this->m_width; col++)
        {
            // Get the cell position in meters wrt the scan pose (0,0,0)
            Eigen::Vector2d cellCoordinates(col, row);

            auto p = this->getLocalCoordinatesForCell(cellCoordinates);
            auto angle = atan2(p.y(), p.x());
            double radius = p.head<2>().norm();

            // Get the closest angle from the scan ranges
            int angleIndex = int((angle - ros_scan.angle_min) / ros_scan.angle_increment);
            double range = ros_scan.ranges[angleIndex];

            // If there is not a range reading in this angle, skip:
            if (std::isnan(range) || std::isinf(range))
            {
                continue;
            }
            // Mark the cell as occupied or unoccupied if it is inside the range
            if (std::abs(radius - ros_scan.ranges[angleIndex]) <= resolution)
            {
                this->m_grid[col][row] = static_cast<int8_t>(amcl::CellValue::OCCUPIED);
            }
            else if (radius < ros_scan.ranges[angleIndex])
            {
                this->m_grid[col][row] = static_cast<int8_t>(amcl::CellValue::FREE_SPACE);
            }
        }
    }
}

int8_t amcl::OccupancyGrid2D::getCellValue(const Eigen::Vector2d &cellCoordinates) const
{
    int x = static_cast<int>(cellCoordinates.x());
    int y = static_cast<int>(cellCoordinates.y());

    if (!isValidCell(x, y))
    {
        return -1;
    }
    else
    {
        return this->m_grid[x][y];
    }
}

int8_t amcl::OccupancyGrid2D::getCellValueAtLocalCoordinates(const Eigen::Vector2d &coordinates) const
{
    auto cellCoordinates = this->m_transform * coordinates.homogeneous();
    return this->getCellValue(cellCoordinates.hnormalized());
}

Eigen::Vector2d amcl::OccupancyGrid2D::getLocalCoordinatesForCell(const Eigen::Vector2d &cellCoordinates) const
{
    return (this->m_inverse * cellCoordinates.homogeneous()).hnormalized();
}

bool amcl::OccupancyGrid2D::isValidCell(int x, int y) const
{
    return x >= 0 && x < int(this->m_width) && y >= 0 && y < int(this->m_height);
}

size_t amcl::OccupancyGrid2D::width() const { return this->m_width; }

size_t amcl::OccupancyGrid2D::height() const { return this->m_height; };

void amcl::OccupancyGrid2D::writeMapToFile(const std::string &filePath) const
{
    std::ofstream outfile(filePath);

    if (!outfile)
    {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    std::map<int8_t, std::string> intCharMap;
    intCharMap[static_cast<int>(amcl::CellValue::FREE_SPACE)] = "██";
    intCharMap[static_cast<int>(amcl::CellValue::UNKNOWN)] = "  ";
    intCharMap[static_cast<int>(amcl::CellValue::OCCUPIED)] = "▒▒";

    // Write the matrix to the file
    for (size_t r = 0; r < this->m_height; r++)
    {
        size_t row = this->m_height - 1 - r;
        for (size_t col = 0; col < this->m_width; col++)
        {
            auto elem = this->m_grid[col][row];

            outfile << intCharMap[elem];
        }
        outfile << std::endl;
    }
    outfile.close();
}

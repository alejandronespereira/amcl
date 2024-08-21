#pragma once

#include "amcl/particle.h"

amcl::Particle::Particle(const amcl::math::Transform2D<double>& pose, std::shared_ptr<OccupancyGrid2D> grid)
    : m_pose(pose), m_grid(grid), m_score(0.0)
{
}

double& amcl::Particle::score() { return this->m_score; }

void amcl::Particle::compareAgainstMap(const amcl::OccupancyGrid2D& map)
{
    size_t width = this->m_grid->width();
    size_t height = this->m_grid->height();

    for (size_t col = 0; col < width; col++)
    {
        for (size_t row = 0; row < width; row++)
        {
            auto scanCellCoordinates = Eigen::Vector2d(col, row);
            auto scanValue = map.getCellValue(scanCellCoordinates);

            auto scanCoordinates = map.getLocalCoordinatesForCell(scanCellCoordinates);
            auto mapCoordinates = this->m_pose.homogeneous().inverse() * scanCoordinates.homogeneous();

            auto mapValue = map.getCellValueAtLocalCoordinates(mapCoordinates.hnormalized());

            if (scanValue == static_cast<int8_t>(amcl::CellValue::UNKNOWN))
            {
                continue;
            }
            else if (scanValue == mapValue)
            {
                this->m_score++;
            }
        }
    }
    this->m_score /= (width * height);
}
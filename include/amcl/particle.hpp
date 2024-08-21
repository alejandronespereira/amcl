#pragma once

#include <cmath>
#include <random>

#include "amcl/particle.h"

amcl::Particle::Particle(const amcl::math::Transform2D<double>& pose, const amcl::OccupancyGrid2D& grid)
    : m_pose(pose), m_grid(grid), m_score(0.0)
{
}

amcl::Particle::Particle(const amcl::Particle& other, const amcl::math::Transform2D<double>& pose)
{
    this->m_pose = this->m_pose * pose;
    this->m_grid = other.m_grid;
    this->m_score = other.m_score;
}

double& amcl::Particle::score() { return this->m_score; }

void amcl::Particle::compareAgainstMap(const amcl::OccupancyGrid2D& map)
{
    size_t width = this->m_grid.width();
    size_t height = this->m_grid.height();

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

std::vector<amcl::Particle> amcl::sampleParticles(const amcl::OccupancyGrid2D& map, const amcl::OccupancyGrid2D& scan,
                                                  int nCells)
{
    std::vector<amcl::Particle> res;
    // Get list of possible cells
    std::vector<std::pair<size_t, size_t>> freeCells = map.getFreeCells();

    // Create random engines

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> yawDistribution(-M_PI, M_PI);
    std::uniform_int_distribution<> cellDistribution(0, freeCells.size() - 1);

    for (int n = 0; n < nCells; n++)
    {
        double yaw = yawDistribution(gen);
        int cellIndex = cellDistribution(gen);

        std::pair<size_t, size_t> cell = freeCells[cellIndex];

        amcl::math::Transform2D<double> particlePose(Eigen::Vector2d(cell.first, cell.second),
                                                     Eigen::Rotation2D<double>(yaw));

        res.push_back(amcl::Particle(particlePose, scan));
    }
    return res;
}

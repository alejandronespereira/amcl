#pragma once

#include <Eigen/Core>

#include "amcl/math.h"
#include "amcl/occupancy_grid_2d.h"

namespace amcl
{

class Particle
{
    /** Particle class. Holds a pointer to the scan data and a score.
     *
     */

  public:
    Particle(const amcl::math::Transform2D<double>& pose, const amcl::OccupancyGrid2D& grid);

    // Copy constructor with a pose
    Particle(const Particle& other, const amcl::math::Transform2D<double>& pose);

    double& score();

    void compareAgainstMap(const OccupancyGrid2D& map);

  private:
    amcl::math::Transform2D<double> m_pose;
    amcl::OccupancyGrid2D m_grid;
    double m_score;
};

std::vector<Particle> sampleParticles(const OccupancyGrid2D& map, const OccupancyGrid2D& scan, int nCells);

}  // namespace amcl

#include "amcl/particle.hpp"

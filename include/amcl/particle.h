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
    Particle(const amcl::math::Transform2D<double>& pose, std::shared_ptr<OccupancyGrid2D> grid);

    double& score();

    void compareAgainstMap(const OccupancyGrid2D& map);

  private:
    amcl::math::Transform2D<double> m_pose;
    std::shared_ptr<OccupancyGrid2D> m_grid;
    double m_score;
};

}  // namespace amcl

#include "amcl/particle.hpp"

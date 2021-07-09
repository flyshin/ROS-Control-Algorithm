#pragma once

namespace swarm {
namespace utils {

static double
HeadingDirectionToGo(double pre_x, double pre_y,
                     double wp_x,  double wp_y)
{
  double dy = wp_y - pre_y;
  double dx = wp_x - pre_x;

  return atan2(dy, dx);
}

} // namespace swarm::utils
} // namespace swarm

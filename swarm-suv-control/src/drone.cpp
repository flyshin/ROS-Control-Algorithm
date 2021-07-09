#include <swarm-suv-control/drone.hpp>

namespace swarm {

Drone::Drone()
{
}

Drone::~Drone()
{
}

void
Drone::quternionToeuler(geometry_msgs::PoseStamped &state, double *currentAngle)
{
  ///Current state
  double x = state.pose.orientation.x;    
  double y = state.pose.orientation.y;
  double z = state.pose.orientation.z;
  double w = state.pose.orientation.w;

  ///Roll angle
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);

  currentAngle[0] = std::atan2(sinr_cosp, cosr_cosp);

  ///Pitch angle(y-axis rotation)
  double sinp = 2 * (w * y - z * x);

  if (std::abs(sinp) >= 1)
    currentAngle[1] = std::copysign(M_PI/2, sinp);
  else
    currentAngle[1] = std::asin(sinp);

  ///Yaw (y-axis rotation)
  double siny_cosp = 2*(w*z + x*y);
  double cosy_cosp = 1 -2*(y*y + z*z);
  currentAngle[2] = std::atan2(siny_cosp, cosy_cosp); ///yaw angle
}

void
Drone::eulerToquternion(double *qAngle,
                        double *dAngle,
                        double yaw)
{
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  
  double cp = cos(dAngle[1]*0.5);
  double sp = sin(dAngle[1]*0.5); 
  double cr = cos(dAngle[0]*0.5);
  double sr = sin(dAngle[0]*0.5);

  qAngle[0] = cy*cp*cr + sy*sp*sr;
  qAngle[1] = cy*cp*sr - sy*sp*cr;
  qAngle[2] = sy*cp*sr + cy*sp*cr;
  qAngle[3] = sy*cp*cr - cy*sp*sr;   
}

} // namespace swarm

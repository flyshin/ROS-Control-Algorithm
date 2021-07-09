#pragma once

#include <swarm-suv-control/common.hpp>
#include <swarm-suv-control/drone.hpp>
#include <swarm-suv-control/drone/follow/leader.hpp>
#include <swarm-suv-control/drone/follow/follower.hpp>

//#include <swarm-suv-control/collisionavoidance.hpp>


//!NOTE: Forwrad DECL
//class CollisionAvoidance;

namespace swarm {
namespace controller {

class Follow 
{
 public:
   Follow();
  ~Follow();
  
  
  void Spin();
 
protected:
private:
  ros::NodeHandle _nh; // have to find
  ros::Rate _rate; // have to find
  int _stationId; 

  //CollisionAvoidance *_pCa;

  std::vector<std::shared_ptr<swarm::Drone>> _vec;

//NOTE:: The step for Collision avoidance
double GetRelativeDistance(double *, double *, double *, int );
double MinValue_fuction(double * );
double CallCollisionAvoidance(double );


                       
}; // class Follow

} // namespace swarm::Drone
} // namespace swarm

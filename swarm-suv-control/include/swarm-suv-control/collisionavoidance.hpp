#ifndef COLLISIONAVOIDANCE_H__
#define COLLISIONAVOIDANCE_H__

#include <iostream>
//#include "geometry_msgs/PoseStamped.h"

//#include <swarm-suv-control/controller//follow.hpp>

class CollisionAvoidance 
{
public:
  CollisionAvoidance();
  ~CollisionAvoidance();

    
  static void   SetMinimumValue(double value) { _MinimumValue = value; }  
  static double GetMinimumValue() { return _MinimumValue; }
  

protected:
  //void quternionToeuler(geometry_msgs::PoseStamped &, double *);
 // void eulerToquternion(double *, double *, double);
  
private:
  static double _MinimumValue;
  
  //double CollisionAvoidance::_MinimumValue = 0;

}; // class Drone

double CollisionAvoidance::_MinimumValue = 0;


#endif
 // namespace swarm

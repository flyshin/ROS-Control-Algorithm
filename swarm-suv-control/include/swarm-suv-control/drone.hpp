#pragma once

#include "geometry_msgs/PoseStamped.h"

#include <swarm-suv-control/utils.hpp>

namespace swarm {

class Drone {
 public:
  Drone();
  virtual ~Drone();

  virtual void Spin() = 0; // you have to redefine 


  //!NOT:: adding
  virtual double LeaderGetStateX()  const= 0; //adding
  virtual double LeaderGetStateY() const= 0; //adding
  virtual double FollowerGetStateX() const = 0; //adding
  virtual double FollowerGetStateY() const = 0; //adding
  
 protected:
  void quternionToeuler(geometry_msgs::PoseStamped &, double *);
  void eulerToquternion(double *, double *, double);
  
 private:
}; // class Drone

} // namespace swarm

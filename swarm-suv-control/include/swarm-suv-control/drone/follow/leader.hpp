#pragma once

#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/PositionTarget.h"

#include <swarm-suv-control/drone.hpp>


namespace swarm {
namespace drone {

class Leader : public swarm::Drone
{
 public:
  //Leader();
  Leader(int stationId);  // *? where stationId is from ? 
  ~Leader();

 
  virtual double LeaderGetStateX() const;
  virtual double LeaderGetStateY() const;
  virtual double FollowerGetStateX() const; //hawk = this is why it has to keep form ,between leader and drone(parents)
  virtual double FollowerGetStateY() const; //hawk = this is why it has to keep form ,between leader and drone(parents)

 

  virtual void Spin();
 protected:
 private:
  ros::NodeHandle _nh;
  ros::Rate _rate;
  ros::Publisher _pub;
  ros::Subscriber _sub_state;


  double _leader_state_x; // drone current state x
  double _leader_state_y;
  
  int _count;
  int _mode;

  geometry_msgs::PoseStamped _current_state;
  geometry_msgs::PoseStamped _pos;
  //mavros_msgs::PositionTarget _pos;

  void sub_state(const geometry_msgs::PoseStamped::ConstPtr&);

  void waypoint_mode(int *, int &);
  void attitude_mode(int , double *);
}; // class Leader

} // namespace swarm::Drone
} // namespace swarm

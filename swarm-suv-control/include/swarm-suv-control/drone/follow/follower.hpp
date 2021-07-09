#pragma once

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h" //velocity of follower
#include "mavros_msgs/PositionTarget.h"  // adding


#include <swarm-suv-control/drone.hpp>

//#include <swarm-suv-control/collisionavoidance.hpp>


namespace swarm {
namespace drone {

//swarm::drone::Follower::
//class CollisionAvoidance;

class Follower : public swarm::Drone
{
 public:
  Follower(int stationId, int followerId);
  ~Follower();

 virtual void Spin();
//--------------------------------------------------------------------------------------------------------------------------
 //void SetMinValue(double v) { _min_dist = v; } // For obtaining the MinValue of relative distance  
 
 #if 0
 double GetStateX() const { return _follow_state_x; }
 double GetStateY() const { return _follow_state_y; }
 #endif

  virtual double FollowerGetStateX() const;                               
  virtual double FollowerGetStateY()  const;
  virtual double LeaderGetStateX() const; //hawk = this is why it has to keep form ,between follower and drone(parents)
  virtual double LeaderGetStateY() const; //hawk = this is why it has to keep form ,between follower and drone(parents)

 protected:

 private:
//----------------------ROS(Robot Operating System)-------------------------- 
  ros::NodeHandle _nh;
  ros::Rate _rate;
  ros::Publisher _pub;

  ros::Subscriber _sub_state;
  ros::Subscriber _sub_state_velocity;
  ros::Subscriber _sub_leader;

  int _count;

  geometry_msgs::PoseStamped _current_state;
  geometry_msgs::TwistStamped _current_state_velocity;
  mavros_msgs::PositionTarget _pos;
  geometry_msgs::PoseStamped _leader_state;
  
  void  sub_state(const geometry_msgs::PoseStamped::ConstPtr &);
  void sub_state_velocity(const geometry_msgs::TwistStamped::ConstPtr &);
  void sub_leader(const geometry_msgs::PoseStamped::ConstPtr &);
//--------------------------------------------------------------------------------  
//-----------------------------Relative Distance from Leader For formation Keeping ----------------------
  double formation_x(double ); // The x position command to keep formation  is coming from this function
  double formation_y(double ); // The y position command to keep formation  is coming from this function
  double  _desired_altitude = 20;// We can select the altitude
//-------------------------------------------------------------------------------------------------------
//-----------------------------Position Controller -------------------------------------------------------------------------
  double X_PositionControl(double ); // The velocity command x to track the x position command is coming from this function 
  double Y_PositionControl(double ); // The velocity command y to track the x position command is coming from this function
  double Z_PositionControl(double ); // The velocity command z to track altitude is coming from this function

    //Gain Value: we can tune the gain value , it means that we get trials and errors  
  double P_gain_x_pos = 0.80; // This is defalut value from PX4 Firmware   
  double D_gain_x_pos = 0.08;        

  double P_gain_y_pos = 0.80; // This is defalut value from PX4 Firmware 
  double D_gain_y_pos = 0.08;

  double P_gain_z_pos = 1.0; // This is defalut value from PX4 Firmware 
  double D_gain_z_pos = 0.1;


  double MinValue_fuction(double *);
  double _follow_state_x;
  double _follow_state_y;
  int _system_id;

//------------------------------Heading Control----------------------------------------------------------------------------------------
  double HeadingControl(double , double, double , double , double );  // when the drone is hovering, To make the heading angle constant
//-------------------------------------------------------------------------------------------------------------------------------------  
//--------------------- About centrallize , Setting : The relative distance from leader for "formation" -------------------
  void Classify(int); // This is for classification
  int _desired_pos_x; // The relative distance x from leader for formation
  int _desired_pos_y; // The relative distance x from leader for formation
  int _min_dist;
//-----------------------------------------------------------------------------------------------------


//CollisionAvoidance _pCa;

}; // class Follower

} // namespace swarm::Drone
} // namespace swarm

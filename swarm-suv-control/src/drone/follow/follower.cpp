#include <swarm-suv-control/common.hpp>
#include <swarm-suv-control/drone/follow/follower.hpp>
//#include <swarm-suv-control/drone/follow/follower.pp>

//#include <swarm-suv-control/controller/follow.cpp>
//#include <swarm-suv-control/collisionavoidance.hpp>

namespace swarm {
namespace drone {

#if 0
Follower::Follower()
{
}
#endif

Follower::Follower(int stationId,
                   int followerId)
    : _nh("follower")
    , _rate(100)
      #if 1
      , _desired_pos_x (0) //  for formation between drones which are follower and leader
      , _desired_pos_y (0)
      , _min_dist(0)
      ,_follow_state_x(0) 
      ,_follow_state_y(0)
      ,_system_id(followerId)

      #endif
{
  //!NOTE: TEST
   


  char szPath[256];

  DCHECK();

  #if 0 
  if (0 > snprintf(szPath,
                   256,
                   "/stations/station%d/follow%d/mavros/setpoint_position/local",
                   stationId,
                   followerId))
    throw std::runtime_error("path1"); 
  DCHECK();
  _pub = _nh.advertise<geometry_msgs::PoseStamped>(szPath,
                                                  100);
  #endif 

  if (0 > snprintf(szPath,
                   256,
                   "/stations/station%d/follow%d/mavros/setpoint_raw/local",
                   stationId,
                   followerId))
    throw std::runtime_error("path1");
  DCHECK();
  _pub = _nh.advertise<mavros_msgs::PositionTarget>(szPath,
                                                  100);



  DCHECK();
  if (0 > snprintf(szPath,
                   256,
                   "/stations/station%d/follow%d/mavros/local_position/pose",
                   stationId,
                   followerId))
    throw std::runtime_error("path2");

  DCHECK();
  _sub_state = _nh.subscribe<geometry_msgs::PoseStamped>(szPath,
                                                         100,
                                                         &Follower::sub_state,
                                                         this);

  DCHECK();
  if (0 > snprintf(szPath,
                   256,
                   "/stations/station%d/follow%d/mavros/local_position/velocity_local", // local!
                   stationId,
                   followerId))
  throw std::runtime_error("path3");
  _sub_state_velocity = _nh.subscribe<geometry_msgs::TwistStamped>(szPath,
                                                         100,
                                                         &Follower::sub_state_velocity,
                                                         this);


  DCHECK();
  if (0 > snprintf(szPath,
                   256,
                   "/stations/station%d/leader/mavros/local_position/pose",
                   stationId))
    throw std::runtime_error("path4");

  DCHECK();
  _sub_leader = _nh.subscribe<geometry_msgs::PoseStamped>(szPath,
                                                          100,
                                                          &Follower::sub_leader,
                                                          this);

  DCHECK();
  //ros::spinOnce();


  Classify(followerId);
  
  //distance(followerId);

  //relative_distance_fuction(followerId);

}

Follower::~Follower()
{

}

//!NOTE: Test
void 
Follower::Classify(int id)
{
  switch(id)
  {
    case 1 :
    _desired_pos_x = -20.07;   //The distance from leader to follower 
    _desired_pos_y = 14;
    break;
    case 2:
    _desired_pos_x = -20.07;
    _desired_pos_y = -14;
    break;
    case 3:
    _desired_pos_x = -50.07;
    _desired_pos_y = 28;
    break;
 
    case 4:
    _desired_pos_x = -50.07;
    _desired_pos_y = 0;
    break;
    case 5:
    _desired_pos_x = -50.07;
    _desired_pos_y = -28;
    break;
    default:
    break;
  }
}

void
Follower::Spin()
{

  _pos.header.stamp = ros::Time::now();
  _pos.header.seq = _count;
  _pos.header.frame_id = 1; //ading
  _pos.coordinate_frame = 1; // FRAME_LOCAL_NED=1
  _pos.type_mask = 0b0000101111000111; //i think that it only is using velocity command without position command

 
  double cAngle[3];  // Current Euler Angle
  double leader_cAngle[3]; // Leader : Quaternion Angle

  double qAngle[3];  // Quaternion Angle
  double dAngle[3];  // Desired Angle 
  quternionToeuler(_current_state, cAngle); 
  quternionToeuler(_leader_state, leader_cAngle);  

  double x_position_command = formation_x(leader_cAngle[2]);  // x position command for keeping formation control
  double y_position_command = formation_y(leader_cAngle[2]);  // y position command for keeping formation control
  
  //---------------------P control -----------------------------------
  double _velocity_x = X_PositionControl(x_position_command);
  double _velocity_y = Y_PositionControl(y_position_command);
  double _velocity_z = Z_PositionControl(_desired_altitude);
  //------------------------------------------------------------------
  //--------------------Collision Avoidance --------------------------
  // In the future, will add the collision avoidance
  
//*


   //double min = CollisionAvoidance::GetMinimumValue();

   //std::cout<<"GetMIN"<<min<<std::endl;
 
 //std::cout<<"ID: "<<_system_id<<" ,State X: "<<_follow_state_x<<" ,State Y: "<<_follow_state_y<<std::endl;
  double relative_distance[6] = {6, 6,2, 7 ,-1, 0}; //for 6 UAVs  

 //double relative_distance1[4]= {2,10,1,30};
 
  double Min = MinValue_fuction(relative_distance); // for obtaining the MinValue ( THe values are relative distance between UAVs) 

  //std::cout<<"Min"<<Min<<std::endl;



  //------------------------------------------------------------------
  //--------------------- MAVROS Command------------------------------
  _pos.velocity.x = _velocity_x; // MAVROS Command
  _pos.velocity.y = _velocity_y; // MAVROS Command
  _pos.velocity.z = _velocity_z; // MAVROS Command
  //------------------------------------------------------------------
  //------------------- Heading Control--------------------------------
  double yaw_d = HeadingControl(_current_state_velocity.twist.linear.x, 
  	                              _current_state_velocity.twist.linear.y, 
  	                               cAngle[2], 
  	                              x_position_command, 
  	                              y_position_command);
 //----------------------------------------------------------------------- 
  //ROS_INFO(">> yaw_d : %f", yaw_d);


  //final command

  _pos.yaw = yaw_d; //MAVROS Command
  _pub.publish(_pos);
  _count++;


}

/*
void
Follower::state_colection(id)
{

 swith(id)
  {
  sub_state(const geometry_msgs::PoseStamped::ConstPtr &msg)
 {
  _current_state = *msg;
 }


  } 

}

*/


double 
Follower::FollowerGetStateX()  const 
{ 
  return _follow_state_x; //~ing , 
}



double 
Follower::FollowerGetStateY()  const 
 { 
  return _follow_state_y; 
}

void
Follower::sub_state(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  _current_state = *msg;

  _follow_state_x = _current_state.pose.position.x; // drone current state x
  _follow_state_y = _current_state.pose.position.y;

}



void
Follower::sub_state_velocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  _current_state_velocity = *msg;
}


void
Follower::sub_leader(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  _leader_state = *msg;
}


double 
Follower::formation_x(double heading_uav0)
{
         
  double x = _leader_state.pose.position.x;
  double y = _leader_state.pose.position.y;

  double body_uav0_11_x = x *(cos(heading_uav0));
  double body_uav0_22_x = y *(sin(heading_uav0));
  double body_uav0_xx =  body_uav0_11_x + body_uav0_22_x;
  //std::cout<<"body_uav0_xx" <<body_uav0_xx<<std::endl;

//--------------------------------------------------------------
  double body_uav0_11_y = x *(-sin(heading_uav0));
  double body_uav0_22_y = y *(cos(heading_uav0));
  double body_uav0_yy =  body_uav0_11_y + body_uav0_22_y;
  //std::cout<<"body_uav0_yy" <<body_uav0_yy<<std::endl;



#if 0
  body_uav0_xx = body_uav0_xx - 10.07;
  body_uav0_yy = body_uav0_yy - 14;;   
#else
  body_uav0_xx = body_uav0_xx + _desired_pos_x;
  body_uav0_yy = body_uav0_yy + _desired_pos_y;   // for formation between drones..

#endif   

  double inner_uav0_11_x = body_uav0_xx * (cos(heading_uav0));
  double inner_uav0_22_x = body_uav0_yy * (-sin(heading_uav0));
  double inner_uav0_xx = inner_uav0_11_x  + inner_uav0_22_x;
         
  //std::cout<<"inner_uav0_xx" <<inner_uav0_xx<<std::endl; 

 double inner_uav0_11_y = body_uav0_xx * (sin(heading_uav0));
 double inner_uav0_22_y = body_uav0_yy * (cos(heading_uav0));
 double inner_uav0_yy = inner_uav0_11_y  + inner_uav0_22_y;
         
 //std::cout<<"inner_uav0_yy" <<inner_uav0_yy<<std::endl; 
     
      
 double  x_position_command = inner_uav0_xx; // waypoint x generation to keep formation 
         
 return x_position_command;

} 

double  
Follower::formation_y( double heading_uav0)
{

 double x = _leader_state.pose.position.x; //current position x  of uav0 which is Leader
 double y = _leader_state.pose.position.y; //current position y  of uav0 which is Leader

 double body_uav0_11_x = x *(cos(heading_uav0)); 
 double body_uav0_22_x = y *(sin(heading_uav0));
 double body_uav0_xx =  body_uav0_11_x + body_uav0_22_x; // About x position of inertial frame for Leader, Rotation matrix to change from inertial frame to body frame    
 //std::cout<<"body_uav0_xx" <<body_uav0_xx<<std::endl;

 double body_uav0_11_y = x *(-sin(heading_uav0));
 double body_uav0_22_y = y *(cos(heading_uav0));
 double body_uav0_yy =  body_uav0_11_y + body_uav0_22_y; // About y position of inertial frame for Leader, Rotation matrix to change from inertial frame to body frame
 //std::cout<<"body_uav0_yy" <<body_uav0_yy<<std::endl;


#if 0
  body_uav0_xx = body_uav0_xx - 10.07;
  body_uav0_yy = body_uav0_yy - 14;;   
#else
  body_uav0_xx = body_uav0_xx + _desired_pos_x;
  body_uav0_yy = body_uav0_yy + _desired_pos_y;   

#endif   


 //dy_uav0_xx = body_uav0_xx - 10.07; //Relative distance x from body frame 
 //dy_uav0_yy = body_uav0_yy - 14;;   //Relative distance y from body frame
        

 double inner_uav0_11_x = body_uav0_xx * (cos(heading_uav0));  
 double inner_uav0_22_x = body_uav0_yy * (-sin(heading_uav0));  
 double inner_uav0_xx = inner_uav0_11_x  + inner_uav0_22_x;  //About x position of body frame for Leader, Rotation matrix to change from body frame to inertial frame
          
 //std::cout<<"inner_uav0_xx" <<inner_uav0_xx<<std::endl; 

 double inner_uav0_11_y = body_uav0_xx * (sin(heading_uav0));
 double inner_uav0_22_y = body_uav0_yy * (cos(heading_uav0));
 double inner_uav0_yy = inner_uav0_11_y  + inner_uav0_22_y;  //About y position of body frame for Leader, Rotation matrix to change from body frame to inertial frame
         
 //std::cout<<"inner_uav0_yy" <<inner_uav0_yy<<std::endl; 
     
 double  y_position_command = inner_uav0_yy; // waypoint y generation to keep formation 
         
 return y_position_command;
  
}


double 
Follower::X_PositionControl(double x_position_command)
{
  double _error = (x_position_command - _current_state.pose.position.x);
  double P_vel_control = P_gain_x_pos * _error;
  double _velocity_x = P_vel_control;

  return _velocity_x; 
}

double 
Follower::Y_PositionControl(double y_position_command)
{
  double _error = (y_position_command - _current_state.pose.position.y);
  double P_vel_control = P_gain_y_pos * _error;
  double _velocity_y = P_vel_control; 

  return _velocity_y; 
}

double 
Follower::Z_PositionControl(double _desired_altitude)
{
  double _error = (_desired_altitude - _current_state.pose.position.z);
  double P_vel_control = P_gain_z_pos * _error;
  double _velocity_z = P_vel_control; 

  return _velocity_z; 
}


double
Follower::MinValue_fuction(double * relative_distance)
{

//!NOTE: TODO
  double Min = relative_distance[0];

   for (int i=0; i<6; ++i) // you shoud define i data type such as int, double also auto (you must 0<4 not 0<5 because the form is x[0],x[1],x[2],x[3])
   {
       if(Min > relative_distance[i])
       {
         Min = relative_distance[i];
       }   
   }
           return Min;

}



double 
Follower:: HeadingControl(double _velocity_x, 
	                        double _velocity_y, 
	                        double current_yaw, 
	                        double x_position_command, 
	                        double y_position_command)
{
    double heading_d = swarm::utils::HeadingDirectionToGo(_current_state.pose.position.x,
                                                             _current_state.pose.position.y,
                                                             x_position_command,
                                                             y_position_command);

    // This is for hovering heading command, heading is keeped when drone is trying to hovering   
    double veolocity_scalar = sqrt(pow(_velocity_x,2.0) + pow(_velocity_y, 2.0) );
    std::cout<<"veolocity_scalar" <<veolocity_scalar<<std::endl; 
     
    if ( veolocity_scalar < 0.2)  // To consider hovering heading
       {   
         double yaw_d = current_yaw;   
         return yaw_d;
       }
    else if (veolocity_scalar >0.2)
       {
         double yaw_d = heading_d;
         return yaw_d;
       }
   
}



double Follower::LeaderGetStateX() const { return 0.0f; } //hawk
double Follower::LeaderGetStateY() const { return 0.0f; } //hawk


} // namespace swarm::Drone
} // namespace swarm

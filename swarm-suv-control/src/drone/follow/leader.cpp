#include <swarm-suv-control/common.hpp>
#include <swarm-suv-control/drone/follow/leader.hpp>

namespace swarm {
namespace drone {

#if 0
Leader::Leader()
{
}
#endif

Leader::Leader(int stationId)
    : _nh("leader"),
      _rate(100),
      _count(1), 
      _mode(0),
      _leader_state_x(0), 
      _leader_state_y(0)
{

  char szPath[256];

  DCHECK();


  if (0 > snprintf(szPath,
                   256,
                   "/stations/station%d/leader/mavros/setpoint_position/local",
                   stationId))
    throw std::runtime_error("path1");

  DCHECK();
  _pub = _nh.advertise<geometry_msgs::PoseStamped>(szPath, 100);
 
 
 #if 0
    if (0 > snprintf(szPath,
                   256,
                   "/stations/station%d/leader/mavros/setpoint_raw/local",
                   stationId))
    throw std::runtime_error("path1");
  DCHECK();
  _pub = _nh.advertise<mavros_msgs::PositionTarget>(szPath,
                                                  100);
#endif


  DCHECK();
  if (0 > snprintf(szPath,
                   256,
                   "/stations/station%d/leader/mavros/local_position/pose",
                   stationId))
  throw std::runtime_error("path2");
  
  DCHECK();
  _sub_state = _nh.subscribe<geometry_msgs::PoseStamped>(szPath,
                                                         100,
                                                         &Leader::sub_state,
                                                         this);
  
  DCHECK();
  //ros::spinOnce();
}

Leader::~Leader()
{
}

void
Leader::Spin()
{
  _pos.header.stamp = ros::Time::now();
  _pos.header.seq = _count;
  _pos.header.frame_id = 1; // adding
  //_pos.coordinate_frame = 1; // FRAME_LOCAL_NED=1
  //_pos.type_mask = 0b0000101111000111;

  int arr[3];
  double cAngle[3];
  double qAngle[3];
  
  double dAngle[3];
      
  quternionToeuler(_current_state, cAngle);
  waypoint_mode(arr, _mode);
  attitude_mode(_mode, dAngle);


 
 /* double yaw_d = swarm::utils::HeadingDirectionToGo(_current_state.pose.position.x,
                                                       _current_state.pose.position.y,
                                                       arr[0],
                                                       arr[1]);
  */

//---------------------------------Path planning (waypoint generation) -----------




  double r = 400; // r = 0.2?
  double wn = 0.2;
  double theta = wn*_count*0.01;   

  _pos.pose.position.x = r * sin(theta);
  _pos.pose.position.y = r * cos(theta);
  _pos.pose.position.z = 20;
  std::cout<<"_pos.pose.position.x" <<_pos.pose.position.x<<std::endl;
  std::cout<<"_pos.pose.position.y" <<_pos.pose.position.y<<std::endl;


   double yaw_d = swarm::utils::HeadingDirectionToGo(_current_state.pose.position.x,
                                                       _current_state.pose.position.y,
                                                       _pos.pose.position.x,
                                                       _pos.pose.position.y);


   eulerToquternion(qAngle, dAngle, yaw_d);

  _pos.pose.orientation.w = qAngle[0];
  _pos.pose.orientation.x = qAngle[1];
  _pos.pose.orientation.y = qAngle[2];
  _pos.pose.orientation.z = qAngle[3];





  #if 0
  _pos.velocity.x = 0.85 * (arr[0] -  _current_state.pose.position.x);
  _pos.velocity.y = 0.85 * (arr[1] -  _current_state.pose.position.y);

  // std::cout<<"leader_pos.velocity.x" <<_pos.velocity.x<<std::endl;
  // std::cout<<"leader_pos.velocity.y" <<_pos.velocity.y<<std::endl;

  _pos.velocity.z = 0.85 * (arr[2] -  _current_state.pose.position.z);


  if(_pos.velocity.x >5.0)
      {
        _pos.velocity.x= 5;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
      }
  else if(_pos.velocity.x < -5.0)
      { 
          _pos.velocity.x = -5;
      }


  
  if(_pos.velocity.y >5.0)
      {
        _pos.velocity.y= 5;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
      }
  else if(_pos.velocity.y < -5.0)
      { 
          _pos.velocity.y = -5;
      }

  #endif



  //ROS_INFO(">> leader_yaw_d : %f", yaw_d);
  //ROS_INFO(">> leader_pos.velocity.x : %f", _pos.velocity.x);
 // ROS_INFO(">> leader_pos.velocity.y : %f", _pos.velocity.y);


   //_pos.yaw = yaw_d;
  

  _pub.publish(_pos);

  _count++;
}



double 
Leader::LeaderGetStateX() const 
{ 
  return _leader_state_x; //~ing , 
}



double 
Leader::LeaderGetStateY() const 
 { 
  return _leader_state_y; 
}





void
Leader::sub_state(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  _current_state = *msg;
  _leader_state_x = _current_state.pose.position.x; // drone current state x
  _leader_state_y = _current_state.pose.position.y;

}

void
Leader::waypoint_mode(int *arr,
                      int &mode)
{
  double x = _current_state.pose.position.x;
  double y = _current_state.pose.position.y;

  switch(mode)
  {
    case 0:
      arr[0] = 0;
      arr[1] = 300;
      arr[2] = 10.0;

      if (y > 295 && y < 305)
        mode = mode + 1;               
      break;

    case 1:
      arr[0] = 300.0;
      arr[1] = 0;
      arr[2] = 10.0;
      if (x > 295 && x < 305)
        mode = mode + 1;
      break;

    case 2:
      arr[0] = 0;
      arr[1] = -300.0;
      arr[2] = 10.0;
      if (y <-295  && y  > -305)
        mode = mode + 1;
      break;

    case 3:
      arr[0] = -300.0;
      arr[1] = 0;
      arr[2] = 10.0;
      if (x <-295.0  && x  > -305.0)
        mode = mode + 1;
      break;

    case 4:
      arr[0] = 0;
      arr[1] = 300.0;
      arr[2] = 10.0;

      if (y >295.0  && y  < 305.0)
        mode = 1;
      break;
  }
}

void
Leader::attitude_mode(int mode,
                      double *angle)
{
  switch(mode)
  {
    case 0:
      angle[0] = 0;
      angle[1] = 0;
      angle[2] = 0;
      break;

    case 1:
      angle[0] = 0;
      angle[1] = 0;
      angle[2] = 0;
      break;

    case 2:
      angle[0] = 0;
      angle[1] = 0;
      angle[2] = 0;
      break;

    case 3:
      angle[0] = 0;
      angle[1] = 0;
      angle[2] = 0;
      break;

    case 4:
      angle[0] = 0;
      angle[1] = 0;
      angle[2] = 0;
      break;
  } // switch(mode)
}

double Leader::FollowerGetStateX() const { return 0.0f; }
double Leader::FollowerGetStateY() const { return 0.0f; }

} // namespace swarm::Drone
} // namespace swarm

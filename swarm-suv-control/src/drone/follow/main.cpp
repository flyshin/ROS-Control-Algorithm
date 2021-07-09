#include <swarm-suv-control/controller/follow.hpp>
//#include <swarm-suv-control/collisionavoidance.hpp>
int
main(int argc,
     char **argv)
{
  ros::init(argc, argv, "swarm");
  
  swarm::controller::Follow follow;
  follow.Spin();
  
  return 0;
}

////////////////////////////////////////////////////////////////////////////////

#if 0

namespace swarm {
namespace controller {

Follow::Follow()
    : _nh("follow"),
      _stationId(0),
      _rate(100)
{
  DCHECK();
  ros::NodeHandle nh("~");
  nh.param("station", _stationId, 0);

  DCHECK();
  if (0 >= _stationId || 6 < _stationId)
    throw std::runtime_error("error");

  DCHECK();
  _vec.push_back(std::make_shared<swarm::drone::Leader>(_stationId));

  DCHECK();
  for(int i=1; i<=5; ++i)
    _vec.push_back(std::make_shared<swarm::drone::Follower>(_stationId, i));
}

Follow::~Follow()
{
  DCHECK();
}

void
Follow::Spin()
{
  DCHECK();

  ros::waitForShutdown();

  ROS_INFO("Stopping test");
}

} // namespace swarm::Drone
} // namespace swarm

#endif

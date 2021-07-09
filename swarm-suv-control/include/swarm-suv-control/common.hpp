#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <vector>
#include <memory>
#include <exception>
#include <stdexcept>

#if defined(NDEBUG) // release
#  define DCHECK()
#else // debug
#  define DCHECK() ROS_INFO("%s:%s:%d\n", __FILE__, __FUNCTION__, __LINE__)
#endif

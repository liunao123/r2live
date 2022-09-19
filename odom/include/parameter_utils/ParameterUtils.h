#ifndef PARAMETER_UTILS_H
#define PARAMETER_UTILS_H

#include <cassert>
#include <ros/ros.h>

namespace ros {
namespace param {
inline bool get(const std::string& key, unsigned char& c) {
  int i;
  bool ret = ros::param::get(key, i);
  assert((i >= 0) && (i < 256));
  c = (unsigned char)i;
  return ret;
}

inline bool get(const std::string& key, unsigned int& d) {
  int i;
  bool ret = ros::param::get(key, i);
  assert(i >= 0);
  d = (unsigned int)i;
  return ret;
}

inline bool get(const std::string& key, float& f) {
  double d;
  bool ret = ros::param::get(key, d);
  f = (float)d;
  return ret;
}
}
}

namespace parameter_utils {
template <class M>
bool Get(const std::string& s, M& p) {
  std::string name = ros::this_node::getName();

  std::string r;
  if (!ros::param::search(s, r)) {
    ROS_ERROR("%s: Failed to search for parameter '%s'.", name.c_str(),
              s.c_str());
    return false;
  }

  if (!ros::param::has(r)) {
    ROS_ERROR("%s: Missing required parameter '%s'.", name.c_str(), s.c_str());
    return false;
  }

  if (!ros::param::get(r, p)) {
    ROS_ERROR("%s: Failed to get parameter '%s'.", name.c_str(), s.c_str());
    return false;
  }

  return true;
}

template <class M>
bool Get(const std::string& s, M& p, M def) {
  bool ret = true;

  std::string name = ros::this_node::getName();

  std::string r;
  if (!ros::param::search(s, r)) {
    ROS_DEBUG("%s: Failed to search for parameter '%s', using default.",
              name.c_str(), s.c_str());
    p = def;
    ret = false;
  }

  if (ret && !ros::param::has(r)) {
    ROS_DEBUG("%s: Missing required parameter '%s', using default.",
              name.c_str(), s.c_str());
    p = def;
    ret = false;
  }

  if (ret && !ros::param::get(r, p)) {
    ROS_DEBUG("%s: Failed to get parameter '%s', using default.", name.c_str(),
              s.c_str());
    p = def;
    ret = false;
  }

  return ret;
}
}
#endif

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <queue>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "loop_closure_gtsam/LaserLoopClosure_time.h"
#include "loop_closure_gtsam/LoopTimePair.h"

#include <zlog.h>

#include <std_srvs/Trigger.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

int keyframe_cnts = 0;
geometry_utils::Transform3 last_pose;

// 回环检测的对象
LaserLoopClosure llc;
// double last_callback_time = 0;
bool Exit_this = false;

void add_pose_and_scan_to_gtsam_callback(const geometry_msgs::PoseStampedConstPtr &pose, const sensor_msgs::PointCloud2ConstPtr &scan)
{
  // ROS_INFO("pose time is %f, scan time is : %f", pose->header.stamp.toSec(), scan->header.stamp.toSec());
  geometry_utils::Vector3Base<double> posi_i(pose->pose.position.x,
                                             pose->pose.position.y,
                                             pose->pose.position.z);
  geometry_utils::Rotation3Base<double> ori_i(geometry_utils::QuaternionBase<double>(pose->pose.orientation.w,
                                                                                     pose->pose.orientation.x,
                                                                                     pose->pose.orientation.y,
                                                                                     pose->pose.orientation.z));
  geometry_utils::Transform3 pose_i(posi_i, ori_i);

  //输出的 pcl 类型
  PointCloud::Ptr cloud(new PointCloud);
  pcl::fromROSMsg(*scan, *cloud);
  // cout << "51 cloud->header.stamp is : " << cloud->header.stamp << endl;
  cloud->header.stamp = scan->header.stamp.toSec();
  // cout << "53 cloud->header.stamp is : " << cloud->header.stamp << endl;
	cloud->header.frame_id = cloud->header.frame_id;

  // 第一帧点云 单独插入进去
  if (keyframe_cnts == 0)
  {
    last_pose = pose_i;
    llc.addPoseAndKeyScan(pose_i, cloud);
    keyframe_cnts++;
    return;
  }

  geometry_utils::Transform3 delta = geometry_utils::PoseDelta(last_pose, pose_i);
  last_pose = pose_i;

  if (llc.addPoseAndKeyScan(delta, cloud))
  {
    keyframe_cnts++;
  }
  // cout << "keyframe_cnts is : " << keyframe_cnts << endl;
  ROS_INFO("keyframe_cnts is : %d .", keyframe_cnts);
}

bool startLoopDetect(loop_closure_gtsam::LoopTimePair::Request &req, loop_closure_gtsam::LoopTimePair::Response &res)
{
  llc.setOneLoopTime(req.first, req.second);
  ROS_WARN("get a vision loop time pair. time pair is %f. %f",  req.first, req.second );
  return true;
}

bool endGtsamProcess(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  dzlog_info(" start to save map .then exit gtsam loop optimize process .");
  llc.saveMap();
  Exit_this = true;
  res.success = true;
  res.message = "end gtsam .";
  return true;
}


int initZlog()
{
  if (-1 == access("/home/roslog", F_OK))
  {
    mkdir("/home/roslog", 0777);
  }
  if (dzlog_init("/home/config/zlog.conf", "loop_cat") != 0)
  {
    printf("gnss init zlog failed\n");
    return -1;
  }
  return 0;
}

int main(int argc, char **argv)
{
  //初始化zlog
  if (0 == initZlog())
    dzlog_info("@@@@@@ loopClosure init zlog success !!!");

  ros::init(argc, argv, "loop_closure_gtsam");

  ros::NodeHandle nh;
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_body_pose(nh, "/lidar_pose", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_surf_points(nh, "/livox_surf_point", 10);

  typedef sync_policies::ExactTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_body_pose, sub_surf_points);
  sync.registerCallback(boost::bind(&add_pose_and_scan_to_gtsam_callback, _1, _2));

  ros::ServiceServer server = nh.advertiseService("/get_loop_closure", startLoopDetect);
  ros::ServiceServer server_end_gtsam = nh.advertiseService("/end_gtsam", endGtsamProcess);

  llc.Initialize();

  ros::Rate rate_hz(10);
  while ( ros::ok() )
  { 
    rate_hz.sleep();
    ros::spinOnce();
    if ( Exit_this )
    {
      ROS_INFO("loop_closure_gtsam node exit . ");
      break;
    }
  }

  // ros::spin();

  return 0;
}

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

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>

#include "loop_closure_gtsam/LaserLoopClosure_time.h"
#include "loop_closure_gtsam/LoopTimePair.h"

#include <zlog.h>

#include <std_srvs/Trigger.h>


class OptimizeMapGtsam
{
private:
  ros::NodeHandle nh;

  ros::Subscriber points_sub;

  ros::ServiceServer server;
  ros::ServiceServer server_end_gtsam;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  // 回环检测的对象
  LaserLoopClosure llc;

  geometry_utils::Transform3 last_pose;
  int keyframe_cnts;
  bool Exit_this;

  void points_callback(const sensor_msgs::PointCloud2ConstPtr &points_msg)
  {
    ros::Time pc_stamp = points_msg->header.stamp;

    std::string tfError;
    try
    {
      if (this->tf_buffer.canTransform("world", points_msg->header.frame_id, pc_stamp, ros::Duration(0.1), &tfError))
      {
        geometry_msgs::TransformStamped body_pose_in_world = tf_buffer.lookupTransform("world", points_msg->header.frame_id, pc_stamp, ros::Duration(0.1));

        Eigen::Isometry3d is3d;
        geometry_msgs::PoseStamped pst;

        tf::transformMsgToEigen(body_pose_in_world.transform, is3d);
        tf::poseEigenToMsg(is3d, pst.pose );

        add_pose_and_scan_to_gtsam_callback(pst, points_msg);
      }
      else
      {
        ROS_WARN(" can not canTransform between <world> and <%s> .", points_msg->header.frame_id.c_str() );
      }
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("%s, tf error : %s ", ex.what(), tfError.c_str());
      return;
    }
  }

  bool startLoopDetect(loop_closure_gtsam::LoopTimePair::Request &req, loop_closure_gtsam::LoopTimePair::Response &res)
  {
    llc.setOneLoopTime(req.first, req.second);
    ROS_WARN("get a vision loop time pair. time pair is %f. %f", req.first, req.second);
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

  void add_pose_and_scan_to_gtsam_callback(const geometry_msgs::PoseStamped &pose, const sensor_msgs::PointCloud2ConstPtr &scan)
  {
    // ROS_INFO("pose time is %f, scan time is : %f", pose->header.stamp.toSec(), scan->header.stamp.toSec());
    geometry_utils::Vector3Base<double> posi_i(pose.pose.position.x,
                                               pose.pose.position.y,
                                               pose.pose.position.z);
    geometry_utils::Rotation3Base<double> ori_i(geometry_utils::QuaternionBase<double>(pose.pose.orientation.w,
                                                                                       pose.pose.orientation.x,
                                                                                       pose.pose.orientation.y,
                                                                                       pose.pose.orientation.z));
    geometry_utils::Transform3 pose_i(posi_i, ori_i);

    // 输出的 pcl 类型
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

public:
  OptimizeMapGtsam(/* args */);
  ~OptimizeMapGtsam();
  bool get_exit_flag()
  {
    return Exit_this;
  }

};

OptimizeMapGtsam::OptimizeMapGtsam( ) : tf_buffer(), tf_listener(tf_buffer)
{
  points_sub = nh.subscribe("/livox_surf_point", 5, &OptimizeMapGtsam::points_callback, this);

  server = nh.advertiseService("/get_loop_closure", &OptimizeMapGtsam::startLoopDetect, this);
  server_end_gtsam = nh.advertiseService("/end_gtsam", &OptimizeMapGtsam::endGtsamProcess, this);

  llc.Initialize();
  keyframe_cnts = 0;
  Exit_this = false;

}

OptimizeMapGtsam::~OptimizeMapGtsam()
{
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
  // 初始化zlog
  if (0 == initZlog())
    dzlog_info("@@@@@@ loopClosure init zlog success !!!");

  ros::init(argc, argv, "loop_closure_gtsam_tf2");

  OptimizeMapGtsam omg;

  ros::Rate rate_hz(10);
  while (ros::ok())
  {
    rate_hz.sleep();
    ros::spinOnce();
    if ( omg.get_exit_flag() )
    {
      break;
    }
  }

  return 0;
}

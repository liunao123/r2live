#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <deque>
#include <vector>
#include <mutex>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>

#include <pcl/filters/extract_indices.h>

using namespace std;

// rslidar和velodyne的格式有微小的区别
// rslidar的点云格式
struct RsPointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  // PCL_ADD_INTENSITY;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

std::deque<sensor_msgs::Imu::ConstPtr> imu_msg_v;
std::mutex mutex_lock;

double last_lidar_time = 0;
int lidar_cnts = 0;
pcl::ExtractIndices<RsPointXYZIRT> extract;

ros::Publisher pub_undistorted;

sensor_msgs::PointCloud2 undis_pc;

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr * 180 / M_PI;
}

// IMU消息回调存储
void imu_cbk(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
  mutex_lock.lock();
  imu_msg_v.push_back(imu_msg);
  mutex_lock.unlock();
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (lidar_cnts == 0)
  {
    last_lidar_time = msg->header.stamp.toSec();
    lidar_cnts++;
    return;
  }

  mutex_lock.lock();
  while (imu_msg_v.front()->header.stamp.toSec() < last_lidar_time)
  {
    imu_msg_v.pop_front();
  }

  last_lidar_time = msg->header.stamp.toSec();

  mutex_lock.unlock();

  Eigen::Quaterniond imu_q = Eigen::Quaterniond::Identity(); // R
  Eigen::Vector3d angular_velocity_1;
  angular_velocity_1 << imu_msg_v[0]->angular_velocity.x, imu_msg_v[0]->angular_velocity.y, imu_msg_v[0]->angular_velocity.z;
  double lastest_time = imu_msg_v[0]->header.stamp.toSec();
  // for (int i = 1; i < imu_msg_v.size(); i++)
  int i = 1;
  while (i < imu_msg_v.size() && imu_msg_v[i]->header.stamp.toSec() < msg->header.stamp.toSec())
  {
    double t = imu_msg_v[i]->header.stamp.toSec();
    double dt = t - lastest_time;
    Eigen::Vector3d angular_velocity_2;
    angular_velocity_2 << imu_msg_v[i]->angular_velocity.x, imu_msg_v[i]->angular_velocity.y, imu_msg_v[i]->angular_velocity.z;
    Eigen::Vector3d aver_angular_vel = (angular_velocity_1 + angular_velocity_2) / 2.0;
    imu_q = imu_q * Eigen::Quaterniond(1, 0.5 * aver_angular_vel(0) * dt, 0.5 * aver_angular_vel(1) * dt, 0.5 * aver_angular_vel(2) * dt);

    // std::cout << "eulerAngle: " << R2ypr(imu_q.toRotationMatrix()).transpose() <<  std::endl;

    lastest_time = t;
    i++;
  }

  ROS_ERROR("--- %s ", pcl::getFieldsList(*msg).c_str()) ;
  pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_in(new pcl::PointCloud<RsPointXYZIRT>());
  pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_undis(new pcl::PointCloud<RsPointXYZIRT>());

  pcl::fromROSMsg(*msg, *cloud_in);
  // std::cout << "cloud_in " << cloud_in->size() << std::endl;

  // 根据索引把 nan 去掉
  pcl::PointCloud<pcl::PointXYZ> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  std::vector<int> save_index;
  pcl::removeNaNFromPointCloud(pl_orig, pl_orig, save_index);
  boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(save_index);
  extract.setInputCloud(cloud_in);
  extract.setIndices(index_ptr);
  extract.setNegative(false); // 保留 不是 索引的 数据 设置为  true
  extract.filter(*cloud_in);

  // std::cout << "cloud_in " << cloud_in->size()  << std::endl;
  // std::cout << "index_ptr size  " << index_ptr->size()  << std::endl;

  double period_time = msg->header.stamp.toSec() - cloud_in->points[0].timestamp;

  // ROS_WARN_STREAM( "cloud_in " << cloud_in->size()  << std::endl);

  pcl::io::savePCDFileASCII("/home/liunao/qt/tools/static_pcd/distorted_cloud_in.pcd", *cloud_in);

  // cloud_undis->points.resize( cloud_in->size() );

  for (int i = 0; i < cloud_in->size(); i++)
  {
    auto pt = cloud_in->points[i];
    // printf("pt : %f %f %f %f %f %d \n",pt.x, pt.y,pt.z, pt.intensity, pt.timestamp, pt.ring );
    // printf("pt : %f %d \n", pt.timestamp, pt.ring );

    Eigen::Quaterniond lidar_q0 = Eigen::Quaterniond::Identity(); // R

    double percent = 1 - (msg->header.stamp.toSec() - pt.timestamp) / period_time;
    // ROS_WARN_STREAM("percent " << percent << std::endl);

    Eigen::Quaterniond q_slerp = lidar_q0.slerp(percent, imu_q);
    Eigen::Matrix3d R_slerp = q_slerp.toRotationMatrix();
    Eigen::Matrix4d T_lidar_slerp;
    T_lidar_slerp << R_slerp(0, 0), R_slerp(0, 1), R_slerp(0, 2), 0,
        R_slerp(1, 0), R_slerp(1, 1), R_slerp(1, 2), 0,
        R_slerp(2, 0), R_slerp(2, 1), R_slerp(2, 2), 0,
        0, 0, 0, 1;
    Eigen::Vector4d Pi, Pj;
    Pi << cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, 1;
    Pj = T_lidar_slerp.inverse() * Pi;
    pt.x = Pj(0);
    pt.y = Pj(1);
    pt.z = Pj(2);

    cloud_undis->points.push_back(pt);
  }
  // ROS_WARN_STREAM( "cloud_undis " << cloud_undis->size()  << std::endl);
  cloud_undis->width = cloud_undis->points.size();
  cloud_undis->height = 1;

  pcl::io::savePCDFileASCII("/home/liunao/qt/tools/static_pcd/undistorted_cloud_in.pcd", *cloud_undis);

  undis_pc.header = (*msg).header;
  undis_pc.header.stamp =  msg->header.stamp;
  undis_pc.header.frame_id = "velodyne";

  // ROS_WARN_STREAM(" sec " << undis_pc.header.stamp.toSec() << std::endl);
  // ROS_WARN_STREAM("undis_pc.header.frame_id " << undis_pc.header.frame_id << std::endl);

  pcl::toROSMsg(*cloud_undis, undis_pc);

  pub_undistorted.publish(undis_pc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "undistorted_pcd");

  ros::NodeHandle nh;

  ros::Subscriber sub_pcl = nh.subscribe("/velodyne_points", 20, standard_pcl_cbk);
  ros::Subscriber sub_imu = nh.subscribe("/livox/imu", 200000, imu_cbk);
  pub_undistorted = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_undistorted", 100);

  ros::spin();

  return 0;
}

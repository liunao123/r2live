#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <std_srvs/Trigger.h>

#include <ctime>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZ;

int step = 2;

class Save_pointcloud
{
private:
  PointCloudXYZ::Ptr map; //   (new PointCloudXYZI());
  ros::Subscriber sub_cloud_registered;
  ros::ServiceServer server_end_gtsam;
  pcl::VoxelGrid<PointType> downSizeFilterMap;
  int cnts;
  double last_pc_time;
  char pcd_time[100];

public:
  Save_pointcloud(ros::NodeHandle &nh);

  void save_map(const sensor_msgs::PointCloud2::ConstPtr &reg_pc);
  bool service_sm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  ~Save_pointcloud();
};

Save_pointcloud::Save_pointcloud(ros::NodeHandle &nh)
{
  map = boost::make_shared<PointCloudXYZ>();
  cnts = 0;
  downSizeFilterMap.setLeafSize(0.2f, 0.2f, 0.2f);
  sub_cloud_registered = nh.subscribe("/cloud_registered", 1000, &Save_pointcloud::save_map, this);
  server_end_gtsam = nh.advertiseService("/save_map", &Save_pointcloud::service_sm, this);
  last_pc_time = 0;
}

// Save_pointcloud::~Save_pointcloud()
// {
//   std::cout << "size of map <before filter >: " << map->width * map->height << std::endl;
//   downSizeFilterMap.setInputCloud(map);
//   downSizeFilterMap.filter(*map);
//   std::cout << "size of map <after filter >:  " << map->width * map->height << std::endl;
//   // 保存点云到指定路径
//   if (!map->points.empty())
//   {
//     std::cout << "START : save map to pcd file : /home/map/Save_cloud_registered.pcd" << std::endl;
//     pcl::io::savePCDFile("/home/map/Save_cloud_registered.pcd", *map);
//     std::cout << "FINISH : save map to pcd file : /home/map/Save_cloud_registered.pcd" << std::endl;
//   }
//   else
//   {
//     ROS_WARN("-------------- WARN: pointcloud is empty . -----------");
//   }
// }

Save_pointcloud::~Save_pointcloud()
{
  // std_srvs::Trigger::Request req;
  // std_srvs::Trigger::Response res;
  // service_sm(req, res);
  std::cout << "original.pcd : " << map->width * map->height << std::endl;
  pcl::io::savePCDFile("/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_original.pcd", *map);
}

bool Save_pointcloud::service_sm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{

  std::cout << "pcd_name: " << pcd_time << std::endl;

  if (!map->points.empty())
  {
    std::cout << "size of map <before filter >: " << map->width * map->height << std::endl;
    downSizeFilterMap.setLeafSize(0.1f, 0.1f, 0.1f);
    downSizeFilterMap.setInputCloud(map);
    downSizeFilterMap.filter(*map);
    std::cout << "size of map <after filter >:  " << map->width * map->height << std::endl;
    // 保存点云到指定路径
    std::cout << "START : save map to pcd file : "
              << "/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_10.pcd" << std::endl;
    pcl::io::savePCDFile("/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_10.pcd", *map);
    std::cout << "FINISH : save map to pcd file : "
              << "/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_10.pcd" << std::endl;

    downSizeFilterMap.setLeafSize(0.25f, 0.25f, 0.25f);
    downSizeFilterMap.setInputCloud(map);
    downSizeFilterMap.filter(*map);
    pcl::io::savePCDFile("/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_025.pcd", *map);
  }
  else
  {
    ROS_WARN("-------------- WARN: pointcloud is empty . -----------");
  }
  // map->points.clear();
  res.success = true;
  res.message = "pcd file:  /home/map/" + std::string(pcd_time) + "_Save_cloud_registered_10.pcd";
  return true;
}

void Save_pointcloud::save_map(const sensor_msgs::PointCloud2::ConstPtr &reg_pc)
{

  if (reg_pc->header.stamp.toSec() < last_pc_time || std::fabs(reg_pc->header.stamp.toSec() - last_pc_time) > 10000.0)
  {
    /* code */
    map->points.clear();
    ROS_WARN("-------------- clear map: pointcloud is empty . -----------");
  }

  cnts++;
  if (cnts == 1)
  {
    time_t first_time = reg_pc->header.stamp.toSec();
    tm local_time;
    localtime_r(&first_time, &local_time);
    strftime(pcd_time, sizeof(pcd_time), "%Y-%m-%d-%H-%M-%S", &local_time);
  }

  if (cnts % step == 0)
  // if (1)
  {
    PointCloudXYZ::Ptr one_frame(new PointCloudXYZ());
    pcl::fromROSMsg(*reg_pc, *one_frame);
    // PointCloudXYZI::Ptr temp;
    // temp = boost::make_shared<PointCloudXYZI>();
    // pcl::copyPointCloud(*laserCloudFullResColor, *temp); //复制
    // (*temp_map) += (*temp); //
    // ROS_WARN("size of one_frame <before filter >: %d ", one_frame->width * one_frame->height);
    // downSizeFilterMap.setInputCloud(one_frame);
    // downSizeFilterMap.filter(*one_frame);
    // ROS_WARN("size of one_frame <after filter >: %d ", one_frame->width * one_frame->height);
    (*map) += (*one_frame);
    // ROS_INFO("size of map :  %d  ", map->width * map->height);
  }
  last_pc_time = reg_pc->header.stamp.toSec();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_map");
  std::cout << "save_map node start ...... " << std::endl;
  if (argc == 2)
  {
    step = std::stoi(argv[1]);
    std::cout << "step: " << step << std::endl;
  }
  ros::NodeHandle nh;

  Save_pointcloud sm(nh);
  ros::spin();

  return (0);
}

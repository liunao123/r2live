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


typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZ;

class Save_pointcloud
{
private:
  PointCloudXYZ::Ptr map;//   (new PointCloudXYZI());
  ros::Subscriber sub_cloud_registered;
  pcl::VoxelGrid<PointType> downSizeFilterMap;
  int cnts ;

public:
  Save_pointcloud(ros::NodeHandle &nh);

  void save_map(const sensor_msgs::PointCloud2::ConstPtr &reg_pc);

  ~Save_pointcloud();

};

Save_pointcloud::Save_pointcloud(ros::NodeHandle &nh)
{
  // map = new PointCloudXYZ();
  map = boost::make_shared<PointCloudXYZ>();
  cnts = 3;
  downSizeFilterMap.setLeafSize(0.1f , 0.1f, 0.1f);
  sub_cloud_registered = nh.subscribe("/cloud_registered", 1000, &Save_pointcloud::save_map, this);
}

Save_pointcloud::~Save_pointcloud()
{
  std::cout << "size of map <before filter >: " << map->width * map->height << std::endl;
  downSizeFilterMap.setInputCloud(map);
  downSizeFilterMap.filter(*map);
  std::cout << "size of map <after filter >:  " << map->width * map->height << std::endl;
  // 保存点云到指定路径
  if(!map->points.empty())
  {
      std::cout << "START : save map to pcd file : /home/map/Save.pcd" << std::endl;
      pcl::io::savePCDFile ( "/home/map/Save.pcd", *map);
      std::cout << "FINISH : save map to pcd file : /home/map/Save.pcd" << std::endl;
  }
  else
  {
      ROS_WARN("-------------- WARN: pointcloud is empty . -----------");
  }
}


void Save_pointcloud::save_map(const sensor_msgs::PointCloud2::ConstPtr &reg_pc)
{
  if (cnts++ == 3)
{
  cnts = 0;
  PointCloudXYZ::Ptr one_frame(new PointCloudXYZ());
  pcl::fromROSMsg(*reg_pc, *one_frame);
  // PointCloudXYZI::Ptr temp;
  // temp = boost::make_shared<PointCloudXYZI>();
  // pcl::copyPointCloud(*laserCloudFullResColor, *temp); //复制
  // (*temp_map) += (*temp); // 
  //ROS_WARN("size of one_frame <before filter >: %d ", one_frame->width * one_frame->height);                    
  downSizeFilterMap.setInputCloud(one_frame);
  downSizeFilterMap.filter(*one_frame);
  //ROS_WARN("size of one_frame <after filter >: %d ", one_frame->width * one_frame->height);
  (*map) += (*one_frame);
  ROS_INFO("size of map :  %d  ", map->width * map->height);
}

}






int main (int argc, char** argv)
{
  ros::init(argc, argv, "save_map");
  std::cout << "save_map node start ...... " << std::endl;
  ros::NodeHandle nh;
  Save_pointcloud sm(nh);
  ros::spin();
  //pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);

  return (0);
  
}

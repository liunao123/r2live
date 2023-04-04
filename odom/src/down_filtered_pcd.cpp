#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/distances.h>

int main(int argc, char **argv)
{
  Eigen::Vector4f pt = {-1,0,0,0};
	Eigen::Vector4f line_pt_1 = { 0.218,0.278,0,0 };
	Eigen::Vector4f line_pt_2 = { 138.709, -8.279 ,0,0 };
	Eigen::Vector4f line_dir= line_pt_2 - line_pt_1;

  double dis_sqr = pcl::sqrPointToLineDistance(pt, line_pt_1, line_dir);
  printf("27 dis is %f \n", dis_sqr);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cerr << "cloud : " << cloud->size() << " points to pcd." << std::endl;

  pcl::PointCloud<pcl::PointXYZ> output_cloud;
  for (uint i = 0; i < cloud->size(); ++i) {
    if( cloud->points[i].y > -8.5 && cloud->points[i].y < 0.0 &&  cloud->points[i].z > 0 )
      {
        pt = Eigen::Vector4f (cloud->points[i].x, cloud->points[i].y, 0, 0);
        dis_sqr = pcl::sqrPointToLineDistance(pt, line_pt_1, line_dir);
        if (dis_sqr < 2.0 * 2.0 )
        {
          output_cloud.points.push_back(cloud->points[i]); // 保留车
          // continue;
        }
          // output_cloud.points.push_back(cloud->points[i]); // 去掉 车
      }
  }
  output_cloud.is_dense = false;
  output_cloud.width = output_cloud.points.size();
  output_cloud.height = 1;
  pcl::io::savePCDFileASCII("/home/liunao/qt/pcd/sh_st/_che.pcd", output_cloud);
  printf("44 ok");
  std::cerr << "Saved : " << output_cloud.size() << " points to pcd." << std::endl;
  return 1;
}

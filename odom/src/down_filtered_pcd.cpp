#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv)
{

  if (argc != 3)
    return -1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *target_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }

  pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterTempMap;
  const float voxel_size = std::stof( argv[2] );

  downSizeFilterTempMap.setLeafSize(voxel_size, voxel_size, voxel_size);
  downSizeFilterTempMap.setInputCloud(target_cloud);
  downSizeFilterTempMap.filter(*cloud);  
  pcl::io::savePCDFile("test_pcd_30.pcd", *cloud);

  // downSizeFilterTempMap.setLeafSize(0.01, 0.01, 0.01);
  // downSizeFilterTempMap.setInputCloud(target_cloud);
  // downSizeFilterTempMap.filter(*cloud);
  // pcl::PointCloud<pcl::PointXYZ> output_cloud;

  //       for (uint i = 0; i < cloud->size(); ++i) {
  //       if( cloud->points[i].x > 0  and cloud->points[i].x < 5 and  std::fabs( cloud->points[i].y ) < 5 )
  //       {
  //         output_cloud.points.push_back(cloud->points[i]);
  //       }
  //     }
  // output_cloud.is_dense = false;
  // output_cloud.width = output_cloud.points.size();
  // output_cloud.height = 1;
  // pcl::io::savePCDFileASCII("test_pcd_.pcd", output_cloud);

  // pcl::io::savePCDFile("test_pcd_30.pcd", *cloud);



  // downSizeFilterTempMap.setLeafSize(voxel_size+0.3, voxel_size+0.3, voxel_size+0.3);
  // downSizeFilterTempMap.setInputCloud(target_cloud);
  // downSizeFilterTempMap.filter(*cloud);  
  // pcl::io::savePCDFile("test_pcd_3.pcd", *cloud);

  std::cerr << "Saved voxel after: " << cloud->size() << " data points to test_pcd.pcd." << std::endl;

  return (0);
}

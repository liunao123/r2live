#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>

int main(int argc, char **argv)
{
  std::string cloud_file;
  if (argc > 1)
  {
    cloud_file = argv[1];
  }
  else
  {
    PCL_ERROR("you need given a pcd file .\n");
    PCL_ERROR("USAGE: rotation_pcd_to_horizon_pcl pcd_file. \n");
    PCL_ERROR("ex: rotation_pcd_to_horizon_pcl /home/1.pcd \n");
    return -1;
  }
  std::cout << "Point cloud file is \"" << cloud_file << "\"\n";
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_some;

  pcl::io::loadPCDFile(cloud_file, cloud);
  printf("size of all cloud map: %ld . \n", cloud.points.size());

  // dzlog_info(" pc size() %d ", cloud->size());
  float range = 3.0;
  static pcl::CropBox<pcl::PointXYZ> cropBoxFilter_temp(true);
  cropBoxFilter_temp.setInputCloud( cloud.makeShared() );
  cropBoxFilter_temp.setMin(Eigen::Vector4f(-range, -range, -range, 1.0f));
  cropBoxFilter_temp.setMax(Eigen::Vector4f(range, range, range, 1.0f));
  cropBoxFilter_temp.setNegative(false);
  cropBoxFilter_temp.filter(*cloud_some);
  printf("size of some cloud map: %ld . \n", cloud_some->points.size());

  // 创建一个模型参数对象，用于记录结果
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // inliers表示误差能容忍的点 记录的是点云的序号
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // 创建一个分割器
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
  seg.setOptimizeCoefficients(true);
  // Mandatory-设置目标几何形状
  seg.setModelType(pcl::SACMODEL_PLANE);
  // 分割方法：随机采样法
  seg.setMethodType(pcl::SAC_RANSAC);
  // 设置误差容忍范围，也就是我说过的阈值
  seg.setDistanceThreshold(0.001);
  // 输入点云
  // seg.setInputCloud(cloud.makeShared());
  seg.setInputCloud( cloud_some );
  // 分割点云
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("Could not estimate a planar model for the given dataset. EXIT . ");
    return (-1);
  }

  std::cout << "get planar model size: " << inliers->indices.size() << std::endl;

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;

  // 参考: https://blog.csdn.net/weixin_38636815/article/details/109543753

  // 首先求解出旋转轴和旋转向量
  // a,b,c为求解出的拟合平面的法向量，是进行归一化处理之后的向量。
  Eigen::Vector3d plane_norm(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

  // xz_norm是参考向量，也就是XOY坐标平面的法向量
  Eigen::Vector3d xz_norm(0.0, 0.0, 1.0);

  // 求解两个向量的点乘
  double v1v2 = plane_norm.dot(xz_norm);

  // 计算平面法向量和参考向量的模长，因为两个向量都是归一化之后的，所以这里的结果都是1.
  double v1_norm = plane_norm.norm();
  double v2_norm = xz_norm.norm();
  // 计算两个向量的夹角
  double theta = std::acos(v1v2 / (v1_norm * v2_norm));
  std::cout << "theta <rad> is  : " << theta << std::endl;
  std::cout << "theta <deg> is  : " << theta * 180.0 / 3.14159 << std::endl;

  // 根据向量的叉乘求解同时垂直于两个向量的法向量。
  Eigen::Vector3d axis_v1v2 = xz_norm.cross(plane_norm);

  // 对旋转向量进行归一化处理
  axis_v1v2 = axis_v1v2 / axis_v1v2.norm();

  // 计算旋转矩阵
  Eigen::AngleAxisd ro_vector(-theta, Eigen::Vector3d(axis_v1v2.x(), axis_v1v2.y(), axis_v1v2.z()));
  Eigen::Matrix3d ro_matrix = ro_vector.toRotationMatrix();
  // std::cout << "ro_matrix eigen " << ro_matrix << std::endl;

  // 计算提取出的地面点的 平均高度，后面可以统一减去这个值
  double sum_z = 0;
  for (int i = 0; i < inliers->indices.size (); i++)
  {
    sum_z += cloud.points[ inliers->indices[i] ].z ;
  }
  double mean_z = sum_z / inliers->indices.size ();
  std::cout << "mean_z: " << mean_z << std::endl;

  pcl::PointCloud<pcl::PointXYZ> flat_cloud;
  flat_cloud = cloud;
  flat_cloud.points.clear();

  for (int i = 0; i < cloud.points.size(); i++)
  {
    Eigen::Vector3d newP(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    Eigen::Vector3d new_point = ro_matrix * newP;
    pcl::PointXYZ pt;
    pt.x = new_point.x();
    pt.y = new_point.y();
    pt.z = new_point.z()  -  mean_z ;
    flat_cloud.points.push_back(pt);
  }

  std::cout << "flat_cloud size: " << flat_cloud.points.size() << std::endl;

  cloud_file.insert(cloud_file.size() - 4, "_horizontal");

  std::cout << "save result pcd :cloud_file_horizontal.pcd  >>  " << cloud_file << std::endl;

  pcl::io::savePCDFileASCII(cloud_file, flat_cloud);
}

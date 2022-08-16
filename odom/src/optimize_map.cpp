/*
 * @Author: liunao123 812807457@qq.com
 * @Date: 2022-08-13 18:15:25
 * @LastEditors: liunao123 812807457@qq.com
 * @LastEditTime: 2022-08-14 11:23:50
 * @FilePath: /odom/src/optimize_map.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <ros/time.h>
#include <vector>

#include <algorithm>
#include <stdlib.h>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <sys/types.h>
#include <dirent.h>

#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>



using namespace std;
using namespace Eigen;

geometry_msgs::PoseStamped to_PoseStamped(std::vector<double> v8)
{
	geometry_msgs::PoseStamped p;
	p.header.stamp = ros::Time().fromSec(v8[0] / 1e9);
	p.pose.position.x = v8[1];
	p.pose.position.y = v8[2];
	p.pose.position.z = v8[3];
	p.pose.orientation.w = v8[4];
	p.pose.orientation.x = v8[5];
	p.pose.orientation.y = v8[6];
	p.pose.orientation.z = v8[7];

	// for(int i=0;i<8;i++)
	//     	cout << " " << v8[i]  ;
   	// cout << endl  ;

	return p;
}

void GetFileNames(string path, vector<string> &filenames)
{
	DIR *pDir;
	struct dirent *ptr;
	if (!(pDir = opendir(path.c_str())))
		return;
	while ((ptr = readdir(pDir)) != 0)
	{
		if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
			filenames.push_back(path + "/" + ptr->d_name);
	}
	closedir(pDir);
}



int main(int argc, char **argv)
{
	ifstream infile;
	std::string work_dir = "/home/map/";
	infile.open(work_dir + "0_0.000000vins_result_loop.txt", ios::in);
	if (!infile.is_open())
	{
		std::cout << "读取文件失败" << std::endl;
		return 0;
	}

	const char *delim = ",";
	char buf[1024];

    // 获取 loop_path 的位姿
	std::vector<geometry_msgs::PoseStamped> loop_pose;
	while (infile.getline(buf, sizeof(buf)))
	{
		char *p;
		p = strtok(buf, delim);

		std::vector<double> item_8;
		while (p)
		{
			std::string item(p);
			item_8.push_back(stold(item));
			p = strtok(NULL, delim);
		}
		loop_pose.push_back(to_PoseStamped(item_8));
	}

	double start_time = loop_pose[0].header.stamp.toSec();
	double end_time = loop_pose.back().header.stamp.toSec();
	cout << "start_time : " << std::to_string(start_time) << endl;
	cout << "end_time : " << std::to_string(end_time) << endl;

	vector<string> pcd_file;
	std::string pcd_dir = work_dir + "pcd_0805";
	GetFileNames(pcd_dir, pcd_file);

	pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

	// for (size_t i = 0; i < pcd_file.size(); i++)
	for (size_t i = 0; i < 500 ; i++)
	{
		double pcd_time = stold(pcd_file[i].substr(pcd_file[i].find_last_of('/') + 1, 22));

		// 在优化的轨迹时间段之外的点云去掉
		if (pcd_time < start_time || pcd_time > end_time)
		{
			continue;
		}
		// 在优化轨迹时间段内的 点云 进行，插值求位姿
		else
		{
		cout << i << "th file : " << pcd_file[i] << endl;
			for (size_t j = 0; j < loop_pose.size() - 1; j++)
			{
				double before_time = loop_pose[j].header.stamp.toSec();
				double after_time = loop_pose[j + 1].header.stamp.toSec();
				// 该点云处在  哪两个 loop_path之间
				if (pcd_time > before_time && pcd_time < after_time)
				{
					double dt_before = pcd_time - before_time;

					double dt_after = after_time - pcd_time;
					double dt = after_time - before_time ;
					cout <<  "  dt : " << dt   << " dt_before " << dt_before << "  lamda :" << dt_before / dt << endl;
					
					if (dt > 0.3)
					{
						continue;
					}

					// 对旋转插值
					Quaterniond start_ori = Quaterniond(loop_pose[j].pose.orientation.w,
														loop_pose[j].pose.orientation.x,
														loop_pose[j].pose.orientation.y,
														loop_pose[j].pose.orientation.z);

					Quaterniond end_ori = Quaterniond(loop_pose[j + 1].pose.orientation.w,
													  loop_pose[j + 1].pose.orientation.x,
													  loop_pose[j + 1].pose.orientation.y,
													  loop_pose[j + 1].pose.orientation.z);

					Eigen::Quaternion<double> now_pcd_rot = start_ori.slerp(  0.5  , end_ori);


					// Eigen::Vector3d start_ori_eular = start_ori.matrix().eulerAngles(2,1,0);
					// cout << start_ori_eular.x() << " " << start_ori_eular.y() << " " << start_ori_eular.z() << " " << endl << endl;
					// Eigen::Vector3d now_pcd_rot_eular = now_pcd_rot.matrix().eulerAngles(2,1,0);
					// cout << now_pcd_rot_eular.x() << " " << now_pcd_rot_eular.y() << " " << now_pcd_rot_eular.z() << " " << endl << endl;
					// Eigen::Vector3d end_ori_eular = end_ori.matrix().eulerAngles(2,1,0);
					// cout << end_ori_eular.x() << " " << end_ori_eular.y() << " " << end_ori_eular.z() << " " << endl << endl;

					// 对平移插值
					Eigen::Vector3d
						start_pos = Eigen::Vector3d(loop_pose[j].pose.position.x,
													loop_pose[j].pose.position.y,
													loop_pose[j].pose.position.z);
					Eigen::Vector3d
						end_pos = Eigen::Vector3d(loop_pose[j + 1].pose.position.x,
												  loop_pose[j + 1].pose.position.y,
												  loop_pose[j + 1].pose.position.z);

					// Eigen::Vector3d now_pcd_tans = start_pos + (end_pos - start_pos) * (dt_before / dt);
					Eigen::Vector3d now_pcd_tans = start_pos + (end_pos - start_pos) * (0.5);

					// cout << now_pcd_tans.transpose() << endl;

					if(dt_before > dt_after)
					{
						now_pcd_rot = end_ori;
						now_pcd_tans = end_pos;
					}
					else
					{
						now_pcd_rot = start_ori;
						now_pcd_tans = start_pos;
					}

					// end 20220813
					// 构建变换矩阵
					Eigen::Isometry3d TM = Eigen::Isometry3d::Identity();
					TM.pretranslate(now_pcd_tans);
					TM.rotate(now_pcd_rot.matrix());
					// TODO:
					// cout << TM.matrix() << endl;
					// 根据上述 旋转和平移构造对应的 变换矩阵，把点云 投影过去即可。
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

					if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file[i], *cloud) == -1)
					{
						PCL_ERROR("can not load pcd file!");
						return (-1);
					}
					pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
					transformed_cloud->points.resize(cloud->points.size());


                    static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.05512, 0.02226, 0.0297); // Horizon r2live
					// 官方 https://livox-wiki-cn.readthedocs.io/zh_CN/latest/introduction/Point_Cloud_Characteristics_and_Coordinate_System%20.html#id2
                    // static const Eigen::Vector3d Lidar_offset_to_IMU_temp(-0.0847, -0.0425, -0.0353); // -84.7，-42.5，-35.3

// way 1
					for (int in=0; in < cloud->points.size() ; in++)
					{
                        cloud->points[in].x += Lidar_offset_to_IMU_temp[0];
                        cloud->points[in].y += Lidar_offset_to_IMU_temp[1];
                        cloud->points[in].z += Lidar_offset_to_IMU_temp[2];
					}
					pcl::transformPointCloud(*cloud, *transformed_cloud, TM.matrix());



// way 2
					// for (int in=0; in < cloud->points.size() ; in++)
					// {
                    //     Eigen::Vector3d p_body(cloud->points[in].x, cloud->points[in].y, cloud->points[in].z);
                    //     Eigen::Vector3d p_global(now_pcd_rot.matrix() * (p_body + Lidar_offset_to_IMU_temp) + now_pcd_tans);
                    //     transformed_cloud->points[in].x = p_global[0];
                    //     transformed_cloud->points[in].y = p_global[1];
                    //     transformed_cloud->points[in].z = p_global[2];
					// }

					(*map) += (*transformed_cloud);
                    viewer.showCloud(map);
		             boost::this_thread::sleep(boost::posix_time::microseconds(100));

				}
			}
		}
	}
	std::string name;
	name = pcd_dir + "map.pcd";
	pcl::io::savePCDFile(name, *map);
	cout << "path is :" << name << endl;

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(map);
	float resolution = 0.25f;
	sor.setLeafSize(resolution, resolution, resolution);
	sor.filter(*map);
	name = pcd_dir + std::to_string(resolution) + "_map.pcd";
	pcl::io::savePCDFile(name, *map);
	cout << "path is :" << name << endl;
	return 0;
}

#include <iostream>
#include <iomanip>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <vector>
#include <string>
#include <algorithm>

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/gicp.h>


// #include <odom/LaserLoopClosure.h>
#include <odom/LaserLoopClosure_time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <zlog.h>

using namespace std;
using namespace Eigen;
namespace gu = geometry_utils;

void scan_dir_get_filename(string path, vector<string> &filenames)
{
	struct dirent **entry_list;
	int count;
	int i;

	count = scandir(path.c_str(), &entry_list, 0, alphasort);
	if (count < 0)
	{
		perror("scandir");
	}

	for (i = 0; i < count; i++)
	{
		struct dirent *entry;
		entry = entry_list[i];
		// printf("%s\n", entry->d_name);
		// 跳过 ./ 和 ../ 两个目录
		if (i < 2)
		{
			continue;
		}
		filenames.push_back(path + std::string(entry->d_name));
		free(entry);
	}
	free(entry_list);
}

geometry_msgs::PoseStamped to_PoseStamped(std::vector<double> v8)
{
	geometry_msgs::PoseStamped p;
	p.header.stamp = ros::Time().fromSec(v8[0]);
	p.header.frame_id = "world";
	p.pose.position.x = v8[1];
	p.pose.position.y = v8[2];
	p.pose.position.z = v8[3];
	// p.pose.orientation.x = v8[4];
	// p.pose.orientation.y = v8[5];
	// p.pose.orientation.z = v8[6];
	// p.pose.orientation.w = v8[7];

	p.pose.orientation.w = v8[4];
	p.pose.orientation.x = v8[5];
	p.pose.orientation.y = v8[6];
	p.pose.orientation.z = v8[7];

	return p;
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
	vector<string> pcd_file_1;
	vector<PointCloud> pc_1;

	vector<string> pcd_file_2;
	vector<PointCloud> pc_2;
	
	int ok_cunt = 0;
	// pcl::GeneralizedIterativeClosestPoint<PointCloud, PointCloud> icp;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	double icp_tf_epsilon_ = 0.0000000001;
	double icp_corr_dist_ = 0.25;
	double icp_iterations_ = 30;
    cout << "sssssssssssss"<< endl;

	scan_dir_get_filename("/home/map/0924_1/pcd_lidar/", pcd_file_1);
	for (size_t i = 0; i < pcd_file_1.size(); i+=5)
	{
		cout << "pcd_file_1 i is: " << i << endl;

		PointCloud::Ptr cloud(new PointCloud);
		try
		{
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_1[i], *cloud) != -1)
				pc_1.push_back(*cloud);
			else
				continue;
		}
		catch (const std::exception &e)
		{
			// 某些点云数据 没有写完整，跳过该点云
			cout << "ERROR PCD : " << i << "th pcd : " << pcd_file_1[i] << endl;
			// PCL_ERROR("can not load pcd file!");
			std::cerr << e.what() << '\n';
			continue;
		}
	}

	scan_dir_get_filename("/home/map/0924_2/pcd_lidar/", pcd_file_2);
	for (size_t i = 0; i < pcd_file_2.size(); i+=5)
	{
		cout << "pcd_file_2 i is: " << i << endl;

		PointCloud::Ptr cloud(new PointCloud);
		try
		{
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_2[i], *cloud) != -1)
			{
				pc_2.push_back(*cloud);
			}
			else
				continue;
		}
		catch (const std::exception &e)
		{
			// 某些点云数据 没有写完整，跳过该点云
			cout << "ERROR PCD : " << i << "th pcd : " << pcd_file_2[i] << endl;
			// PCL_ERROR("can not load pcd file!");
			std::cerr << e.what() << '\n';
			continue;
		}
	}

	for (size_t i = 0; i < pc_1.size(); i++)
	{
		for (size_t j = 0; j < pc_2.size(); j++)
		{
			// cout << "i ,j  is: " << i << "  " << j << endl;

			icp.setTransformationEpsilon(icp_tf_epsilon_);
			icp.setMaxCorrespondenceDistance(icp_corr_dist_);
			icp.setMaximumIterations(icp_iterations_);
			icp.setRANSACIterations(50);

			icp.setInputSource(pc_1[i].makeShared()); // source
			icp.setInputTarget(pc_2[j].makeShared());

			// Perform ICP.
			// PointCloud unused_result;
			icp.align(pc_1[i]); //将2个关键帧配准

			if (!icp.hasConverged())
			{
				dzlog_info("icp isn't coverged");
				continue;
			}

			if (icp.getFitnessScore() > 0.36)
			{
				dzlog_info("score %f ", icp.getFitnessScore());
				// continue;
			}

			ok_cunt++;
			cout << "ok icp is" << ok_cunt << endl;
			
		}
	}
			cout << "ok icp is" << ok_cunt << endl;
			cout << "ok icp is" << ok_cunt << endl;
			cout << "ok icp is" << ok_cunt << endl;


	/*
		//初始化zlog
		if (0 == initZlog())
		{
			dzlog_info("@@@@@@ loopClosure init zlog success !!!");
		}
		std::cout << " Initialize  : " << std::endl;

		// ros::init(argc, argv, "optimize_map_gtsam");
		// ros::NodeHandle nh;
		// std::string work_dir = "/home/map/0817_0826/";
		std::string work_dir = "/home/map/0805_big_imu_noise_pose_graph_0_0/";

		if (argc > 1)
		{
			work_dir = argv[1];
		}

		std::cout << "Your work dir is : " << work_dir << std::endl;
		ifstream infile_loop_path;
		infile_loop_path.open(work_dir + "vins_result_loop.txt", ios::in);

		if (!infile_loop_path.is_open())
		{
			std::cout << "124 读取文件失败" << std::endl;
			return 0;
		}

		const char *delim = " ";
		char buf[1024];

		// 获取 loop_path 的位姿
		std::vector<geometry_msgs::PoseStamped> loop_pose;
		while (infile_loop_path.getline(buf, sizeof(buf)))
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
		infile_loop_path.close();

		// 获取 lio_path 的位姿
		ifstream infile_lio;

		infile_lio.open(work_dir + "lio_path.txt", ios::in);
		// infile_lio.open(work_dir + "lio_optimized_path.txt", ios::in);
		if (!infile_lio.is_open())
		{
			std::cout << "156 读取文件失败" << std::endl;
			return 0;
		}

		std::vector<geometry_msgs::PoseStamped> lio_pose;
		while (infile_lio.getline(buf, sizeof(buf)))
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
			lio_pose.push_back(to_PoseStamped(item_8));
		}
		infile_lio.close();

		cout << "lio_pose size is: " << lio_pose.size() << endl;

		double start_time = loop_pose[0].header.stamp.toSec();
		double end_time = loop_pose.back().header.stamp.toSec();
		cout << "start_time : " << std::to_string(start_time) << endl;
		cout << "end_time : " << std::to_string(end_time) << endl;

		vector<string> pcd_file;

		// 这个读取的顺序是对的
		scan_dir_get_filename(work_dir + "pcd_lidar/", pcd_file);

		PointCloud::Ptr map(new PointCloud);

		pcl::visualization::CloudViewer viewer("Cloud Viewer");

		cout << "size of pcd : " << pcd_file.size() << endl;
		cout << "size of lio_pose : " << lio_pose.size() << endl;

		LaserLoopClosure llc;
		llc.Initialize();

		// cout <<  "th pcd : " << pcd_file[696] << endl;
		// cout <<  "th pcd : " << pcd_file[1448] << endl;

		int cnts = 0;
		int keyframe_cnts = 0;

		const int step_len = 1;

		bool jump = false;

		// 跳过中间没有回环的部分，直接看回环部分的点云
		// 0817
		// const int jump_start = 2000;
		// const int jump_end = pcd_file.size() - 1500;

		// 0805
		// const int jump_start = 450;
		// const int jump_end = pcd_file.size() - 1600;

		const int jump_start = 450;
		const int jump_end = pcd_file.size() - 2500;
		ROS_INFO("sssssssss");

		// for (int i = 0; i <  pcd_file.size() - 1200; i += step_len)
		for (int i = 10; i < pcd_file.size() - 20; i += step_len)
		// for (int i = 0; i < 2100 ; i += step_len)
		{
			geometry_utils::Transform3 pose_jump;

			if (i > jump_start && i < jump_start + 100)
			{
				i = jump_end;
				jump = true;

				gu::Vector3Base<double> posi_jump(lio_pose[jump_start].pose.position.x,
												  lio_pose[jump_start].pose.position.y,
												  lio_pose[jump_start].pose.position.z);
				gu::Rotation3Base<double> ori_jump(gu::QuaternionBase<double>(lio_pose[jump_start].pose.orientation.w,
																			  lio_pose[jump_start].pose.orientation.x,
																			  lio_pose[jump_start].pose.orientation.y,
																			  lio_pose[jump_start].pose.orientation.z));
				geometry_utils::Transform3 pose_jump_temp(posi_jump, ori_jump);
				pose_jump = pose_jump_temp;
			}

			auto pos_last_g = pcd_file[i].find_last_of("/") + 1;
			auto pos_last_d = pcd_file[i].find_last_of(".") + 1;
			// 从文件名字里，获取时间
			double pcd_time = std::stold(pcd_file[i].substr(pos_last_g, pos_last_d - pos_last_g - 1));

			double pose_time = lio_pose[i].header.stamp.toSec();

			// cout << i << "th pcd : " << pcd_file[i] << endl;
			PointCloud::Ptr cloud(new PointCloud);
			try
			{
				if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file[i], *cloud) != -1)
					cnts++;
				else
					continue;
			}
			catch (const std::exception &e)
			{
				// 某些点云数据 没有写完整，跳过该点云
				PCL_ERROR("can not load pcd file!");
				std::cerr << e.what() << '\n';
				continue;
			}

			if (cloud->points.empty())
				continue;

			// 第一帧点云 单独插入进去
			if (keyframe_cnts == 0)
			{
				gu::Vector3Base<double> posi_if(lio_pose[i].pose.position.x,
												lio_pose[i].pose.position.y,
												lio_pose[i].pose.position.z);
				gu::Rotation3Base<double> ori_if(gu::QuaternionBase<double>(lio_pose[i].pose.orientation.w,
																			lio_pose[i].pose.orientation.x,
																			lio_pose[i].pose.orientation.y,
																			lio_pose[i].pose.orientation.z));
				geometry_utils::Transform3 pose_if(posi_if, ori_if);
				llc.addPoseAndKeyScan(pose_if, cloud);
				keyframe_cnts++;

				continue;
			}

			gu::Vector3Base<double> posi_i(lio_pose[i - step_len].pose.position.x,
										   lio_pose[i - step_len].pose.position.y,
										   lio_pose[i - step_len].pose.position.z);
			gu::Rotation3Base<double> ori_i(gu::QuaternionBase<double>(lio_pose[i - step_len].pose.orientation.w,
																	   lio_pose[i - step_len].pose.orientation.x,
																	   lio_pose[i - step_len].pose.orientation.y,
																	   lio_pose[i - step_len].pose.orientation.z));
			geometry_utils::Transform3 pose_i(posi_i, ori_i);

			gu::Vector3Base<double> posi_ip1(lio_pose[i].pose.position.x,
											 lio_pose[i].pose.position.y,
											 lio_pose[i].pose.position.z);
			gu::Rotation3Base<double> ori_ip1(gu::QuaternionBase<double>(lio_pose[i].pose.orientation.w,
																		 lio_pose[i].pose.orientation.x,
																		 lio_pose[i].pose.orientation.y,
																		 lio_pose[i].pose.orientation.z));
			geometry_utils::Transform3 pose_ip1(posi_ip1, ori_ip1);

			if (jump == true)
			{
				cout << "----------------------JUMP ----________---------------------------- " << endl;
				pose_i = pose_jump;
			}

			geometry_utils::Transform3 delta = gu::PoseDelta(pose_i, pose_ip1);

			if (jump == true)
			{
				cout << "----------------------delta is  :----------------------- " << endl
					 << delta << endl;
				jump = false;
			}

			// 这一个点云的 header
			cloud->header.stamp = pcd_time;
			cloud->header.frame_id = lio_pose[i].header.frame_id;
			// bool addPoseAndKeyScan(const geometry_utils::Transform3 &delta,const PointCloud::ConstPtr &scan);

			if (llc.addPoseAndKeyScan(delta, cloud))
			{

				keyframe_cnts++;

				// 可视化关键帧的点云
				Eigen::Vector3d now_pcd_tans = Eigen::Vector3d(lio_pose[i].pose.position.x,
															   lio_pose[i].pose.position.y,
															   lio_pose[i].pose.position.z);

				Eigen::Quaterniond now_pcd_rot = Eigen::Quaterniond(lio_pose[i].pose.orientation.w,
																	lio_pose[i].pose.orientation.x,
																	lio_pose[i].pose.orientation.y,
																	lio_pose[i].pose.orientation.z);

				// 会覆盖已经有的文件
				string file_temp = work_dir + "keyframe_pose.txt";
				static std::ofstream r2live_relo_relative_pose(file_temp, ios::out);

				// 往上面的文件里写 打开
				r2live_relo_relative_pose.open(file_temp, ios::app);

				r2live_relo_relative_pose.setf(ios::fixed, ios::floatfield);
				r2live_relo_relative_pose.precision(10);
				r2live_relo_relative_pose << pcd_time << " ";
				r2live_relo_relative_pose.precision(5);

				r2live_relo_relative_pose
					<< now_pcd_tans.x() << " "
					<< now_pcd_tans.y() << " "
					<< now_pcd_tans.z() << " "
					<< now_pcd_rot.x() << " "
					<< now_pcd_rot.y() << " "
					<< now_pcd_rot.z() << " "
					<< now_pcd_rot.w() << endl;

				// 把上面的文件 关闭
				r2live_relo_relative_pose.close();

				if (keyframe_cnts % 3 != 0)
				{
					continue;
				}

				cout << keyframe_cnts << "th keyframe_cnts  : " << i << "th pointclouds " << endl;

				PointCloud::Ptr transformed_cloud(new PointCloud);
				transformed_cloud->points.resize(cloud->points.size());

				// static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.05512, 0.02226, 0.0297); // Horizon r2live
				// 官方 https://livox-wiki-cn.readthedocs.io/zh_CN/latest/introduction/Point_Cloud_Characteristics_and_Coordinate_System%20.html#id2
				// static const Eigen::Vector3d Lidar_offset_to_IMU_temp(-0.0847, -0.0425, -0.0353); // -84.7，-42.5，-35.3 // one
				static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.0847, 0.0425, 0.0353); // -84.7，-42.5，-35.3 // two
				// static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.05512, 0.02226, 0.0297); // Horizon r2live three
				//    static const Eigen::Vector3d Lidar_offset_to_IMU_temp(-0.05512, -0.02226, -0.0297); // Horizon r2live four
				//    static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.0, 0.0, 0.0); //  zero
				//    static const Eigen::Vector3d Lidar_offset_to_IMU_temp(-0.0575, -0.0375, 0.0425); //  zero

				// 构建变换矩阵
				Eigen::Isometry3d TM = Eigen::Isometry3d::Identity();
				TM.pretranslate(now_pcd_tans);
				TM.rotate(now_pcd_rot.matrix());
				// cout << TM.matrix() << endl;

				for (int in = 0; in < cloud->points.size(); in++)
				{
					if (cloud->points[in].x < 0)
					{
						cloud->points.clear();
						cout << "ERROR PCD <  x < 0 . -_- >. " << pcd_file[i] << endl;
						break;
					}
					cloud->points[in].x += Lidar_offset_to_IMU_temp[0];
					cloud->points[in].y += Lidar_offset_to_IMU_temp[1];
					cloud->points[in].z += Lidar_offset_to_IMU_temp[2];
				}
				pcl::transformPointCloud(*cloud, *transformed_cloud, TM.matrix());

				(*map) += (*transformed_cloud);

				// viewer.showCloud(transformed_cloud);
				viewer.showCloud(map);
				// boost::this_thread::sleep(boost::posix_time::microseconds(2));
			}
		}
		ROS_INFO("eeeeeeeeee");

		std::cout << "Your work  is DOEN .pointcloud cnts is : " << cnts << std::endl;
		cout << "keyframe_cnts is : " << keyframe_cnts << endl;

		std::string name;
		name = work_dir + "keyframe_point_original.pcd";
		pcl::io::savePCDFile(name, *map);
		cout << "path is :" << name << endl;

		llc.saveGtsam2G2oFile(work_dir + "loop_gtsam_original.g2o");

		cout << "wait cmd (s is save map .) : " << keyframe_cnts << endl;
		while (1)
		{
			char ch = getchar();
			if ('s' == ch)
			{
				llc.saveMap();
				llc.saveGtsam2G2oFile(work_dir + "loop_gtsam_optimized.g2o");
				break;
			}
			usleep(1000 * 1000);
		}
	*/
	return 1;
}

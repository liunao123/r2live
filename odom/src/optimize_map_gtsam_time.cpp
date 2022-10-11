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

#include <odom/LaserLoopClosure_time.h>
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
	cout << "size of pcd : " << filenames.size() << endl;
}

geometry_msgs::PoseStamped to_PoseStamped(std::vector<double> v8)
{
	geometry_msgs::PoseStamped p;
	p.header.stamp = ros::Time().fromSec(v8[0]);
	p.header.frame_id = "world";
	p.pose.position.x = v8[1];
	p.pose.position.y = v8[2];
	p.pose.position.z = v8[3];
	p.pose.orientation.x = v8[4];
	p.pose.orientation.y = v8[5];
	p.pose.orientation.z = v8[6];
	p.pose.orientation.w = v8[7];
	
	// p.pose.orientation.w = v8[4];
	// p.pose.orientation.x = v8[5];
	// p.pose.orientation.y = v8[6];
	// p.pose.orientation.z = v8[7];

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
	// enum Roster { Tom = 1, Sharon, Bill, Teresa, John };
	// std::cout << "   : " << std::to_string(Teresa)  <<  std::endl;
	// return 0;

    //初始化zlog
	if (0 == initZlog())
	{
		dzlog_info("@@@@@@ loopClosure init zlog success !!!");
	}
	std::cout << " Initialize  : " << std::endl;

	// ros::init(argc, argv, "optimize_map_gtsam");
	// ros::NodeHandle nh;
	// std::string work_dir = "/home/map/0805-less-drift/";
	// std::string work_dir = "/home/map/0817_0826/";
	std::string work_dir = "/home/map/0916_2000s/";
	
	if (argc > 1)
	{
		work_dir = argv[1];
	}

	std::cout << "Your work dir is : " << work_dir << std::endl;
	ifstream infile_loop_path;
	infile_loop_path.open(work_dir + "vins_result_loop.txt", ios::in);

	if (!infile_loop_path.is_open())
	{
		std::cout << __LINE__ << " 读取文件失败" << std::endl;
		return 0;
	}

	const char *delim = " ";
	char buf[1024];

	// 获取 lio_path 的位姿
	ifstream infile_lio;
	infile_lio.open(work_dir + "lio_path.txt", ios::in);
	// infile_lio.open(work_dir + "lio_optimized_path.txt", ios::in);
	if (!infile_lio.is_open())
	{
		std::cout << __LINE__ << " 读取文件失败" << std::endl;
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

	PointCloud::Ptr map(new PointCloud);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	// 这个读取的顺序是对的
	vector<string> pcd_file;
	scan_dir_get_filename(work_dir + "pcd_lidar/", pcd_file);


	// 获取 视觉回环的时间
	ifstream loop_time_file(work_dir + "vins_result_loop_time.txt", ios::in);
	if (!loop_time_file.is_open())
	{
		std::cout << "156 读取文件失败" << std::endl;
		return 0;
	}
	// 获取 loop_time_file 的 时间
	std::vector<std::pair<double, double>> loop_time;
	double last_loop_time_s = 0;
	double last_loop_time_f = 0;
	while (loop_time_file.getline(buf, sizeof(buf)))
	{
		char *p;
		p = strtok(buf, delim);
		double f = stold(string(p));
		p = strtok(NULL, delim);
		double s = stold(string(p));
		// std::cout << string(p) << std::endl;
		std::pair<double, double> pair_temp(f, s);

		// 两个视觉回环 视觉 相隔 大于 5秒，才加进去一个
		const float loop_time_gap = 1.0;
		if ( fabs( f - last_loop_time_f ) > loop_time_gap || fabs( s - last_loop_time_s ) > loop_time_gap )
		{
			loop_time.push_back(pair_temp);
            last_loop_time_f = f;
            last_loop_time_s = s;
	    	// std::cout <<  setiosflags(ios::fixed) << f << " " << s << std::endl;
		}
	}
	loop_time_file.close();

    std::cout << loop_time.size() << std::endl;

	// return 0;

	// 回环检测的 对象
	LaserLoopClosure llc;
	llc.Initialize();
	// 把视觉回环的时间放进去
	llc.setVisionLoopTime(loop_time);
	llc.setWorkPath(work_dir);
	
	int _start = 0 ;
	int _end = pcd_file.size() ;

	int jump_start = pcd_file.size() + 100 ;
	int jump_end =  0;

	if (argc == 6)
	{
		_start = std::stoi(argv[2]);
		_end =std::stoi(argv[3]);
	    jump_start = std::stoi(argv[4]);
	    jump_end = std::stoi(argv[5]);
	}

	// 跳过中间没有回环的部分，直接看回环部分的点云
	// 0908
	//  int jump_start = 3000;
	//  int jump_end = 12500;


	// 0805
	// const int jump_start = 600;
	// const int jump_end = pcd_file.size() - 2500;
	ROS_INFO("sssssssss");

	int step_len = 2;
	int cnts = 0;
	int keyframe_cnts = 0;
	bool jump = false;

	srand(time(0));
	for (int i =  _start ; i <  pcd_file.size() - _end ; i += step_len)
	{
		geometry_utils::Transform3 pose_jump;

		if ( i > jump_start && i < jump_end  )
		{
			i = jump_end;
	        jump = true;

	    	gu::Vector3Base<double> posi_jump(lio_pose[jump_start].pose.position.x,
	    								   lio_pose[jump_start].pose.position.y,
	    								   lio_pose[jump_start].pose.position.z);
	    	gu::Rotation3Base<double> ori_jump(gu::QuaternionBase<double>(lio_pose[jump_start].pose.orientation.w,
	    															   lio_pose[ jump_start ].pose.orientation.x,
	    															   lio_pose[ jump_start ].pose.orientation.y,
	    															   lio_pose[ jump_start ].pose.orientation.z));
	    	geometry_utils::Transform3 pose_jump_temp(posi_jump, ori_jump);
	    	pose_jump = pose_jump_temp;
		}

    	// const int jump_start_2 = 9600 ;
		// if ( i > jump_start_2 && i < jump_start_2 + 100 ) 
		// {   
		// 	jump_start = jump_start_2;
		// 	i = 12900;
	    //     jump = true;

	    // 	gu::Vector3Base<double> posi_jump(lio_pose[jump_start].pose.position.x,
	    // 								   lio_pose[jump_start].pose.position.y,
	    // 								   lio_pose[jump_start].pose.position.z);
	    // 	gu::Rotation3Base<double> ori_jump(gu::QuaternionBase<double>(lio_pose[jump_start].pose.orientation.w,
	    // 															   lio_pose[ jump_start ].pose.orientation.x,
	    // 															   lio_pose[ jump_start ].pose.orientation.y,
	    // 															   lio_pose[ jump_start ].pose.orientation.z));
	    // 	geometry_utils::Transform3 pose_jump_temp(posi_jump, ori_jump);
	    // 	pose_jump = pose_jump_temp;
		// }

		// const int jump_start_3 = 13500 ;
		// if ( i > jump_start_3 && i < jump_start_3 + 100 ) 
		// {   
		// 	jump_start = jump_start_3;
		// 	i = 16600;
	    //     jump = true;

	    // 	gu::Vector3Base<double> posi_jump(lio_pose[jump_start].pose.position.x,
	    // 								   lio_pose[jump_start].pose.position.y,
	    // 								   lio_pose[jump_start].pose.position.z);
	    // 	gu::Rotation3Base<double> ori_jump(gu::QuaternionBase<double>(lio_pose[jump_start].pose.orientation.w,
	    // 															   lio_pose[ jump_start ].pose.orientation.x,
	    // 															   lio_pose[ jump_start ].pose.orientation.y,
	    // 															   lio_pose[ jump_start ].pose.orientation.z));
	    // 	geometry_utils::Transform3 pose_jump_temp(posi_jump, ori_jump);
	    // 	pose_jump = pose_jump_temp;
		// }

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

			static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.05512, 0.02226, 0.0297); // Horizon r2live
			// 官方 https://livox-wiki-cn.readthedocs.io/zh_CN/latest/introduction/Point_Cloud_Characteristics_and_Coordinate_System%20.html#id2
			// static const Eigen::Vector3d Lidar_offset_to_IMU_temp(-0.0847, -0.0425, -0.0353); // -84.7，-42.5，-35.3 // one
			// static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.0847, 0.0425, 0.0353); // -84.7，-42.5，-35.3 // two
			// static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.05512, 0.02226, 0.0297); // Horizon r2live three
			//    static const Eigen::Vector3d Lidar_offset_to_IMU_temp(-0.05512, -0.02226, -0.0297); // Horizon r2live four
			//    static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.0, 0.0, 0.0); //  zero
			//    static const Eigen::Vector3d Lidar_offset_to_IMU_temp(-0.0575, -0.0375, 0.0425); //  zero

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
			cout <<  "----------------------JUMP -------------------------------- " << endl ;
			pose_i = pose_jump;
		}

		geometry_utils::Transform3 delta = gu::PoseDelta(pose_i, pose_ip1);

		if (jump == true)
		{
			cout <<  "----------------------delta is  :----------------------- " << endl << delta << endl;
	        jump = false;
		}

		// 这一个点云的 header
		cloud->header.stamp = pcd_time;
		cloud->header.frame_id = lio_pose[i].header.frame_id;
		//  保证每次的step不一样，这样不会每次都用同样的点云。更合理一点
		step_len = rand()%2 + 2; //[4,5]
    	// ROS_INFO("step_len : %d .",step_len );
		// if( i > 2000 && i < 22000)
		// {
		//     step_len = 4;
		// 	llc.setTranslationThreshold(0.75);
		// }
		// else
		// {
		//     step_len = 3;
		// 	llc.setTranslationThreshold(0.5);
		// }


		if (llc.addPoseAndKeyScan(delta, cloud))
		// if (1) 
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

			// 抽样 显示
			if (keyframe_cnts % 20 != 0)
				continue;
			
			cout << keyframe_cnts << "th keyframe_cnts  : " << i << "th pointclouds " << "step_len: " << step_len <<  endl;
			dzlog_info("%dth keyframe_cnts, %dth pointclouds.", keyframe_cnts, i);


			PointCloud::Ptr transformed_cloud(new PointCloud);
			transformed_cloud->points.resize(cloud->points.size());

			// 构建变换矩阵
			Eigen::Isometry3d TM = Eigen::Isometry3d::Identity();
			TM.pretranslate(now_pcd_tans);
			TM.rotate(now_pcd_rot.matrix());
			// cout << TM.matrix() << endl;
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
		// 堵塞在这 等着 键盘的输入
		if ( llc.getIsLoopThreadExit() )
		{
			break;
		}
		usleep(1000*1000);
	}

	return 1;
}

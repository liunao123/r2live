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
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
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


using namespace std;
using namespace Eigen;

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

void scan_dir_get_filename(string path, vector<string> &filenames)
{
	struct dirent **entry_list;
    int count;
    int i;

    count = scandir(path.c_str(), &entry_list, 0, alphasort);
    if (count < 0) {
        perror("scandir");
    }
 
    for (i = 0; i < count; i++) {
        struct dirent *entry; 
        entry = entry_list[i];
        // printf("%s\n", entry->d_name);
		// 跳过 ./ 和 ../ 两个目录
		if (i < 2)
		{
			continue;
		}		
		filenames.push_back( path + std::string( entry->d_name ));
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
	p.pose.orientation.x = v8[4];
	p.pose.orientation.y = v8[5];
	p.pose.orientation.z = v8[6];
	p.pose.orientation.w = v8[7];
	return p;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "publisher_pose_loop");    
    ros::NodeHandle nh;
	// ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("loop_pose",1000);
    
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("loop_odom", 50);

	std::string work_dir = "/home/map/0805_big_imu_noise_pose_graph_0_0/";
	if (argc > 1)
	{
    	work_dir = argv[1];
	}
// return 0;
	
    std::cout << "Your work dir is : " << work_dir << std::endl;

	ifstream infile_loop_path;
	infile_loop_path.open(work_dir + "vins_result_loop.txt", ios::in);
	
	if (!infile_loop_path.is_open())
	{
		std::cout << "读取文件失败" << std::endl;
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


	// 	nav_msgs::Odometry odom;
    // odom.header.stamp = loop_pose.back().header.stamp;
    // odom.header.frame_id = "world";
    // odom.child_frame_id = "base_link";
 
    //    //set the position
    //    odom.pose.pose = loop_pose.back().pose;
 
    //    //set the velocity
	// 	odom_pub.publish( odom  );

	// 	ros::Duration(0.1).sleep();
    //     cout << "pub : " << endl;

	}
    infile_loop_path.close();
    // return 0;

    // 获取 lio_path 的位姿
	ifstream infile_lio;
	infile_lio.open( work_dir + "lio_path.txt", ios::in);
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

    cout << "lio_pose size is : " << lio_pose.size() << endl;

	double start_time = loop_pose[0].header.stamp.toSec();
	double end_time = loop_pose.back().header.stamp.toSec();
	cout << "start_time : " << std::to_string(start_time) << endl;
	cout << "end_time : " << std::to_string(end_time) << endl;

	vector<string> pcd_file;

	// 这个读取的顺序是对的 
	scan_dir_get_filename(work_dir + "pcd_lidar/", pcd_file);

	pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
    
	// pcl::visualization::CloudViewer viewer("Cloud Viewer");

	cout << "size of pcd : " << pcd_file.size() << endl;
	cout << "size of lio_pose : " << lio_pose.size() << endl;

	int cnts_ = 0;
	
    // for (int i = 0; i < pcd_file.size(); i += 1 )
    for (int i = 6000; i < 7050; i += 5 )
	{
		// 跳过中间没有回环的部分，直接看回环部分的点云
		// if(i == 1000 )
		//     i = 5000;

		auto pos_last_g = pcd_file[i].find_last_of("/") + 1 ;
		auto pos_last_d = pcd_file[i].find_last_of(".") + 1 ;
		// 从文件名字里，获取时间
		double pcd_time = std::stold( pcd_file[i].substr(pos_last_g , pos_last_d - pos_last_g - 1 ) );

		double pose_time = lio_pose[i].header.stamp.toSec();

		if ( std::fabs( pcd_time - pose_time ) > 0.1 )
		{
        	cout  <<  "pcd 时间与其 pose 时间不对应 " << i + 1 << pcd_time - pose_time << endl;
			continue;
		}

		if (pcd_time < start_time || pcd_time > end_time)
		{
        	cout  <<  "this pcd time is out the loop path time : " << pcd_file[i].substr(pos_last_g , pos_last_d - pos_last_g - 1 ) << endl;
    	    cout  <<  i << "th pcd : " << pcd_file[i] << endl;
			continue;
		}
		else
		{

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
					// cout <<  "  dt : " << dt   << " dt_before " << dt_before << "  lamda :" << dt_before / dt << endl;

					// 对平移插值
					Eigen::Vector3d
						start_pos = Eigen::Vector3d(loop_pose[j].pose.position.x,
													loop_pose[j].pose.position.y,
													loop_pose[j].pose.position.z);
					Eigen::Vector3d
						end_pos = Eigen::Vector3d(loop_pose[j + 1].pose.position.x,
												  loop_pose[j + 1].pose.position.y,
												  loop_pose[j + 1].pose.position.z);

			        Eigen::Vector3d now_pcd_tans = start_pos + (end_pos - start_pos) * (dt_before / dt);

					auto diff_posi = (end_pos - start_pos).transpose() ;

					// if(diff_posi.norm() > 0.5 )
					// {				
					// 	cnts_++;
					// 	// cout << "start_pos : " << start_pos.transpose() << endl;
					// 	// cout << "end_pos : " << end_pos.transpose() << endl;
					// 	cout << "diff_posi : " << diff_posi << endl;
    				// 	break;
					// }


					double t =  180.0 / 3.1415926;
					// 对旋转插值
					Eigen::Quaterniond start_ori = Eigen::Quaterniond(loop_pose[j].pose.orientation.w,
														loop_pose[j].pose.orientation.x,
														loop_pose[j].pose.orientation.y,
														loop_pose[j].pose.orientation.z);
					start_ori.normalize ();
														

					Eigen::Quaterniond end_ori = Eigen::Quaterniond(loop_pose[j + 1].pose.orientation.w,
													  loop_pose[j + 1].pose.orientation.x,
													  loop_pose[j + 1].pose.orientation.y,
													  loop_pose[j + 1].pose.orientation.z);
					end_ori.normalize ();
					
					Eigen::Quaternion<double> now_pcd_rot = start_ori.slerp(  dt_before / dt  , end_ori );
					now_pcd_rot.normalize ();

                    Eigen::Vector3d start_eulerAngle = start_ori.matrix().eulerAngles(0,1,2) * t ;
					Eigen::Vector3d now_eigen_eulerAngle = now_pcd_rot.matrix().eulerAngles(0,1,2) * t ;
                    Eigen::Vector3d end_eulerAngle = end_ori.matrix().eulerAngles(0,1,2) * t ;
                    
					cout <<"start_eulerAngle   : " << start_eulerAngle.transpose() << endl;
					cout <<"now_eigen_eulerAngle   : " << now_eigen_eulerAngle.transpose() << endl;
					cout <<"end_eulerAngle   : " << end_eulerAngle.transpose() << endl;


                    double tf_roll, tf_pitch, tf_yaw;
                    tf::Quaternion stf_RQ2(start_ori.x(), start_ori.y(), start_ori.z(), start_ori.w());					
                    tf::Matrix3x3(stf_RQ2).getRPY(tf_roll, tf_pitch, tf_yaw);
                    printf("start_ori tf eulerAngle: roll=%f ,pitch=%f ,yaw=%f .\n",tf_roll*t,tf_pitch*t,tf_yaw*t);

                    tf::Quaternion ntf_RQ2(now_pcd_rot.x(), now_pcd_rot.y(), now_pcd_rot.z(), now_pcd_rot.w());					
                    tf::Matrix3x3(ntf_RQ2).getRPY(tf_roll, tf_pitch, tf_yaw);
                    printf("now tf eulerAngle: roll=%f ,pitch=%f ,yaw=%f .\n",tf_roll*t,tf_pitch*t,tf_yaw*t);

					tf::Quaternion etf_RQ2(end_ori.x(), end_ori.y(), end_ori.z(), end_ori.w());					
                    tf::Matrix3x3(etf_RQ2).getRPY(tf_roll, tf_pitch, tf_yaw);
                    printf("end_ori tf eulerAngle: roll=%f ,pitch=%f ,yaw=%f .\n",tf_roll*t,tf_pitch*t,tf_yaw*t);

					printf("\n");

/*
					// 2,1,0 的顺序对应 绕ZYX的旋转

					auto diff_ang = (end_eulerAngle - start_eulerAngle).transpose() ;

                    tf::Quaternion start_RQ2;
                    tf::Quaternion end_RQ2;
                    double start_roll,start_pitch,start_yaw;
                    double end_roll,end_pitch,end_yaw;

					for (int item = 0; item < 3; item++)
					{

					    if( fabs(diff_ang(item)) > 160  and fabs(diff_ang(item)) < 200)
					    {
					    	cnts_++;
					    	cout <<"i : " << i << " item : " << item << "  diff_ang : " << diff_ang(item) << endl;

													                    //输入四元数，转化成欧拉角数在终端输出
                    tf::quaternionMsgToTF(loop_pose[j].pose.orientation,start_RQ2); 
                    tf::Matrix3x3(start_RQ2).getRPY(start_roll,start_pitch,start_yaw);  
                    ROS_INFO("[j] eulerAngle: roll=%f ,pitch=%f ,yaw=%f .",start_roll*t,start_pitch*t,start_yaw*t);

                    tf::quaternionMsgToTF(loop_pose[j + 1].pose.orientation,end_RQ2); 
                    tf::Matrix3x3(end_RQ2).getRPY(end_roll,end_pitch,end_yaw);  
                    ROS_INFO("[j+1]eulerAngle: roll=%f ,pitch=%f ,yaw=%f .\n",end_roll*t,end_pitch*t,end_yaw*t);
							
							if (item == 2)
							{
					    	    cout << "start_eulerAngle : " << start_eulerAngle.transpose() << endl;
					    	    cout << "  end_eulerAngle : " << end_eulerAngle.transpose() << endl;
							}			

					    }
					}
*/



					// 对平移插值 只用 了  Z 方向上的值
					// now_pcd_tans(0) = 	lio_pose[i].pose.position.x;
					// now_pcd_tans(1) = 	lio_pose[i].pose.position.y;
					// now_pcd_tans(2) = 	lio_pose[i].pose.position.z;

				    // Eigen::Quaterniond now_pcd_rot = Eigen::Quaterniond(lio_pose[i].pose.orientation.w,
					// 									  lio_pose[i].pose.orientation.x,
					// 									  lio_pose[i].pose.orientation.y,
					// 									  lio_pose[i].pose.orientation.z);


                    // 当前点云，离那个位姿近（时间上的近），就用哪一个姿态
					// int which_one = j;
					// if (fabs(dt_before) <= fabs(dt_after) )
					// 	which_one = j;
					// else
					// 	which_one = j + 1;
					// now_pcd_tans = Eigen::Vector3d(loop_pose[which_one].pose.position.x,
					// 							  loop_pose[which_one].pose.position.y,
					// 							  loop_pose[which_one].pose.position.z);
					// now_pcd_rot = Eigen::Quaterniond(loop_pose[which_one].pose.orientation.w,
					// 									  loop_pose[which_one].pose.orientation.x,
					// 									  loop_pose[which_one].pose.orientation.y,
					// 									  loop_pose[which_one].pose.orientation.z);

					// 根据上述 旋转和平移构造对应的 变换矩阵，把点云 投影过去即可。
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		            try
		            {
		            	pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file[i], *cloud) ;
		            }
		            catch(const std::exception& e)
		            {
						// 某些点云数据 没有写完整，跳过该点云
		            	PCL_ERROR("can not load pcd file!");
		            	std::cerr << e.what() << '\n';
		            	cnts_++;
						break;
		            }

					pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
					transformed_cloud->points.resize(cloud->points.size());

                    // static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.05512, 0.02226, 0.0297); // Horizon r2live
					// 官方 https://livox-wiki-cn.readthedocs.io/zh_CN/latest/introduction/Point_Cloud_Characteristics_and_Coordinate_System%20.html#id2
                    // static const Eigen::Vector3d Lidar_offset_to_IMU_temp(-0.0847, -0.0425, -0.0353); // -84.7，-42.5，-35.3 // one
                    static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.0847, 0.0425, 0.0353); // -84.7，-42.5，-35.3 // two
                    // static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.05512, 0.02226, 0.0297); // Horizon r2live three
                    // static const Eigen::Vector3d Lidar_offset_to_IMU_temp(-0.05512, -0.02226, -0.0297); // Horizon r2live four
                    // static const Eigen::Vector3d Lidar_offset_to_IMU_temp(0.0, 0.0, 0.0); //  zero

					// 构建变换矩阵
					// Eigen::Isometry3d TM = Eigen::Isometry3d::Identity();
					// TM.pretranslate(now_pcd_tans);
					// TM.rotate(now_pcd_rot.matrix());
					// // cout << TM.matrix() << endl;

					// for (int in=0; in < cloud->points.size() ; in++)
					// {
                    //     cloud->points[in].x += Lidar_offset_to_IMU_temp[0];
                    //     cloud->points[in].y += Lidar_offset_to_IMU_temp[1];
                    //     cloud->points[in].z += Lidar_offset_to_IMU_temp[2];
					// }
					// pcl::transformPointCloud(*cloud, *transformed_cloud, TM.matrix() );

					// (*map) += (*transformed_cloud);
                    // viewer.showCloud( map );
		            // boost::this_thread::sleep(boost::posix_time::microseconds(1));

					break;
				}
			}

		}

	}

	cout << "--cnts_ ----DONE----- "  << cnts_<< endl;
	cout << " START WRITE PCD . "  << endl;

	// return 0;

    // while (!viewer.wasStopped())
    // {    }
	/*	*/

	pcl::VoxelGrid<pcl::PointXYZ> sor;

	sor.setInputCloud(map);
	float resolution = 0.1f;
	sor.setLeafSize(resolution, resolution, resolution);
	sor.filter(*map);

	std::string name;
	name = work_dir + "optimized_map_full.pcd";
	pcl::io::savePCDFile(name, *map);
	cout << "path is :" << name << endl;

	// sor.setInputCloud(map);
	// resolution = 0.2f;
	// sor.setLeafSize(resolution, resolution, resolution);
	// sor.filter(*map);
	// name = work_dir  + "optimized_map_" +  std::to_string(resolution) + ".pcd";
	// pcl::io::savePCDFile(name, *map);
	// cout << "path is :" << name << endl;
	return 0;
}

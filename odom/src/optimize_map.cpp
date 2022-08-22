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

// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>


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
	p.header.stamp = ros::Time().fromSec( v8[0] );
	p.pose.position.x = v8[1];
	p.pose.position.y = v8[2];
	p.pose.position.z = v8[3];
	p.pose.orientation.w = v8[4];
	p.pose.orientation.x = v8[5];
	p.pose.orientation.y = v8[6];
	p.pose.orientation.z = v8[7];
    // cout << p.header.stamp << " ";
	// 		for(int i=1;i<8;i++)
	// 		{
	// 			if(i == 7)
	// 			    cout << v8[i] ;
	// 			else
	// 			    cout << v8[i] << " ";
	// 		}
   	//         cout << endl  ;
	return p;
}


int main(int argc, char **argv)
{
	std::string work_dir = "/home/map/0817/";
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

	}
    infile_loop_path.close();

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

	int cnts_ = 0;

    for (int i = 0; i < pcd_file.size(); i += 1 )
	{
  	    cout  <<  i << "th pcd : " << pcd_file[i] << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		try
		{
			pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file[i], *cloud) ;
			/* code */
		}
		catch(const std::exception& e)
		{
			PCL_ERROR("can not load pcd file!");
			std::cerr << e.what() << '\n';
			cnts_++;
		}
	}
	
	cout << "--cnts_ ----DONE----- "  << cnts_<< endl;

	return 0;
}

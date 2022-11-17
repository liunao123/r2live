#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf/tf.h>


using namespace Eigen;
using namespace std;

ros::Publisher pub_repro_path;

Eigen::Isometry3d init_transform;

bool get_init_pose = false;

Eigen::Isometry3d getTfFromPose(geometry_msgs::Pose pose)
{
	Eigen::Vector3d now_pcd_tans = Eigen::Vector3d(pose.position.x,
											   pose.position.y,
											   pose.position.z);

	Eigen::Quaterniond now_pcd_rot = Eigen::Quaterniond(pose.orientation.w,
														pose.orientation.x,
														pose.orientation.y,
														pose.orientation.z);
	// 构建变换矩阵
	Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
	tf.pretranslate(now_pcd_tans);
	tf.rotate(now_pcd_rot.matrix());
	return tf;
}


void r2livePathCallback(const nav_msgs::PathConstPtr &r2live_path)
{
	if (! get_init_pose )
	{
		return ;
	}
	
	nav_msgs::Path repro_path;
	repro_path.header = r2live_path->header;
	repro_path.header.frame_id = "world";

	for ( const auto & one_pt : r2live_path->poses )
	{
		// auto one_pt = r2live_path->poses.back();
		// 两个变换矩阵相乘，即可得到在map坐标系下的位姿
		Eigen::Isometry3d tf_init = getTfFromPose( one_pt.pose );
		Eigen::Isometry3d tf_map = init_transform * tf_init;
		
	    // cout << "tf is : " << endl << tf_map.translation().transpose() << endl;
	    // cout << "tf is : " << endl << tf_map.rotation() << endl;
		geometry_msgs::PoseStamped pt;
		pt.header = one_pt.header;
	    pt.header.frame_id = repro_path.header.frame_id;

		pt.pose.position.x = tf_map.translation()[0];
		pt.pose.position.y = tf_map.translation()[1];
		pt.pose.position.z = tf_map.translation()[2];

		Eigen::Quaterniond qua_temp(tf_map.rotation());
		pt.pose.orientation.x = qua_temp.x();
		pt.pose.orientation.y = qua_temp.y();
		pt.pose.orientation.z = qua_temp.z();
		pt.pose.orientation.w = qua_temp.w();

	    repro_path.poses.push_back(pt);
	}
	pub_repro_path.publish(repro_path);
}

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initpose)
{
    init_transform = getTfFromPose(initpose->pose.pose);
	// 将接收到的消息打印出来
	cout << "initPose to transform is : " << endl << init_transform.matrix() << endl;
	get_init_pose = true;
	ROS_INFO("get initial pose ok .");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reproject_r2live_path");
	ROS_INFO("reproject_r2live_path node init ok .");

	ros::NodeHandle nh;

	init_transform = Eigen::Isometry3d::Identity();

	ros::Subscriber sub_init_pose = nh.subscribe("/initialpose", 10, initPoseCallback);

	ros::Subscriber sub_r2live_path = nh.subscribe("/r2live_locate/path", 10, r2livePathCallback);

	pub_repro_path = nh.advertise<nav_msgs::Path>("/relocate_path", 10);

	ros::spin();
	//           nav_msgs::Path path;
	//         path.header.stamp = ros::Time::now();
	//         path.header.frame_id = "/world";

	return 0;
}

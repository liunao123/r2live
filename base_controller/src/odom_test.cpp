/******************************************************************
为测试程序，可不使用
作用：模拟一个里程计的发布

*******************************************************************/
#include "../include/base_controller.h"
#include <ros/console.h>
#include "serial.cpp"
#include "math.h"
#include<ctime>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Empty.h>
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
Motion_Data_Typdef Motion_Data;
ros::Publisher odom_pub;  
nav_msgs::Odometry odom;
ros::Publisher slam_status_pub_;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_controller");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
    /***********************	nav_msgs::Odometry	********************************/
    odom_pub= n.advertise<nav_msgs::Odometry>("/odom", 200); //定义要发布/odom主题,之后用odom_pub.publish就可以了
    slam_status_pub_=n.advertise<std_msgs::Empty>("/blam/velodyne_slam/slam_status", 10, false);
    slam_status_pub_.publish(std_msgs::Empty());
    sleep(1);
    slam_status_pub_.publish(std_msgs::Empty());
    sleep(1);
    slam_status_pub_.publish(std_msgs::Empty());
    static tf::TransformBroadcaster odom_broadcaster;//定义tf对象
    geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息
    geometry_msgs::Quaternion odom_quat; //四元数变量,是要发布的odom的角度量
    //定义covariance矩阵，作用为解决文职和速度的不同测量的不确定性
    float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
                            0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
                            0,  0,    99999, 0,     0,     0,  // covariance on gps_z
                            0,  0,    0,     99999, 0,     0,  // large covariance on rot x
                            0,  0,    0,     0,     99999, 0,  // large covariance on rot y
                            0,  0,    0,     0,     0,     0.01};  // large covariance on rot z 
    //载入covariance矩阵
    for(int i = 0; i < 36; i++)
    {
        odom.pose.covariance[i] = covariance[i];;
    } 
    ros::Rate loop_rate(20);//设置数据接收频率
    while(ros::ok())
    {
        odom.header.stamp = ros::Time::now(); //载入里程计时间戳		
        odom.header.frame_id = "odom";			
        odom.child_frame_id = "base_footprint";  //base_footprint 
        odom.pose.pose.position.x =0.01;
        odom.pose.pose.position.y =0.02;
        odom.pose.pose.position.z = 0.0;
        odom.twist.twist.linear.x =0.00;			//载入线速度
        odom.twist.twist.angular.z =0.0;   		//载入角速度
        odom_quat = tf::createQuaternionMsgFromYaw(float(0.0));
        odom.pose.pose.orientation =odom_quat;  
//        ROS_INFO("odom.pose.pose.position.x: [%f]", (float) odom.pose.pose.position.x);
//        ROS_INFO("odom.pose.pose.position.y: [%f]", (float)odom.pose.pose.position.y);
//        //printf("Motion_Data.Status.Odom.Yaw: [%f]", (float)(Motion_Data.Status.Odom.Yaw/1000.0));
//        ROS_INFO("odom.twist.twist.linear.x: [%f]", (float)odom.twist.twist.linear.x);
//        ROS_INFO("odom.twist.twist.angular.z: [%f]", (float)odom.twist.twist.angular.z);
        odom_pub.publish(odom);
        //slam_status_pub_.publish(std_msgs::Empty());
        odom_trans.transform.translation.x =odom.pose.pose.position.x ;			//tf位置数据：x,y,z,方向
        odom_trans.transform.translation.y =odom.pose.pose.position.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;  
        odom_trans.header.stamp = ros::Time::now();								//载入坐标（tf）变换时间戳
        odom_trans.header.frame_id = "odom";									   //发布坐标变换的父子坐标系
        odom_trans.child_frame_id = "base_footprint";       
        odom_broadcaster.sendTransform(odom_trans);
        //ros::spinOnce();  //程序周期性调用			
        //loop_rate.sleep();//周期休眠
        //usleep(20*1000);
        sleep(3);
    }
    /*ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();*/
    return 0;
}

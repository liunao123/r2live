/******************************************************************
作用：此程序为测试程序，可不编译，也可不使用。
用于测试导航，这个可模拟小车的实际的导航行为。根据导航发布的速度模拟出一个小车的运动轨迹，可大大节省时间。
*******************************************************************/
#include "../include/base_controller.h"
#include "math.h"
#include<ctime>
#include <tf/transform_broadcaster.h>
#include<base_controller/csgPoseStampedMatchValue.h>
#include <iostream>
#include <iterator>
#include <random>
/****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
ros::Publisher currentPub_;
ros::Publisher kedaCurrentPub_;
double x_ = 0.0;
double y_ = 0.0;
double theta_ = 0.0;
geometry_msgs::Twist last_vel;
void callback(const geometry_msgs::Twist & cmd_input)
{ 
    double delta_x = (cmd_input.linear.x * cos(theta_) - cmd_input.linear.y *
    sin(theta_)) * 0.2;
    double delta_y = (cmd_input.linear.x * sin(theta_) + cmd_input.linear.y *
    cos(theta_)) * 0.2;
    double delta_th = cmd_input.angular.z * 0.2;
    x_ += delta_x;
    y_ += delta_y;
    theta_+= delta_th;
    if(theta_ >= 3.14)
    {
        theta_ = theta_ - 6.28;
    }
    if(theta_ <= -3.14)
    {
        theta_ = theta_ + 6.28;
    }   
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "currentPub_");//初始化串口节点
    ros::NodeHandle n;  //定义节点进程句柄
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::Quaternion current_quat;   
    /***********************		********************************/
    ros::Subscriber sub = n.subscribe("cmd_vel_mux/input/teleop", 200, callback);
    currentPub_ = n.advertise<geometry_msgs::PoseStamped>("/current_pose", 200);
    kedaCurrentPub_ = n.advertise<base_controller::csgPoseStampedMatchValue>("/keda_current_pose", 200);
    double xySigma = 0.001;
    std::default_random_engine generator;
    std::normal_distribution<double>* xyGauss;
    xyGauss = new std::normal_distribution<double>(0.0, xySigma);
    ros::Rate loop_rate(5);//设置数据接收频率
    while(ros::ok())
    {
        struct  timeval  start;
        struct  timeval  end;
        unsigned long timer;
        gettimeofday(&start,NULL);   
        gettimeofday(&end,NULL);
        timer = (1000000 * end.tv_sec + end.tv_usec) - (1000000 * start.tv_sec + start.tv_usec);
        printf("timer = %ld us\n",timer);       
        current_pose.header.stamp = ros::Time::now(); 		
        current_pose.header.frame_id = "odom";			        
        current_pose.pose.position.x = x_ + (*xyGauss)(generator);
        current_pose.pose.position.y = y_ + (*xyGauss)(generator);
        current_pose.pose.position.z = 0.0;        
        current_quat = tf::createQuaternionMsgFromYaw(theta_ + (*xyGauss)(generator));
        current_pose.pose.orientation = current_quat;      
        currentPub_.publish(current_pose);
        base_controller::csgPoseStampedMatchValue cpm;
        cpm.pose = current_pose.pose;
        kedaCurrentPub_.publish(cpm);
        ros::Time stamp_;
//        int d=1e3;
//        ROS_INFO("1e3 %d",d);
//        stamp_.fromNSec(current_pose.header.stamp);
//        ROS_INFO("stamp %d  nsec %d",stamp_.sec,stamp_.nsec);           
        ros::spinOnce();  //程序周期性调用			
        loop_rate.sleep();//周期休眠
    }
    /*ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();*/
    return 0;
}

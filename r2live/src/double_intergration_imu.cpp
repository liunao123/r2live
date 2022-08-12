#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <queue> 


ros::Publisher IMU_pub;
std::queue < sensor_msgs::Imu > imu_queue;

 
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // ROS_INFO("imu: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
    //          msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
    //          msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    imu_queue.push( *msg );
    if (imu_queue.size() >= 2 )
    {
      auto fisrt_imu = imu_queue.front();
      imu_queue.pop();
      auto second_imu = imu_queue.front();
      imu_queue.pop();
      //线加速度
      fisrt_imu.linear_acceleration.x = ( fisrt_imu.linear_acceleration.x + second_imu.linear_acceleration.x) / 2;  
      fisrt_imu.linear_acceleration.y = ( fisrt_imu.linear_acceleration.y + second_imu.linear_acceleration.y) / 2;
      fisrt_imu.linear_acceleration.z = ( fisrt_imu.linear_acceleration.z + second_imu.linear_acceleration.z) / 2;
	    //角速度
      fisrt_imu.angular_velocity.x = ( fisrt_imu.angular_velocity.x + second_imu.angular_velocity.x) / 2;  
      fisrt_imu.angular_velocity.y = ( fisrt_imu.angular_velocity.y + second_imu.angular_velocity.y) / 2;
      fisrt_imu.angular_velocity.z = ( fisrt_imu.angular_velocity.z + second_imu.angular_velocity.z) / 2;
      IMU_pub.publish(fisrt_imu);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "double_intergration_imu");
    ros::NodeHandle node;
    IMU_pub = node.advertise<sensor_msgs::Imu>("/livox/imu", 1000);  
    ros::Subscriber subimu = node.subscribe("/old_imu", 1000, imuCallback);
    ros::spin();
    return 0;
}



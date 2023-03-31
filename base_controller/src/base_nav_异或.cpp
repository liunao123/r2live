/******************************************************************
作用：接收底盘里程计数据和订阅陀螺仪数据，然后集成到odom,然后发布到ros.
*******************************************************************/
#include "../include/base_controller.h"
#include <ros/console.h>
#include "serial.cpp"
#include <thread>
#include <math.h>
#include <ctime>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "zlog.h"
#include <std_msgs/UInt8.h>
#include <base_controller/CSGMagneticInfo.h>
#include <base_controller/CSGMagneticCtrl.h>

#define SLAM_HEAD_1 0xAA
#define SLAM_HEAD_2 0x55
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

//定义的存储上报的里程计数据的结构体
struct Odom
{
    int orx;
    int ory;
    int ori;
    int lin;
    int ang;
};

enum packetFinderState
{
    FRAME_HEAD_1,
    FRAME_HEAD_2,
    FRAME_PAYLOADSIZE,
    FRAME_PAYLOADBODY,
    FRAME_CHECKSUM
};

Motion_Data_Typdef Motion_Data;
ros::Publisher odom_pub_,magneticInfoPub_;
nav_msgs::Odometry odom_;//要发布的里程计对象,odom_pub.publish(odom)

static int od_;
int stop_motion_cmd_ = 0;

typedef struct circ_buff
{
    unsigned char *buff;
    int size;
    volatile int head;
    volatile int tail;
}circ_buff_t;

//对上报的里程计数据的帧进行校验的函数
#define CIRC_CNT(head, tail, size) (((head) - (tail)) & ((size) - 1))
static int __attribute__((unused)) circ_buff_cnt(circ_buff_t *cb)
{
    return CIRC_CNT(cb->head, cb->tail, cb->size);
}

int circ_buff_read(unsigned char *buf, int len, circ_buff_t *cb)
{
    volatile int len2 = len;
    while(len2 > 0)
    {
        /* CIRC_CNT_TO_END */
        int end = cb->size - cb->tail;
        int cnt = (cb->head + end) & (cb->size - 1);
        cnt = cnt < end ? cnt : end;
        if (cnt > len2)
            cnt = len2;
        if (cnt <= 0)
            break;
        memcpy(buf, (const char *)cb->buff + cb->tail, cnt);
        cb->tail = (cb->tail + cnt) & (cb->size - 1);
        buf  += cnt;
        len2 -= cnt;
    }
    return (len - len2);
}

int circ_buff_write(const unsigned char *buf, int len, circ_buff_t *cb)
{
    int len2 = len;
    while (len2 > 0)
    {
        /* CIRC_SPACE_TO_END */
        int end = cb->size - 1 - cb->head;
        int n = (end + cb->tail) & (cb->size - 1);
        int avail = n <= end ? n : end+1;
        if (avail > len2)
            avail = len2;
        if (avail <= 0)
            break;
        memcpy(cb->buff + cb->head, buf, avail);
        cb->head = (cb->head + avail) & (cb->size - 1);
        buf  += avail;
        len2 -= avail;
    }
    return (len - len2);
}

//计算校验和
static uint8_t checkSum(uint8_t *data, int index)
{
    uint8_t cs = 0;
    for(int i = 0; i < index; i++)
    {
        cs ^= data[i];
    }
    return cs;
}

static void crc_buf_init(circ_buff_t *cb, char *p , int len)
{
    cb->buff = (unsigned char *)p;
    cb->size = len;
    cb->head = 0;
    cb->tail = 0;
}

//这个函数是判断收到的是否是一帧数据
static bool isReceiveFrame(uint8_t* frame_data, int max_len,  circ_buff_t *cb)
{
    bool ret = false;
    uint8_t readChar = 0;
    int bytes_read = 0;
    static char state = FRAME_HEAD_1;
    static uint8_t index = 0;

    while(circ_buff_cnt(cb) > 0)
    {
        bytes_read = circ_buff_read((unsigned char *)&readChar, 1, cb);
        if(bytes_read == 0)
        {
            break;
        }
        switch(state)
        {
            case FRAME_HEAD_1:
                if(readChar == SLAM_HEAD_1)
                {
                    state = FRAME_HEAD_2;
                }
                break;
            case FRAME_HEAD_2:
                if(readChar == SLAM_HEAD_2)
                {
                    state = FRAME_PAYLOADSIZE;
                }
                else if(readChar == SLAM_HEAD_1)
                {
                    state = FRAME_HEAD_2;
                }
                else
                {
                    state = FRAME_HEAD_1;
                }
                break;
            case FRAME_PAYLOADSIZE:
                index = 0;
                frame_data[index ++] = readChar;
                if(max_len < readChar + 1)
                {
                    state = FRAME_HEAD_1;
                }
                else
                {
                    state = FRAME_PAYLOADBODY;
                }
                break;
            case FRAME_PAYLOADBODY:
                frame_data[index ++] = readChar;
                if((index - 1)  == frame_data[0])
                {
                    state = FRAME_CHECKSUM;
                }
                if((index - 1)  > frame_data[0])
                {
                    state = FRAME_HEAD_1;
                }
                break;
            case FRAME_CHECKSUM:
                state = FRAME_HEAD_1;
                if((uint8_t)readChar == checkSum(frame_data, index))
                {
                    return true;
                }
                break;
            default:
                state = FRAME_HEAD_1;
                break;
        }
    }
    return ret;
}

//是速度的回调函数，接收到导航下发的速度后，进入此回调函数，与MC定的，会把速度值都乘以1000下发到MC
void callback(const geometry_msgs::Twist & cmd_input)
{
    u8 send_buf[25];
    double send_linear, send_angular;
    if(stop_motion_cmd_ == 1)
    {
      dzlog_info("@@@@@@ callback() need_stop_motion !!!");
      send_linear  = 0.0;
      send_angular = 0.0;
    }
    else
    {
      send_linear  = cmd_input.linear.x;
      send_angular = cmd_input.angular.z;
    }

    dzlog_info("@@@@@@ callback() linear = %f,angular = %f", send_linear,send_angular);

    Motion_Data.Control.Car_Lin_x = send_linear * 1000;
    Motion_Data.Control.Car_Ang_z = send_angular * 1000;	//角速度	单位rad/s
    send_buf[HEAD_ONE] = ROS_FRAME_HEAD_ONE;//这个值为0xAA
    send_buf[HEAD_TWO] = ROS_FRAME_HEAD_TWO;
    send_buf[LENGTH]   = 0x13;
    send_buf[OBJ]      = 0x02;
    send_buf[0x04]     = 0x08;

    send_buf[PARAM1]   = (Motion_Data.Control.Car_Lin_x      ) & 0xff;
    send_buf[PARAM2]   = (Motion_Data.Control.Car_Lin_x >> 8 ) & 0xff;
    send_buf[PARAM3]   = (Motion_Data.Control.Car_Lin_x >> 16) & 0xff;
    send_buf[PARAM4]   = (Motion_Data.Control.Car_Lin_x >> 24) & 0xff;

    send_buf[PARAM5]   = (Motion_Data.Control.Car_Ang_z      ) & 0xff;
    send_buf[PARAM6]   = (Motion_Data.Control.Car_Ang_z >> 8 ) & 0xff;
    send_buf[PARAM7]   = (Motion_Data.Control.Car_Ang_z >> 16) & 0xff;
    send_buf[PARAM8]   = (Motion_Data.Control.Car_Ang_z >> 24) & 0xff;

    send_buf[PARAM9]    = 1;
    send_buf[PARAM10]   = 1;
    send_buf[PARAM11]   = 2;
    send_buf[PARAM12]   = 3;
    send_buf[PARAM13]   = (8      ) & 0xff;
    send_buf[PARAM14]   = (8 >> 8 ) & 0xff;
    send_buf[PARAM15]   = (8 >> 16) & 0xff;
    send_buf[PARAM16]   = (8 >> 24) & 0xff;
    send_buf[PARAM17]   = 2;

    unsigned char cs(0);
    for(unsigned int i = 2; i < PARAM18; i++)
    {
        cs ^= send_buf[i];
    }
    send_buf[PARAM18]  = cs;
    int base_ab = serial_write(od_, send_buf, 23);
    if(base_ab < 0)
    {
        ROS_ERROR("no_vel_serial");
        dzlog_info("@@@@@@ callback() no_vel_serial !!!");
    }
}

void magneticCtrlCallback(const base_controller::CSGMagneticCtrl &cmd_input)
{
    u8 send_buf[25];
    double send_linear, send_angular;
    if(stop_motion_cmd_ == 1)
    {
      dzlog_info("@@@@@@ magneticCtrlCallback() need_stop_motion !!!");
      send_linear  = 0.0;
      send_angular = 0.0;
    }
    else
    {
      send_linear  = cmd_input.fLineV;
      send_angular = cmd_input.fAngleV;
    }

    dzlog_info("@@@@@@ magneticCtrlCallback() linear = %f,angular = %f", send_linear,send_angular);

    Motion_Data.Control.Car_Lin_x = send_linear * 1000;
    Motion_Data.Control.Car_Ang_z = send_angular * 1000;	//角速度	单位rad/s
    send_buf[HEAD_ONE] = ROS_FRAME_HEAD_ONE;//这个值为0xAA
    send_buf[HEAD_TWO] = ROS_FRAME_HEAD_TWO;
    send_buf[LENGTH]   = 0x13;
    send_buf[OBJ]      = 0x02;
    send_buf[0x04]     = 0x08;

    send_buf[PARAM1]   = (Motion_Data.Control.Car_Lin_x      ) & 0xff;
    send_buf[PARAM2]   = (Motion_Data.Control.Car_Lin_x >> 8 ) & 0xff;
    send_buf[PARAM3]   = (Motion_Data.Control.Car_Lin_x >> 16) & 0xff;
    send_buf[PARAM4]   = (Motion_Data.Control.Car_Lin_x >> 24) & 0xff;

    send_buf[PARAM5]   = (Motion_Data.Control.Car_Ang_z      ) & 0xff;
    send_buf[PARAM6]   = (Motion_Data.Control.Car_Ang_z >> 8 ) & 0xff;
    send_buf[PARAM7]   = (Motion_Data.Control.Car_Ang_z >> 16) & 0xff;
    send_buf[PARAM8]   = (Motion_Data.Control.Car_Ang_z >> 24) & 0xff;

    send_buf[PARAM9]    = cmd_input.iNavType;
    send_buf[PARAM10]   = 0;
    send_buf[PARAM11]   = cmd_input.iDirection;
    send_buf[PARAM12]   = 0;
    send_buf[PARAM13]   = (cmd_input.iRFID      ) & 0xff;
    send_buf[PARAM14]   = (cmd_input.iRFID >> 8 ) & 0xff;
    send_buf[PARAM15]   = (cmd_input.iRFID >> 16) & 0xff;
    send_buf[PARAM16]   = (cmd_input.iRFID >> 24) & 0xff;
    send_buf[PARAM17]   = cmd_input.iMLSEMotor;

    unsigned char cs(0);
    for (unsigned int i = 2; i < PARAM18; i++)
    {
        cs ^= send_buf[i];
    }
    send_buf[PARAM18]  = cs;
    int base_ab = serial_write(od_, send_buf, 23);
    if(base_ab < 0)
    {
        ROS_ERROR("no_vel_serial");
        dzlog_info("@@@@@@ magneticCtrlCallback() no_vel_serial !!!");
    }
}

void stopMotionCmdCallback(const std_msgs::UInt8::ConstPtr stop_cmd_msg)
{
    stop_motion_cmd_ = int(stop_cmd_msg->data);
}

void readOdom()
{
  od_ = serial_open("/dev/mcslam_serial", B115200);
  static tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::Quaternion odom_quat;
  float covariance[36] = {0.01,   0,    0,     0,     0,     0,  // covariance on gps_x
                          0,  0.01, 0,     0,     0,     0,  // covariance on gps_y
                          0,  0,    99999, 0,     0,     0,  // covariance on gps_z
                          0,  0,    0,     99999, 0,     0,  // large covariance on rot x
                          0,  0,    0,     0,     99999, 0,  // large covariance on rot y
                          0,  0,    0,     0,     0,     0.01};  // large covariance on rot z
  for(int i = 0; i < 36; i++)
  {
      odom_.pose.covariance[i] = covariance[i];
  }

  Motion_Data.Status.Odom.cur_time = ros::Time::now();	//设置Motion_Data的时间戳
  Motion_Data.Status.Odom.Location.x = 0;
  Motion_Data.Status.Odom.Location.y = 0;
  Motion_Data.Status.Odom.Yaw = 0;
  struct circ_buff rx_buff;
  int bytes_to_process     = 0;
  char buf[512]            = {0};
  uint8_t receive_data[50] = {0};
  uint8_t frame_data[50]   = {0};
  crc_buf_init(&rx_buff, buf, 512);
  static char count = 0;

  while(1)
  {
      usleep(25 * 1000);
      if(od_ < 0)
      {
          ROS_ERROR("no_serial");
          dzlog_info("no_serial");
      }
      else
      {
          bytes_to_process = serial_read(od_, receive_data, 50);
          if(bytes_to_process <= 0)
          {
              ROS_ERROR("no_data");
              dzlog_info("no_data");
          }
          else
          {
              circ_buff_write(receive_data, bytes_to_process, &rx_buff);
              if(circ_buff_cnt(&rx_buff) <= 0)
              {
                  ROS_ERROR("error_buff_write");
                  dzlog_info("error_buff_write");
              }
              else if(circ_buff_cnt(&rx_buff) > 0)
              {
                  bool ret = isReceiveFrame(frame_data, sizeof(frame_data), &rx_buff);
                  if(!ret)
                  {
                      dzlog_info("no_complete_data");
                  }
                  else if(ret)
                  {
                      if(frame_data[1] != 0x01)
                      {
                          ROS_ERROR("id_error");
                          dzlog_info("id_error");
                      }
                      switch(frame_data[1])
                      {
                          case 0x01:
                          Motion_Data.Status.Odom.Location.x = frame_data[2 + 1] + (frame_data[3 + 1] << 8) + (frame_data[4 + 1] << 16) + (frame_data[5 + 1] << 24);
                          Motion_Data.Status.Odom.Location.y = frame_data[6 + 1] + (frame_data[7 + 1] << 8) + (frame_data[8 + 1] << 16) + (frame_data[9 + 1] << 24);
                          Motion_Data.Status.Odom.Yaw = frame_data[10 + 1] + (frame_data[11 + 1] << 8) + (frame_data[12 + 1] << 16) + (frame_data[13 + 1] << 24);
                          Motion_Data.Status.Car_Lin_x = frame_data[14 + 1] + (frame_data[15 + 1] << 8) + (frame_data[16 + 1] << 16) + (frame_data[17 + 1] << 24);
                          Motion_Data.Status.Car_Ang_z = frame_data[18 + 1] + (frame_data[19 + 1] << 8) + (frame_data[20 + 1] << 16) + (frame_data[21 + 1] << 24);
                          uint8_t iNavType = frame_data[22 + 1];//***0:磁导 1:激光***//
                          int iRFID = frame_data[23 + 1] + (frame_data[24 + 1] << 8) + (frame_data[25 + 1] << 16) + (frame_data[26 + 1] << 24);
//                          uint8_t iMLSEF = frame_data[27 + 1];//***前传感器0：没有磁条 1：一条磁条 2：两条磁条 3：三条磁条***//
//                          uint8_t iMLSEB = frame_data[28 + 1];//***后传感器0：没有磁条 1：一条磁条 2：两条磁条 3：三条磁条***//
                          uint8_t iMLSEMotor = frame_data[29 + 1];//***0：磁导未接触到位开关 1：磁导升到顶，2：磁导降到底，3：磁导升到位降到位***//
//                          uint8_t iCMDSN = frame_data[30 + 1];//***保留***//

                          odom_.header.stamp = ros::Time::now(); //载入里程计时间戳
                          odom_.header.frame_id = "odom";
                          odom_.child_frame_id = "base_footprint";  //base_footprint
                          odom_.pose.pose.position.x = float(Motion_Data.Status.Odom.Location.x / 1000.0);
                          odom_.pose.pose.position.y = float(Motion_Data.Status.Odom.Location.y / 1000.0);
                          odom_.pose.pose.position.z = 0.0;
                          odom_.twist.twist.linear.x = (float)(Motion_Data.Status.Car_Lin_x / 1000.0);			//载入线速度
                          odom_.twist.twist.angular.z = (float)(Motion_Data.Status.Car_Ang_z / 1000.0);   		//载入角速度
                          odom_quat = tf::createQuaternionMsgFromYaw(float(Motion_Data.Status.Odom.Yaw / 1000.0));
                          odom_.pose.pose.orientation = odom_quat;
                          odom_pub_.publish(odom_);

                          base_controller::CSGMagneticInfo magInfo;
                          magInfo.iNavType = iNavType;
                          magInfo.iRFID = iRFID;
                          magInfo.iMLSEMotor = iMLSEMotor;
                          magneticInfoPub_.publish(magInfo);

                          if(count++ >= 20)
                          {
                              dzlog_info("odom.pose.pose.position.x: [%f]", (float) odom_.pose.pose.position.x);
                              dzlog_info("odom.pose.pose.position.y: [%f]", (float)odom_.pose.pose.position.y);
                              dzlog_info("Motion_Data.Status.Odom.Yaw: [%f]", (float)(Motion_Data.Status.Odom.Yaw/1000.0));
                              dzlog_info("odom.twist.twist.linear.x: [%f]", (float)odom_.twist.twist.linear.x);
                              dzlog_info("odom.twist.twist.angular.z: [%f]", (float)odom_.twist.twist.angular.z);
                              dzlog_info("magNetic info RFID: [%d]", magInfo.iRFID);
                              dzlog_info("magNetic info MLSEMotor: [%d]", magInfo.iMLSEMotor);
                              count = 0;
                          }

                          odom_trans.transform.translation.x = float(Motion_Data.Status.Odom.Location.x / 1000.0);			//tf位置数据：x,y,z,方向
                          odom_trans.transform.translation.y = float(Motion_Data.Status.Odom.Location.y / 1000.0);
                          odom_trans.transform.translation.z = 0.0;
                          odom_trans.transform.rotation = odom_quat;
                          odom_trans.header.stamp = ros::Time::now();								//载入坐标（tf）变换时间戳
                          odom_trans.header.frame_id = "odom";									   //发布坐标变换的父子坐标系
                          odom_trans.child_frame_id = "base_footprint";
                          odom_broadcaster.sendTransform(odom_trans);
                      }
                  }
              }
          }
      }
  }
}

void sigHandle(int sig)
{
    if(sig==2||sig==15||sig==1||sig==3)
    {
        dzlog_info("tool_catch_signal %d", sig);
        _exit(0);
    }
}

int initZlog()
{
    if (-1 == access("/home/roslog", F_OK))
    {
        mkdir("/home/roslog", 0777);
    }
    if (dzlog_init("/home/config/zlog.conf", "base_cat") != 0)
    {
        printf("loc init zlog failed\n");
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if(initZlog() == 0)
    {
        dzlog_info("base_have_log");
    }
    for(int i=0;i<32;i++)
    {
        signal(i, sigHandle);
    }

    ros::init(argc, argv, "base_nav", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    odom_pub_ = n.advertise<nav_msgs::Odometry>("/odom_raw", 200);
    magneticInfoPub_ = n.advertise<base_controller::CSGMagneticInfo>("/magnetic_info", 200);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel_mux/input/teleop", 200, callback);
    ros::Subscriber magnetic_vel_sub = n.subscribe("cmd_vel_mux/magnetic/ctrl", 200, magneticCtrlCallback);
    ros::Subscriber stop_montion_sub_ = n.subscribe("stop_motion_cmd",200,stopMotionCmdCallback);

    std::thread t(&readOdom);
    t.detach();
    ros::spin();
    return 0;
}

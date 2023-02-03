
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

using namespace std;
using namespace Eigen;

// geometry_msgs::Pose get_PoseFromXYTheta(const double &x, const double &y, const double &theta )
// {
// 	geometry_msgs::Pose p;
// 	p.position.x = x;
// 	p.position.y = y;
// 	// p.position.z = 0;
//     p.orientation = tf::createQuaternionMsgFromYaw(theta);
// 	return p;
// }

double to_rad(const double &ang)
{
  return (ang * M_PI / 180);
}

double to_ang(const double &rad)
{
  return (rad * 180 / M_PI);
}

Eigen::Isometry3d getTransform_Second_2_First(const Eigen::Vector3d First, const Eigen::Vector3d Second)
{
  // 得到欧拉角yaw， 两个frame的 相对旋转
  double theta = (First[2] - Second[2]);
  double ct = cos(theta);
  double st = sin(theta);
  cout << endl
       << "theta " << theta << endl
       << endl;

  Eigen::Vector3d dt;
  // 根据 P = R*p0 + t; 来求 t
  dt[0] = First[0] - (Second[0] * ct - Second[1] * st);
  dt[1] = First[1] - (Second[0] * st + Second[1] * ct);
  dt[2] = 0;

  cout << endl
       << "dt " << dt << endl
       << endl;

  // Isometry3d 是先平移过去（dt ），等二者的原点重合后，再旋转（Rat）
  Eigen::AngleAxisd Rat(theta, Eigen::Vector3d(0, 0, 1));

  Eigen::Isometry3d TF_Second2First = Eigen::Isometry3d::Identity();

  TF_Second2First.rotate(Rat);
  TF_Second2First.pretranslate(dt);
  cout << "TF_Second2First.matrix " << endl
       << TF_Second2First.matrix() << endl
       << endl;
  return TF_Second2First;
}

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr;
}

Eigen::Vector3d getPoseByT(const Eigen::Isometry3d tf_, const Eigen::Vector3d source_pose_)
{
  Eigen::Vector3d target_pose_ = tf_ * source_pose_;

  // 这两个都是旋转矩阵的表示
  // cout << "88" << tf_.rotation() << endl;
  // cout << "89" << tf_.linear().matrix() << endl;
  // theta
  // 转出的角度 偶尔会 有 加减 180 的情况

  // Eigen转出的欧拉角 与 ros tf 不一致，需要注意

  // Eigen::Vector3d euler_angles = tf_.linear().matrix().eulerAngles(2,1,0);	// ZYX顺序，即yaw pitch roll顺序

  Eigen::Vector3d euler_angles = R2ypr(tf_.rotation());

  cout << "yaw pitch roll = " << euler_angles.transpose() << endl
       << endl;
  cout << "source_pose_ = " << source_pose_.transpose() << endl
       << endl;

  //   target_pose_[2] = source_pose_[2] + euler_angles[0] - M_PI; //左手系
  target_pose_[2] = source_pose_[2] + euler_angles[0]; // 右手系

  return target_pose_;
}

int main(int argc, char **argv)
{
    


  Eigen::Matrix3d r_cl = Eigen::Matrix3d::Identity();
  r_cl << 0.0431643, -0.9987869,  0.0236951,
          0.0684185, -0.0207063, -0.9974418,
          0.9967225,  0.0446751,  0.0674417;
  Eigen::Vector3d t_cl;
  t_cl << 0.012,  -0.066, -0.044;

  Eigen::Isometry3d T_cl = Eigen::Isometry3d::Identity();
  T_cl.prerotate(r_cl);
  T_cl.translate(t_cl);

  cout << "T_cl.matrix " << endl
       << T_cl.matrix() << endl
       << endl;

  cout << "T_cl.inverse().matrix " << endl
       << T_cl.inverse().matrix() << endl
       << endl;

    
  Eigen::Matrix3d r_il = Eigen::Matrix3d::Identity();
  r_il << 0.999901 , 0.013091 ,-0.005085,
                   -0.013061 , 0.999897 , 0.005917 , 
                   0.005162 , -0.005850 , 0.999970;
  Eigen::Vector3d t_il;
  t_il << 0.081677, 0.168409, 0.141423;

  Eigen::Isometry3d T_il = Eigen::Isometry3d::Identity();
  T_il.prerotate(r_il);
  T_il.translate(t_il);

  cout << "T_il.matrix " << endl
       << T_il.matrix() << endl
       << endl;

  cout << "T_il.inverse().matrix " << endl
       << T_il.inverse().matrix() << endl
       << endl;

  Eigen::Isometry3d T_ic = T_il * T_cl.inverse();

  cout << "T_ic.matrix " << endl
       << T_ic.matrix() << endl
       << endl;


  return 0;

  // nav_msgs::Odometry odom_start;
  // odom_start.header.stamp = 1;
  // odom_start.header.frame_id = "odom";
  // odom_start.child_frame_id = "base_link";
  // odom_start.pose.pose = get_PoseFromXYTheta(0, 0, 0);

  // nav_msgs::Odometry odom_end;
  // odom_end.header.stamp = 1;
  // odom_end.header.frame_id = "odom";
  // odom_end.child_frame_id = "base_link";
  // odom_end.pose.pose = get_PoseFromXYTheta(2 * 1.414, 0, to_rad(-45)); // 2倍根号2

  // 以下坐标均为 右手系 的值 X, Y ,Theta
  // Eigen::Vector3d odom_start(0, 0, 0);
  // Eigen::Vector3d odom_end(0, -2 * 1.414 , to_rad(-45)); //理论真值

  // Eigen::Vector3d bds_start(1, -1,  to_rad(45));
  // Eigen::Vector3d bds_end(3, -3, 0); //理论真值

  // Eigen::Isometry3d T_odom2bds = getTransform_Second_2_First(bds_start, odom_start);

  // Eigen::Vector3d bds_end_com = getPoseByT(T_odom2bds, odom_end);

  //   cout<<"bds_end_com = "<< bds_end_com.transpose() << endl;

  // 以下坐标均为 右手系 的值 X, Y ,Theta
  Eigen::Vector3d odom_start(-964.557983, -69.304001, to_rad(-172.804));
  Eigen::Vector3d odom_end(-965.775024, -69.496002, to_rad(-170.512));

  Eigen::Vector3d bds_start(-1.056, -0.980, to_rad(-109.06));
  // Eigen::Vector3d bds_end(-1.459, -2.148, to_rad(-107.50) );

  // 求 变换矩阵
  Eigen::Isometry3d T_odom2bds = getTransform_Second_2_First(bds_start, odom_start);

  Eigen::Vector3d bds_end_com = getPoseByT(T_odom2bds, odom_end);

  cout << "bds_end_com = " << bds_end_com.transpose() << endl;
  cout << "bds_end_com = " << to_ang(bds_end_com[2]) << endl;
}

// #include <iostream>
// #include <vector>
// #include <algorithm>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

// using namespace std;
// using namespace Eigen;

// Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
// {
//   Eigen::Vector3d n = R.col(0);
//   Eigen::Vector3d o = R.col(1);
//   Eigen::Vector3d a = R.col(2);

//   Eigen::Vector3d ypr(3);
//   double y = atan2(n(1), n(0));
//   double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
//   double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
//   ypr(0) = y;
//   ypr(1) = p;
//   ypr(2) = r;

//   return ypr; // / M_PI * 180.0;
// }

// /**
//  * @brief Get the Transform Map2UTM ,根据 在两个frame下的同一个位置 <x,y,yaw> <左手系>
//  *
//  * @param vfirst
//  * @param vsecond
//  * @return Eigen::Isometry3d
//  */
// Eigen::Isometry3d getTransformSecond2First(const Eigen::Vector3d vfirst, const Eigen::Vector3d vsecond)
// {
//   // 得到欧拉角yaw， 两个frame的 相对旋转
//   double theta =  ( vfirst[2] - vsecond[2] );
//   double ct = cos(theta);
//   double st = sin(theta);

//   Eigen::Vector3d dt;
//   // 根据 P = R*p0 + t; 来求 t
//   dt[0] = vfirst[0] - (vsecond[0]*ct  - vsecond[1]*st) ;
//   dt[1] = vfirst[1] - (vsecond[0]*st  + vsecond[1]*ct) ;
//   dt[2] = 0;

//   // cout << endl << "dt " << dt <<  endl << endl;

//   // Isometry3d 是先平移过去（dt ），等二者的原点重合后，再旋转（Rat）
//   Eigen::AngleAxisd Rat( theta , Eigen::Vector3d(0, 0, 1));

//   Eigen::Isometry3d TF_map2utm = Eigen::Isometry3d::Identity();
//   cout << endl << "-- " << TF_map2utm.matrix() <<  endl << endl;

//   TF_map2utm.pretranslate(  dt );
//   TF_map2utm.rotate ( Rat );
//   return TF_map2utm;

// }

// /**
//  * @brief Get the Pose In UTM <左手系>
//  *
//  * @param TF_get
//  * @param pose_  <x,y,yaw>
//  * @return Eigen::Vector3d
//  */
// Eigen::Vector3d getPoseInFirst(const Eigen::Isometry3d TF_get, const Eigen::Vector3d pose_)
// {
//   Eigen::Vector3d pose_transed = TF_get * pose_ ;
//   // cout << TF_get.rotate() << endl;
//   // cout << TF_get.linear().matrix() << endl;
//   // theta

// // Eigen 得到的欧拉角没有负值，是有问题的
// // https://zhuanlan.zhihu.com/p/55790406?utm_source=wechat_session
// // https://blog.csdn.net/delovsam/article/details/104432185?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-104432185-blog-104392537.pc_relevant_aa_2&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-104432185-blog-104392537.pc_relevant_aa_2&utm_relevant_index=2

//   //Eigen::Vector3d euler_angles = TF_get.linear().matrix().eulerAngles(2,1,0);	// ZYX顺序，即yaw pitch roll顺序
//   Eigen::Vector3d euler_angles = R2ypr( TF_get.linear().matrix() );

//   cout<<"yaw pitch roll = "<< euler_angles.transpose() << endl << endl;

//   pose_transed[2] = pose_[2] + euler_angles[0];

//   return pose_transed;

// }

// int main(int argc, char** argv)
// {

// //Eigen::Vector3d P_first(-4.946163,  -4.920077,  0.967766);
// //Eigen::Vector3d P_second(9.060000,  4.429000, 0.000000);

// Eigen::Vector3d P_first(8.935928,4.276596,-3.093629);
// Eigen::Vector3d P_second(0000,  000, 0.000000);

// Eigen::Isometry3d TF_second2first = getTransformSecond2First( P_first,  P_second);

// Eigen::Vector3d trans_result =  getPoseInFirst(TF_second2first, P_second);

// cout  << "----------trans result is :" << trans_result.transpose() << endl <<  endl << endl;

// return 0;
// }
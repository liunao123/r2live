#include <odom/LaserLoopClosure_time.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
// #include <velodyne_slam/KeyedScan.h>
// #include <velodyne_slam/PoseGraph.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <thread>
#include <zlog.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <chrono>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

using gtsam::BetweenFactor;
using gtsam::ISAM2;
using gtsam::ISAM2Params;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector3;
using gtsam::Vector6;

LaserLoopClosure::LaserLoopClosure() : key_(0),
                                       last_closure_key_(0),
                                       bIsLoopCheckedM(false),
                                       bIsLoopThreadExitM(false),
                                       bIsGetSaveMapM(false),
                                       iCurHdlKeyM(0),
                                       iLoopKeyM(0),

                                       loop_cnts_(0),
                                       detect_time_regional_(2.5), // 在视觉回环发生的多少时间范围内，进行 点云匹配
                                       detect_step_(1),           // 点云检测的步长，跳着去匹配
                                       have_loop_closure_flag(false),
                                       work_dir_("/home/map/temp_test/")
{
    std::cout << " construct fun : " << std::endl;
}

LaserLoopClosure::~LaserLoopClosure()
{
    std::cout << __FILE__ << "  destructor  fun : " << std::endl;
}

void LaserLoopClosure::saveMap()
{
    cout << "start save map and  optimized path , waiting please ......" << endl;
    std::cout << __FILE__ << ":" << __LINE__ << "  loopClosure Check DONE, SAVE pointcloud map .  " << std::endl;

	saveGtsam2G2oFile(work_dir_ + "loop_gtsam_optimized.g2o");

    PointCloud *points = new PointCloud();
    GetMaximumLikelihoodPoints(points); // points->makeShared()

    std::string name = work_dir_ + "optimized_map_gtsam.pcd";
    pcl::io::savePCDFile(name, *points);
    cout << "map path is :" << name << endl;

    cout << "before filter size is :"  << points->points.size() << endl;
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterTempMap;
    downSizeFilterTempMap.setLeafSize(0.1f , 0.1f, 0.1f);
    downSizeFilterTempMap.setInputCloud((*points).makeShared());
    downSizeFilterTempMap.filter(*points);
    cout << __FILE__ << ":" << __LINE__ << " after filter size is <0.3f >:"  << points->points.size() << endl;
    name = work_dir_ + "optimized_map_gtsam_1dm.pcd";
    pcl::io::savePCDFile(name, *points);

    cout << "save map ok, you can exit ......" << endl;
}

void LaserLoopClosure::setVisionLoopTime(const std::vector<std::pair<double, double>> &loop_time)
{
    loop_time_ = loop_time;
    cout << "Vision loop counts is : " << loop_time_.size() << endl;
    dzlog_info("Vision loop counts is : %d .", loop_time_.size());
}

void LaserLoopClosure::setOneLoopTime(const double first, const double second)
{
    have_loop_closure_flag = true;
    // 加锁 ，防止和后面pop的动作冲突
    if (loop_time_queue.empty())
    {
        g_Mutex.lock();
        loop_time_queue.push(std::make_pair(first, second));
        g_Mutex.unlock();
    }
    else
    {
        if (std::fabs(loop_time_queue.back().first - first) > 2.0 || std::fabs(loop_time_queue.back().second - second) > 2.0)
        {
            g_Mutex.lock();
            loop_time_queue.push(std::make_pair(first, second));
            g_Mutex.unlock();
        }
    }

    cout << "get a vision loop time pair. loop_time_queue size is: "  << loop_time_queue.size() << endl;
    dzlog_info("get a vision loop time pair. loop_time_queue size is: %d" , loop_time_queue.size() );

}

void LaserLoopClosure::setWorkPath(const std::string work_dir)
{
    work_dir_ = work_dir;
    cout << " work_dir_ is : " << work_dir_ << endl;
}

void LaserLoopClosure::setMapThreadDone()
{
    bIsLoopThreadExitM= true;
}


void LaserLoopClosure::setTranslationThreshold(const double translation_threshold)
{
    translation_threshold_ = translation_threshold;
    // cout << " Reset translation_threshold value,  now is : " << translation_threshold_ << endl;
}

bool LaserLoopClosure::Initialize()
{
    //***create loopclosure thread***//
    std::thread t1(&LaserLoopClosure::loopClosureThread, this);
    t1.detach();

    std::cout << " LoadParameters s  : " << std::endl;
    LoadParameters();
    std::cout << " LoadParameters e  : " << std::endl;
    return true;
}

void LaserLoopClosure::filter_pointcloud(PointCloud &pc)
{
    // downsample clouds
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.setInputCloud(pc.makeShared());
    vg.filter(pc);
}

bool LaserLoopClosure::LoadParameters()
{
    /* */
    // Load frame ids.
    // if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;
    // if (!pu::Get("frame_id/base", base_frame_id_)) return false;

    // Should we turn loop closure checking on or off?
    check_for_loop_closures_ = true;
    // if (!pu::Get("loop_closure/check_for_loop_closures", check_for_loop_closures_)) return false;

    // Load ISAM2 parameters.
    unsigned int relinearize_skip = 1; // 1
    double relinearize_threshold = 0.01;
    // if (!pu::Get("loop_closure/relinearize_skip", relinearize_skip)) return false;
    // if (!pu::Get("loop_closure/relinearize_threshold", relinearize_threshold)) return false;

    // Load loop closing parameters.
    translation_threshold_ = 0.5;  // 0.75
    proximity_threshold_ = 15;     // 10
    max_tolerable_fitness_ = 0.36;  // 0.36; 不要太高< less 0.5 >，否则错误的约束加到GTSAM里面后，无法优化出结果
    skip_recent_poses_ = 20;       // 20
    maxLoopKeysYawM = 1.0; //1.05
    // if (!pu::Get("loop_closure/translation_threshold", translation_threshold_)) return false;
    // if (!pu::Get("loop_closure/proximity_threshold", proximity_threshold_)) return false;
    // if (!pu::Get("loop_closure/max_tolerable_fitness", max_tolerable_fitness_)) return false;
    // if (!pu::Get("loop_closure/skip_recent_poses", skip_recent_poses_)) return false;
    // if (!pu::Get("loop_closure/max_loopkeys_yaw", maxLoopKeysYawM)) return false;
    // dzlog_info("@@@@@@ LoadParameters() maxLoopKeysYawM = %f",maxLoopKeysYawM);

    // Load ICP parameters.
    icp_tf_epsilon_ = 0.0000000001;
    icp_corr_dist_ = 0.25;
    icp_iterations_ = 30;
    // if (!pu::Get("icp/tf_epsilon", icp_tf_epsilon_)) return false;
    // if (!pu::Get("icp/corr_dist", icp_corr_dist_)) return false;
    // if (!pu::Get("icp/iterations", icp_iterations_)) return false;

    // Load initial position and orientation.

    double init_x = 0.01, init_y = 0.01, init_z = 0.01;
    double init_roll = 0.001, init_pitch = 0.001, init_yaw = 0.001;

    // double init_x = 0.001, init_y = 0.001, init_z = 0.001;
    // double init_roll = 0.00001, init_pitch = 0.00001, init_yaw = 0.00001;

    // if (!pu::Get("init/position/x", init_x)) return false;
    // if (!pu::Get("init/position/y", init_y)) return false;
    // if (!pu::Get("init/position/z", init_z)) return false;
    // if (!pu::Get("init/orientation/roll", init_roll)) return false;
    // if (!pu::Get("init/orientation/pitch", init_pitch)) return false;
    // if (!pu::Get("init/orientation/yaw", init_yaw)) return false;

    // Load initial position and orientation noise.

    double sigma_x = 0.01, sigma_y = 0.01, sigma_z = 0.01;
    double sigma_roll = 0.001, sigma_pitch = 0.001, sigma_yaw = 0.001;
    // double sigma_x = 0.001, sigma_y = 0.001, sigma_z = 0.001;
    // double sigma_roll = 0.00001, sigma_pitch = 0.00001, sigma_yaw = 0.00001;

    // if (!pu::Get("init/position_sigma/x", sigma_x)) return false;
    // if (!pu::Get("init/position_sigma/y", sigma_y)) return false;
    // if (!pu::Get("init/position_sigma/z", sigma_z)) return false;
    // if (!pu::Get("init/orientation_sigma/roll", sigma_roll)) return false;
    // if (!pu::Get("init/orientation_sigma/pitch", sigma_pitch)) return false;
    // if (!pu::Get("init/orientation_sigma/yaw", sigma_yaw)) return false;

    // Create the ISAM2 solver.
    ISAM2Params parameters;

    parameters.factorization = ISAM2Params::QR; // add by ln, to solve :terminate called after throwing an instance of 'gtsam::IndeterminantLinearSystemException'

    parameters.relinearizeSkip = relinearize_skip;
    parameters.relinearizeThreshold = relinearize_threshold;
    isam_.reset(new ISAM2(parameters));

    // Set the initial position.
    Vector3 translation(init_x, init_y, init_z);
    Rot3 rotation(Rot3::RzRyRx(init_roll, init_pitch, init_yaw));
    Pose3 pose(rotation, translation);

    // Set the covariance on initial position.
    Vector6 noise;
    noise << sigma_x, sigma_y, sigma_z, sigma_roll, sigma_pitch, sigma_yaw;
    LaserLoopClosure::Diagonal::shared_ptr covariance(
        LaserLoopClosure::Diagonal::Sigmas(noise));

    // Initialize ISAM2.
    NonlinearFactorGraph new_factor;
    Values new_value;
    new_factor.add(MakePriorFactor(pose, covariance));
    new_value.insert(key_, pose);

    isam_->update(new_factor, new_value);
    values_ = isam_->calculateEstimate();

    key_++;

    // Set the initial odometry.
    odometry_ = Pose3::identity();

    std::cout << __FILE__ << ":" << __LINE__ << " : LoadParameters : " << std::endl;

    return true;
}

void LaserLoopClosure::GetMaximumLikelihoodPoints(PointCloud *points)
{
    if (points == NULL)
    {
        dzlog_info("%s: Output point cloud container is null.", name_.c_str());
        return;
    }
    points->points.clear();
    
    cout << work_dir_ << endl;
    cout << "start save gtsam pose, graph.g2o, sth for interactive_slam. wait ...... " << endl;

    // 会覆盖已经有的文件
    ofstream GTSAM_pose(work_dir_ + "GTSAM_pose.txt", ios::out);

    // 先清空文件夹
    system( ("rm -r " + work_dir_ + "pose_graph/" ).c_str() );
    
    string SC_path = work_dir_  + "SC/" ;
    std::string mk_sc_f =  "mkdir -p " + SC_path;
    system(mk_sc_f.c_str());

    // Iterate over poses in the graph, transforming their corresponding laser
    // scans into world frame and appending them to the output.
    for (const auto &keyed_pose : values_)
    {
        const unsigned int key = keyed_pose.key;

        // int temp = int();
        if (key % 500 == 0)
        {
            // cout << "haved save " << temp << "% . " << endl;
            cout  << int ((1.0 * key) / (1.0 * key_) * 100) << "%. " ;
        }
        // Check if this pose is a keyframe. If it's not, it won't have a scan
        // associated to it and we should continue.
        if (!keyed_scans_.count(key))
            continue;

        const gu::Transform3 pose = ToGu(values_.at<Pose3>(key));
        Eigen::Matrix4d b2w;
        b2w.block(0, 0, 3, 3) = pose.rotation.Eigen();
        b2w.block(0, 3, 3, 1) = pose.translation.Eigen();

        // Transform the body-frame scan into world frame.
        PointCloud scan_world;
        pcl::transformPointCloud(*keyed_scans_[key], scan_world, b2w);

        // Append the world-frame point cloud to the output.
        *points += scan_world;

        // save sth for interactive slam
        std::stringstream ss;
        ss << setw(6) << setfill('0') << key;
        // cout << ss.str() << endl;

        string one_path = work_dir_ + "pose_graph/" + ss.str();

        // std::string rmf =  "rm -r " + one_path;
        // system(rmf.c_str());

        std::string mkf =  "mkdir -p " + one_path;
        system(mkf.c_str());

        // save pcd
        PointCloud::Ptr scan(new PointCloud);
        getKeyedScan(key, scan);
        pcl::io::savePCDFile( one_path + "/cloud.pcd", *scan);
        
        // save data file
        ofstream pose_data( one_path + "/data", ios::out);
        pose_data
            << "stamp " << scan->header.stamp << " 0" << endl
            <<  "estimate" << endl << fixed << setprecision(10)
            << pose.rotation(0) << " " << pose.rotation(1) << " " << pose.rotation(2) << " " << pose.translation(0) << endl
            << pose.rotation(3) << " " << pose.rotation(4) << " " << pose.rotation(5) << " " << pose.translation(1) << endl
            << pose.rotation(6) << " " << pose.rotation(7) << " " << pose.rotation(8) << " " << pose.translation(2) << endl
            << "0 0 0 1" << endl
            << "odom" << endl
            << pose.rotation(0) << " " << pose.rotation(1) << " " << pose.rotation(2) << " " << pose.translation(0) << endl
            << pose.rotation(3) << " " << pose.rotation(4) << " " << pose.rotation(5) << " " << pose.translation(1) << endl
            << pose.rotation(6) << " " << pose.rotation(7) << " " << pose.rotation(8) << " " << pose.translation(2) << endl
            << "0 0 0 1" << endl
            << "id " << key << endl;

        pose_data.close();

        //  result for sc
        // pcl::io::savePCDFile( SC_path + std::to_string(key) + ".pcd", *scan);

        // system( ( "cp " + one_path + "/data " + SC_path + std::to_string(key) + ".odom"  ).c_str() );
        
        // 跳过第一个，因为第一个是0
        if (key < 2)
            continue;

        // 往上面的文件里写
        GTSAM_pose.setf(ios::fixed, ios::floatfield);
        GTSAM_pose.precision(10);
        //  GTSAM_pose << key  << " ";
        GTSAM_pose << keyed_scans_[key]->header.stamp << " ";
        GTSAM_pose.precision(5);

        Eigen::Vector3d correct_t(pose.translation.Eigen());
        Eigen::Quaterniond correct_q(pose.rotation.Eigen());

        GTSAM_pose
            << correct_t.x() << " "
            << correct_t.y() << " "
            << correct_t.z() << " "
            << correct_q.x() << " "
            << correct_q.y() << " "
            << correct_q.z() << " "
            << correct_q.w() << endl;
    }
    saveGtsam2G2oFile(work_dir_ + "pose_graph/graph.g2o");
    // 把上面的文件 关闭
    GTSAM_pose.close();

}

gu::Transform3 LaserLoopClosure::GetLastPose() const
{
    unsigned int lastKey = (key_ > 1) ? (key_ - 1) : 0;
    Pose3 lastKeyPose = values_.at<Pose3>(lastKey);
    Pose3 curPose = lastKeyPose.compose(odometry_);
    dzlog_info("@@@@@@ GetLastPose() curkey = %d,lastkey = %d", key_, lastKey);
    dzlog_info("@@@@@@ GetLastPose() increodom dist = %f,yaw = %f", odometry_.translation().norm(), odometry_.rotation().yaw());
    dzlog_info("@@@@@@ GetLastPose() lastkeypose x = %f,y = %f", lastKeyPose.translation().x(), lastKeyPose.translation().y());
    dzlog_info("@@@@@@ GetLastPose() curpose x = %f,y = %f", curPose.translation().x(), curPose.translation().y());
    return ToGu(curPose);
}

gu::Transform3 LaserLoopClosure::ToGu(const Pose3 &pose) const
{
    gu::Transform3 out;
    out.translation(0) = pose.translation().x();
    out.translation(1) = pose.translation().y();
    out.translation(2) = pose.translation().z();

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
            out.rotation(i, j) = pose.rotation().matrix()(i, j);
    }

    return out;
}

Pose3 LaserLoopClosure::ToGtsam(const gu::Transform3 &pose) const
{
    Vector3 t;
    t(0) = pose.translation(0);
    t(1) = pose.translation(1);
    t(2) = pose.translation(2);

    Rot3 r(pose.rotation(0, 0), pose.rotation(0, 1), pose.rotation(0, 2),
           pose.rotation(1, 0), pose.rotation(1, 1), pose.rotation(1, 2),
           pose.rotation(2, 0), pose.rotation(2, 1), pose.rotation(2, 2));

    return Pose3(r, t);
}

LaserLoopClosure::Mat66 LaserLoopClosure::ToGu(
    const LaserLoopClosure::Gaussian::shared_ptr &covariance) const
{
    gtsam::Matrix66 gtsam_covariance = covariance->covariance();

    LaserLoopClosure::Mat66 out;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            out(i, j) = gtsam_covariance(i, j);

    return out;
}

LaserLoopClosure::Gaussian::shared_ptr LaserLoopClosure::ToGtsam(
    const LaserLoopClosure::Mat66 &covariance) const
{
    gtsam::Matrix66 gtsam_covariance;

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            gtsam_covariance(i, j) = covariance(i, j);

    return Gaussian::Covariance(gtsam_covariance);
}

PriorFactor<Pose3> LaserLoopClosure::MakePriorFactor(
    const Pose3 &pose,
    const LaserLoopClosure::Diagonal::shared_ptr &covariance)
{
    return PriorFactor<Pose3>(key_, pose, covariance);
}

BetweenFactor<Pose3> LaserLoopClosure::MakeBetweenFactor(
    const Pose3 &delta,
    const LaserLoopClosure::Gaussian::shared_ptr &covariance)
{
    odometry_edges_.push_back(std::make_pair(key_ - 1, key_));
    return BetweenFactor<Pose3>(key_ - 1, key_, delta, covariance);
}

void LaserLoopClosure::Serialize2File(string posePath, string normal_HistogramPath, string height_HistogramPath, float x_offset, float y_offset)
{
    std::ofstream pose_file; //输出位姿到文件
    pose_file.open(posePath.c_str(), std::ofstream::out | std::ofstream::app);
    if (!pose_file)
    {
        std::cout << "file not open(pose)!!";
    }

    std::ofstream height_histo_file; //输出直方图表示到文件
    height_histo_file.open(height_HistogramPath.c_str(), std::ofstream::out | std::ofstream::app);
    if (!height_histo_file)
    {
        std::cout << "file not open(histogram)!!";
    }
    std::ofstream normal_histo_file; //输出直方图表示到文件
    normal_histo_file.open(normal_HistogramPath.c_str(), std::ofstream::out | std::ofstream::app);
    if (!normal_histo_file)
    {
        std::cout << "file not open(normal_histogram)!!";
    }

    std::map<unsigned int, vector<float>>::iterator it = keyed_histogram_.begin();
    std::map<unsigned int, vector<float>>::iterator it2 = keyed_histogram_normal_.begin();
    for (; it != keyed_histogram_.end() && it2 != keyed_histogram_normal_.end(); ++it, ++it2)
    {
        unsigned int cur_key = it->first;
        for (int i = 0; i < it->second.size(); i++)
        {
            height_histo_file << it->second[i] << " ";
        }
        height_histo_file << std::endl;

        for (int i = 0; i < it2->second.size(); i++)
        {
            normal_histo_file << it2->second[i] << " ";
        }
        normal_histo_file << std::endl;

        Pose3 p = values_.at<Pose3>(cur_key);
        float x = p.translation().x() - x_offset;
        float y = p.translation().y() - y_offset;
        float yaw = p.rotation().yaw();
        pose_file << x << " " << y << " " << yaw << std::endl;
    }
}

bool LaserLoopClosure::addPoseAndKeyScan(const geometry_utils::Transform3 &delta, const PointCloud::ConstPtr &scan)
{
    unsigned int pose_key;
    gu::MatrixNxNBase<double, 6> covariance;
    covariance.Zeros();
    for (int i = 0; i < 3; ++i)
        covariance(i, i) = 0.01;
    for (int i = 3; i < 6; ++i)
        covariance(i, i) = 0.004;

    isamMutexM.lock();

    // PublishPoseGraph();
    //当位姿平移大于阈值则记录一次关键帧，返回true，否则返回false//
    const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);

    if (!AddBetweenFactor(delta, covariance, stamp, &pose_key))
    {
        isamMutexM.unlock();
        return false;
    }
    isamMutexM.unlock();

    if (!AddKeyScanPair(pose_key, scan))
    {
        return false;
    }
    return true;
}

unsigned int LaserLoopClosure::getLastScanKey()
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    if (!keyed_scans_.empty())
    {
        return (unsigned int)keyed_scans_.rbegin()->first;
    }
    return 0;
}

bool LaserLoopClosure::isKeyFrame(unsigned int iKey)
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    if (!keyed_scans_.count(iKey))
    {
        return false;
    }
    return true;
}

void LaserLoopClosure::getKeyedScan(unsigned int iKey, PointCloud::Ptr scan)
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    const PointCloud::ConstPtr tmpScan = keyed_scans_[iKey];
    pcl::copyPointCloud(*tmpScan, *scan);
}

const geometry_utils::Transform3 LaserLoopClosure::getDiffPose(unsigned int iKey1, unsigned int iKey2)
{
    isamMutexM.lock();
    const gu::Transform3 pose1 = ToGu(values_.at<Pose3>(iKey1));
    const gu::Transform3 pose2 = ToGu(values_.at<Pose3>(iKey2));
    isamMutexM.unlock();
    // printf("pose1: x=%f ,y=%f ,z=%f .\n", pose1.translation.X(), pose1.translation.Y(), pose1.translation.Z());
    // printf("pose2: x=%f ,y=%f ,z=%f .\n", pose2.translation.X(), pose2.translation.Y(), pose2.translation.Z());

    const gu::Transform3 diffPose = gu::PoseDelta(pose1, pose2);
    // printf("d: dx=%f ,dy=%f ,dz=%f .\n", diffPose.translation.X(), diffPose.translation.Y(), diffPose.translation.Z());

    return diffPose;
}

void LaserLoopClosure::updatePoseFactor()
{
    // 查看论文，寻找ICP求协方差矩阵的算法
    LaserLoopClosure::Mat66 covariance;
    covariance.Zeros();
    for (int i = 0; i < 2; ++i)
        covariance(i, i) = 0.01;
    for (int i = 2; i < 5; ++i)
        covariance(i, i) = 0.001;
    for (int i = 5; i < 6; ++i)
        covariance(i, i) = 0.04;

    g_Mutex.lock();
    NonlinearFactorGraph new_factor;
    new_factor.add(BetweenFactor<Pose3>(iCurHdlKeyM, iLoopKeyM, ToGtsam(loopDeltaM), ToGtsam(covariance)));
    isam_->update(new_factor, Values());
    values_ = isam_->calculateEstimate(); //原来是有无回环都调用一次，但是感觉无回环没必要调用
    last_closure_key_ = iCurHdlKeyM;
    g_Mutex.unlock();

    loop_edges_.push_back(std::make_pair(iCurHdlKeyM, iLoopKeyM));
    // 标示我们发现了一个闭环
    loop_closure_notifier_pub_.publish(std_msgs::Empty());
    dzlog_info("@@@@@@ updatePoseFactor() update loop pose between key %u and %u", iCurHdlKeyM, iLoopKeyM);
}

void LaserLoopClosure::setIsLoopChecked(bool bStatus)
{
    bIsLoopCheckedM = bStatus;
}

bool LaserLoopClosure::getIsLoopCheched()
{
    return bIsLoopCheckedM;
}

void LaserLoopClosure::setIsGetSaveMapFlag(bool bStatus)
{
    bIsGetSaveMapM = bStatus;
}

bool LaserLoopClosure::getIsLoopThreadExit()
{
    return bIsLoopThreadExitM;
}

void LaserLoopClosure::saveGtsam2G2oFile(string outputFile)
{
    g_Mutex.lock();
    gtsam::Values value = isam_->calculateEstimate();
    NonlinearFactorGraph factorGraph = isam_->getFactorsUnsafe();
    gtsam::writeG2o(factorGraph, value, outputFile);
    g_Mutex.unlock();
}

bool LaserLoopClosure::PerformICP(const PointCloud::ConstPtr &scan1,
                                  const PointCloud::ConstPtr &scan2,
                                  const gu::Transform3 &pose1,
                                  const gu::Transform3 &pose2,
                                  gu::Transform3 *delta,
                                  LaserLoopClosure::Mat66 *covariance,
                                  double *fitnessReturn)
{
    // dzlog_info(" start GICP . ");
    if (delta == NULL || covariance == NULL)
    {
        dzlog_info("%s: Output pointers are null.", name_.c_str());
        return false;
    }

    // ICP：此处考虑了闭合时，机器人可能不是一个朝向，此时直接ICP可能匹配不上，这时候
    //利用pose1/2时的朝向的估计信息。转换后再匹配。
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setTransformationEpsilon(icp_tf_epsilon_);
    icp.setMaxCorrespondenceDistance(icp_corr_dist_);
    icp.setMaximumIterations(icp_iterations_);
    icp.setRANSACIterations(50);

    // PointCloud LaserLoopClosure::filter_pointcloud(const PointCloud::ConstPtr pc)

    // PointCloud scan1_filter = *scan1;
    // PointCloud scan2_filter = *scan2;
    // filter_pointcloud(scan1_filter);
    // filter_pointcloud(scan2_filter);
    // icp.setInputSource(scan1_filter.makeShared()); // source
    // icp.setInputTarget(scan2_filter.makeShared());

    icp.setInputSource(scan1);  //source
    icp.setInputTarget(scan2);

    // Perform ICP.
    PointCloud unused_result;
    icp.align(unused_result); //将2个关键帧配准

    // Get resulting transform.
    const Eigen::Matrix4f T = icp.getFinalTransformation();
    gu::Transform3 delta_icp;
    delta_icp.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
    delta_icp.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                  T(1, 0), T(1, 1), T(1, 2),
                                  T(2, 0), T(2, 1), T(2, 2));

    *fitnessReturn = icp.getFitnessScore();
    if (!icp.hasConverged())
    {
        dzlog_info("icp isn't coverged");
        return false;
    }

    if (icp.getFitnessScore() > max_tolerable_fitness_)
    {
        dzlog_info("score %f ", icp.getFitnessScore());
        return false;
    }

    // 求出的 变换矩阵 做不做平面约束
    // delta_icp.translation =  gu::Vec3(delta_icp.translation.X(), delta_icp.translation.Y(), 0);
    // delta_icp.rotation = gu::Rot3(0,0,delta_icp.rotation.Yaw());

    //反转坐标系，输出才是对的！
    *delta = gu::PoseInverse(delta_icp);
    // 查看论文，寻找ICP求协方差矩阵的算法
    covariance->Zeros();
    for (int i = 0; i < 2; ++i)
        (*covariance)(i, i) = 0.01;
        // (*covariance)(i, i) = *fitnessReturn / 10;

        
    for (int i = 2; i < 5; ++i)
        (*covariance)(i, i) = 0.001;
        // (*covariance)(i, i) = *fitnessReturn / 100;

    for (int i = 5; i < 6; ++i)
        (*covariance)(i, i) = 0.04;
        // (*covariance)(i, i) = *fitnessReturn * 4 / 10;


    // icp的结果
    static int cnts = 1;
    printf("%dth score %f \n", cnts, icp.getFitnessScore());

    cout << "icp rotation (deg) :" << endl
         << delta->rotation.GetEulerZYX() * 180 / M_PI << endl;

    std::string name;
    name = "/root/Desktop/r2/src/icp/_" + std::to_string(cnts) + "_scan2.pcd";
    // cout << "754 path is :" << name << endl;
    pcl::io::savePCDFile(name, *scan2);
    // pcl::io::savePCDFile(name, scan2_filter);

    name = "/root/Desktop/r2/src/icp/_" + std::to_string(cnts) + "_scan1.pcd";
    // cout << "759 path is :" << name << endl;
    pcl::io::savePCDFile(name, *scan1);
    // pcl::io::savePCDFile(name, scan1_filter);

    name = "/root/Desktop/r2/src/icp/_" + std::to_string(cnts) + "_unused_result.pcd";
    cout << "764 path is :" << name << endl;
    pcl::io::savePCDFile(name, unused_result);

    dzlog_info("%dth score %f , file is %s ", cnts, icp.getFitnessScore(), name.c_str());
    cnts++;
    // dzlog_info(" end GICP . ");

    return true;
}

bool LaserLoopClosure::FindLoopClosures(unsigned int key_temp, const double &target_time)
{
    dzlog_info("try to FindLoopClosures. ");
    int key = key_temp;

    // If loop closure checking is off, don't do this step. This will save some
    // computation time.
    // if (!check_for_loop_closures_)
    //     return false;

    // If a loop has already been closed recently, don't try to close a new one.
    if (std::fabs(key - last_closure_key_) < skip_recent_poses_)
        return false;

    // Get pose and scan for the provided key.
    const gu::Transform3 pose1 = ToGu(values_.at<Pose3>(key));
    const PointCloud::ConstPtr scan1 = keyed_scans_[key];

    // 遍历所有之前位姿，找到与当前位姿最匹配的一个（闭环）
    bool closed_loop = false;
    double fitnessMin = max_tolerable_fitness_;
    gu::Transform3 delta;
    LaserLoopClosure::Mat66 covariance;
    int keyResut = 0;

    int other_key;

    for (const auto &keyed_pose : values_)
    {
        other_key = keyed_pose.key;
        
        // Don't self-check.
        // Don't compare against poses that were recently collected.
        if (other_key > key - skip_recent_poses_)
        {
            break; // 只会与其之前的做匹配
        }

        // 是不是关键帧
        if (!isKeyFrame(other_key))
        {
            continue;
        }

        const gu::Transform3 pose2 = ToGu(values_.at<Pose3>(other_key));
        const gu::Transform3 difference = gu::PoseDelta(pose1, pose2);

        // const PointCloud::ConstPtr scan2 = keyed_scans_[other_key];
        // const PointCloud::ConstPtr scan2;
        PointCloud::Ptr scan2(new PointCloud);
        try
		{
            getKeyedScan(other_key, scan2);
            // scan2 = keyed_scans_[other_key];
		}
		catch (const std::exception &e)
		{
			std::cerr << e.what() << '\n';
			break;
		}

        // 时间戳是不是在视觉回环的附近
        // 2 就可以，在远的匹配得分就很高了
        if (std::fabs(scan2->header.stamp - target_time) > detect_time_regional_ )
        {
            continue;
        }

        dzlog_info("try DO GICP , with other_key is %u , key is %u ", other_key, key);
        // 从头到尾 依次检测，有可能出现：
        // 中间的回环将尾的位置 纠正到 离头的 位置很远，这样 位置差就不满足以下的关系了。
        // 由于已经从视觉回环上确定了大概的时间，此时对应的位置也基本确定，将其注释掉，问题不大。
        // 在测试 20221011
        if ((difference.translation.Norm() < proximity_threshold_) &&
            (fabs(difference.rotation.Yaw()) < maxLoopKeysYawM))
        {
            gu::Transform3 deltaTemp;
            LaserLoopClosure::Mat66 covarianceTemp;
            double fitnessReturn;
        
            if (PerformICP(scan1, scan2, pose1, pose2, &deltaTemp, &covarianceTemp, &fitnessReturn))
            {
                static int icp_ok_cnts = 1;
                // 找到一个闭环位姿
                if (fitnessReturn < fitnessMin)
                {
                    fitnessMin = fitnessReturn;
                    keyResut = other_key;
                    delta = deltaTemp;
                    covariance = covarianceTemp;
                }
                dzlog_info(" %uth icp ok, loop closure between other_key: %u and key: %u , fitnessMin %f diff_dist %f , diff_yaw %f , DIFF key = %u .  ", icp_ok_cnts++, other_key, key, fitnessMin, difference.translation.Norm(), difference.rotation.Yaw() , key - other_key);
                closed_loop = true;
            }
        }
    }

    if (closed_loop)
    {
        std::cout << __FILE__ << ":" << __LINE__ << "  choose one min score ...... " << std::endl;

        dzlog_info(" delta is: translation: %f, %f, %f .", delta.translation(0), delta.translation(1), delta.translation(2));
        auto rota_temp = delta.rotation.GetEulerZYX() * 180 / M_PI;
        dzlog_info(" delta is: rotation: %f, %f, %f (deg).", rota_temp(0), rota_temp(1), rota_temp(2));

        g_Mutex.lock();
        NonlinearFactorGraph new_factor;
        new_factor.add(BetweenFactor<Pose3>(key, keyResut, ToGtsam(delta), ToGtsam(covariance)));
        isam_->update(new_factor, Values());
        values_ = isam_->calculateEstimate();

        g_Mutex.unlock();

        
        loop_cnts_++;
        // saveGtsam2G2oFile(work_dir_ + std::to_string(loop_cnts_) +"_loop_gtsam_optimized.g2o");

        last_closure_key_ = key;
        dzlog_info("\n\n @@@@@@ %uth isHasLoopClosure() find a loop closure between key %u and %u fitnessMin=%f, DIFF key = %u . last_closure_key_ = %u \n ", loop_cnts_, key, keyResut, fitnessMin, key - keyResut, last_closure_key_);
        printf(" %uth loop closure between key %u and %u , fitnessMin %f , DIFF key = %u .  \n\n ", loop_cnts_, key, keyResut, fitnessMin, key - keyResut);
        std::cout << delta << std::endl;

    }
    return closed_loop;
}

bool LaserLoopClosure::AddBetweenFactor(const gu::Transform3 &delta,
                                        const LaserLoopClosure::Mat66 &covariance,
                                        const ros::Time &stamp, unsigned int *key)
{
    if (key == NULL)
    {
        dzlog_info("%s: Output key is null.", name_.c_str());
        return false;
    }

    // Append the new odometry.
    Pose3 new_odometry = ToGtsam(delta);

    // 轨迹累加
    odometry_ = odometry_.compose(new_odometry);

    // 两帧很近
    if ((odometry_.translation().norm() < translation_threshold_) &&
        (fabs(odometry_.rotation().yaw()) < 0.3))
    {
        return false;
    }

    g_Mutex.lock();
    
    NonlinearFactorGraph new_factor;
    Values new_value;
    new_factor.add(MakeBetweenFactor(odometry_ /*new_odometry*/, ToGtsam(covariance)));

    Pose3 last_pose = values_.at<Pose3>(key_ - 1);
    new_value.insert(key_, last_pose.compose(odometry_ /*new_odometry*/));

    // Store this timestamp so that we can publish the pose graph later.
    keyed_stamps_.insert(std::pair<unsigned int, ros::Time>(key_, stamp));

    // Update ISAM2.
    isam_->update(new_factor, new_value);
    values_ = isam_->calculateEstimate();

    g_Mutex.unlock();

    // Assign output and get ready to go again!
    *key = key_++;

    odometry_ = Pose3::identity();
    return true;
}

bool LaserLoopClosure::AddKeyScanPair(unsigned int key, const PointCloud::ConstPtr &scan)
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    if (keyed_scans_.count(key))
    {
        dzlog_info("%s: Key %u already has a laser scan.", name_.c_str(), key);
        //TODO 把不是关键帧的点云也加进去，依次增加点云密度 20221008
        return false;
    }

    // The first key should be treated differently; we need to use the laser
    // scan's timestamp for pose zero.
    if (key == 0)
    {
        const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
        keyed_stamps_.insert(std::pair<unsigned int, ros::Time>(key, stamp));
    }

    // Add the key and scan.
    keyed_scans_.insert(std::pair<unsigned int, PointCloud::ConstPtr>(key, scan));

    return true;
}

void LaserLoopClosure::loopClosureThread()
{
    dzlog_info("@@@@@@ loopClosureThread() loop thread IN !!!");

    // 循环判断 视觉 有无回环进来
    while ( 1 )
    {
        // TODO 如何退出这个循环
        // 如何判断 建图程序已经结束，视觉不再有回环了

        // 有视觉的回环还有就进行 GICP 
        if ( loop_time_queue.empty() )
        {
            dzlog_info("Do not get vision loopclosure .loopClosureThread sleep 20s . key frame %d ......", getLastScanKey() );
            // printf("Do not vision loopclosure .loopClosureThread sleep 20s . key frame %d ......\n", getLastScanKey()  );             
            usleep(20000 * 1000);
            if ( bIsLoopThreadExitM )
            {
                break;
            }
            continue;
        }
        // ROS_WARN("start loop gtsam .");

        // 把最前面的回环时间拿出来
        double loop_time_first = loop_time_queue.front().first;
        double loop_time_second = loop_time_queue.front().second;
        // dzlog_info("get a vision loopclosure .time is %f and %f .", loop_time_first, loop_time_second);
        // 加锁 ，防止和前面push的动作冲突
        g_Mutex.lock();
        loop_time_queue.pop();
        g_Mutex.unlock();
        ROS_INFO("now loop_time_queue size is %ld .", loop_time_queue.size());
        dzlog_info("now loop_time_queue size is %ld .", loop_time_queue.size() );

        // 每次进来，还从 第一个 开始
        unsigned int iKey = 0;
        // unsigned int iKey = 1;
        
        int last_key_cnts = -1;

        while (1)
        {
            // 跳帧 加快计算
            iKey = iKey + detect_step_;

            // 现在的点云id 不能太靠近最后几个
            if (iKey > key_ - skip_recent_poses_)
                break;

            if (iKey - last_closure_key_ < skip_recent_poses_)
                continue;

            PointCloud::Ptr scan(new PointCloud);
            try
		    {
                getKeyedScan(iKey, scan);
		    }
		    catch (const std::exception &e)
		    {
		    	std::cerr << e.what() << '\n';
		    	break;
		    }

            // 三种情况，1、比 loop_time_first 早的点云
            // 三种情况，2、在 loop_time_first 附近的点云 取这个情况
            // 三种情况，3、比 loop_time_first 晚的点云
            // 认为视觉回环的 detect_time_regional_ s内的点云 ， 也应该可以检测到回环,这样来缩小的搜索的范围
            // ROS_WARN("1. time diff %f. detect_time_regional_ %f", std::fabs( scan->header.stamp - loop_time_first ), detect_time_regional_);
            if ( std::fabs( scan->header.stamp - loop_time_first ) < detect_time_regional_)
            {
                dzlog_info("@@@@@@ loopClosureThread() iCurHdlKeyM = %u,  total_key_ = %u .", iKey, key_);
                dzlog_info(" this key:%u time< %f > near have loop ......", iKey, loop_time_second);
                FindLoopClosures(iKey, loop_time_second);
            }
        }
    }
    dzlog_info("-------------------------------------------------------------------------  ");
    dzlog_info("----------------save result finish ...... -------------");
    dzlog_info("-------------------------------------------------------------------------  ");
    dzlog_info("@@@@@@ loopClosureThread() loop thread EXIT !!!");
}

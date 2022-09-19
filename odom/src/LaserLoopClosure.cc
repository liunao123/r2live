#include <odom/LaserLoopClosure.h>
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
#include <thread>
#include <zlog.h>
#include <iostream>

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

LaserLoopClosure::LaserLoopClosure():
key_(0),
last_closure_key_(0), //初始化一个
bIsLoopCheckedM(false),
bIsLoopThreadExitM(false),
bIsGetSaveMapM(false),
iCurHdlKeyM(0),
iLoopKeyM(0)
{
	std::cout << " construct fun : " << std::endl;
}

LaserLoopClosure::~LaserLoopClosure()
{
	std::cout << __FILE__ <<"  xigou  fun : " << std::endl;
}


void LaserLoopClosure::saveMap()
{
    std::cout << __FILE__ << ":" << __LINE__ << "  loopClosure Check DONE, SAVE pointcloud map .  " << std::endl;
    PointCloud* points = new PointCloud();
	GetMaximumLikelihoodPoints( points ) ; // points->makeShared()
	
	std::string name =  "/home/optimized_map_gtsam.pcd";
	pcl::io::savePCDFile(name, *points);
	cout << "map path is :" << name << endl;
}


bool LaserLoopClosure::Initialize()
{
    //***create loopclosure thread***//
    std::thread t1(&LaserLoopClosure::loopClosureThread,this);
    t1.detach();

	std::cout << " LoadParameters s  : " << std::endl;
    LoadParameters();
	std::cout << " LoadParameters e  : " << std::endl;
    return true;
}

bool LaserLoopClosure::LoadParameters() 
{
    loop_cnts = 0;
    /* */
    // Load frame ids.
    // if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;
    // if (!pu::Get("frame_id/base", base_frame_id_)) return false;

    // // Should we turn loop closure checking on or off?
    check_for_loop_closures_ = true;
    // if (!pu::Get("loop_closure/check_for_loop_closures", check_for_loop_closures_)) return false;

    // Load ISAM2 parameters.
    unsigned int relinearize_skip = 1;
    double relinearize_threshold = 0.01;
    // if (!pu::Get("loop_closure/relinearize_skip", relinearize_skip)) return false;
    // if (!pu::Get("loop_closure/relinearize_threshold", relinearize_threshold)) return false;

    // // Load loop closing parameters.
    translation_threshold_ =  0.25; //0.25
    proximity_threshold_ = 15; // 10
    max_tolerable_fitness_ = 0.24 ; //0.36;
    skip_recent_poses_ = 10; // 20
    poses_before_reclosing_ = 10;
    maxLoopKeysYawM = 1.05;
    // if (!pu::Get("loop_closure/translation_threshold", translation_threshold_)) return false;
    // if (!pu::Get("loop_closure/proximity_threshold", proximity_threshold_)) return false;
    // if (!pu::Get("loop_closure/max_tolerable_fitness", max_tolerable_fitness_)) return false;
    // if (!pu::Get("loop_closure/skip_recent_poses", skip_recent_poses_)) return false;
    // if (!pu::Get("loop_closure/poses_before_reclosing", poses_before_reclosing_)) return false;
    // if (!pu::Get("loop_closure/max_loopkeys_yaw", maxLoopKeysYawM)) return false;
    // dzlog_info("@@@@@@ LoadParameters() maxLoopKeysYawM = %f",maxLoopKeysYawM);

    // Load ICP parameters.

    // if (!pu::Get("icp/tf_epsilon", icp_tf_epsilon_)) return false;
    // if (!pu::Get("icp/corr_dist", icp_corr_dist_)) return false;
    // if (!pu::Get("icp/iterations", icp_iterations_)) return false;

    // Load initial position and orientation.
    
    // double init_x = 0.01, init_y = 0.01, init_z = 0.01;
    // double init_roll = 0.001, init_pitch = 0.001, init_yaw = 0.001;

    double init_x = 0.001, init_y = 0.001, init_z = 0.001;
    double init_roll = 0.00001, init_pitch = 0.00001, init_yaw = 0.00001;

    // if (!pu::Get("init/position/x", init_x)) return false;
    // if (!pu::Get("init/position/y", init_y)) return false;
    // if (!pu::Get("init/position/z", init_z)) return false;
    // if (!pu::Get("init/orientation/roll", init_roll)) return false;
    // if (!pu::Get("init/orientation/pitch", init_pitch)) return false;
    // if (!pu::Get("init/orientation/yaw", init_yaw)) return false;

    // Load initial position and orientation noise.

    // double sigma_x = 0.01, sigma_y = 0.01, sigma_z = 0.01;
    // double sigma_roll = 0.001, sigma_pitch = 0.001, sigma_yaw = 0.001;
    double sigma_x = 0.001, sigma_y = 0.001, sigma_z = 0.001;
    double sigma_roll = 0.00001, sigma_pitch = 0.00001, sigma_yaw = 0.00001;

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
    
    std::cout << __FILE__ << ":" << __LINE__ << " : LoadParameters : "  << std::endl;

    return true;
}

bool LaserLoopClosure::RegisterCallbacks(const ros::NodeHandle& n) 
{
	// Create a local nodehandle to manage callback subscriptions.
	ros::NodeHandle nl(n);

	odometry_edge_pub_ =
		nl.advertise<visualization_msgs::Marker>("odometry_edges", 10, false);
	loop_edge_pub_ =
		nl.advertise<visualization_msgs::Marker>("loop_edges", 10, false);
	graph_node_pub_ =
		nl.advertise<visualization_msgs::Marker>("graph_nodes", 10, false);
	keyframe_node_pub_ =
		nl.advertise<visualization_msgs::Marker>("keyframe_nodes", 10, false);
	closure_area_pub_ =
		nl.advertise<visualization_msgs::Marker>("closure_area", 10, false);

	// pose_graph_pub_ =
	// 	nl.advertise<velodyne_slam::PoseGraph>("pose_graph", 10, false);
	// keyed_scan_pub_ =
	// 	nl.advertise<velodyne_slam::KeyedScan>("keyed_scans", 10, false);
	// loop_closure_notifier_pub_ =
	// 	nl.advertise<std_msgs::Empty>("loop_closure", 10, false);

	return true;
}



void LaserLoopClosure::GetMaximumLikelihoodPoints(PointCloud* points)
{
    if (points == NULL)
    {
        dzlog_info("%s: Output point cloud container is null.", name_.c_str());
        return;
    }
    points->points.clear();

    // 会覆盖已经有的文件
    ofstream GTSAM_pose("/home/GTSAM_pose.txt", ios::out);

    // Iterate over poses in the graph, transforming their corresponding laser
    // scans into world frame and appending them to the output.
    for (const auto& keyed_pose : values_)
    {
        const unsigned int key = keyed_pose.key;

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

    // 跳过第一个，因为第一个是0 
    if (key < 1)
        continue;
    
    // 往上面的文件里写
     GTSAM_pose.setf(ios::fixed, ios::floatfield);
     GTSAM_pose.precision(10);
    //  GTSAM_pose << key  << " ";
     GTSAM_pose << keyed_scans_[key]->header.stamp  << " ";
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
    
    // 把上面的文件 关闭
    GTSAM_pose.close();
}

gu::Transform3 LaserLoopClosure::GetLastPose() const 
{
    unsigned int lastKey = (key_>1)?(key_-1):0;
    Pose3 lastKeyPose = values_.at<Pose3>(lastKey);
    Pose3 curPose = lastKeyPose.compose(odometry_);
    dzlog_info("@@@@@@ GetLastPose() curkey = %d,lastkey = %d",key_,lastKey);
    dzlog_info("@@@@@@ GetLastPose() increodom dist = %f,yaw = %f",odometry_.translation().norm(),odometry_.rotation().yaw());
    dzlog_info("@@@@@@ GetLastPose() lastkeypose x = %f,y = %f",lastKeyPose.translation().x(),lastKeyPose.translation().y());
    dzlog_info("@@@@@@ GetLastPose() curpose x = %f,y = %f",curPose.translation().x(),curPose.translation().y());
    return ToGu(curPose);
}

gu::Transform3 LaserLoopClosure::ToGu(const Pose3& pose) const 
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

Pose3 LaserLoopClosure::ToGtsam(const gu::Transform3& pose) const 
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
	const LaserLoopClosure::Gaussian::shared_ptr& covariance) const 
{
    gtsam::Matrix66 gtsam_covariance = covariance->covariance();

    LaserLoopClosure::Mat66 out;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        out(i, j) = gtsam_covariance(i, j);

    return out;
}

LaserLoopClosure::Gaussian::shared_ptr LaserLoopClosure::ToGtsam(
	const LaserLoopClosure::Mat66& covariance) const 
{
    gtsam::Matrix66 gtsam_covariance;

    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        gtsam_covariance(i, j) = covariance(i, j);

    return Gaussian::Covariance(gtsam_covariance);
}

PriorFactor<Pose3> LaserLoopClosure::MakePriorFactor(
	const Pose3& pose,
	const LaserLoopClosure::Diagonal::shared_ptr& covariance) 
{
    return PriorFactor<Pose3>(key_, pose, covariance);
}

BetweenFactor<Pose3> LaserLoopClosure::MakeBetweenFactor(
	const Pose3& delta,
	const LaserLoopClosure::Gaussian::shared_ptr& covariance) 
{
    odometry_edges_.push_back(std::make_pair(key_-1, key_));
    return BetweenFactor<Pose3>(key_-1, key_, delta, covariance);
}



void LaserLoopClosure::Serialize2File(string posePath, string normal_HistogramPath, string height_HistogramPath,  float x_offset, float y_offset)
{
	std::ofstream pose_file; //输出位姿到文件
	pose_file.open(posePath.c_str(),std::ofstream::out | std::ofstream::app);
	if(!pose_file)
	{
		std::cout<<"file not open(pose)!!";
	}

	std::ofstream height_histo_file; //输出直方图表示到文件
	height_histo_file.open(height_HistogramPath.c_str(),std::ofstream::out | std::ofstream::app);
	if(!height_histo_file)
	{
		std::cout<<"file not open(histogram)!!";
	}
	std::ofstream normal_histo_file; //输出直方图表示到文件
	normal_histo_file.open(normal_HistogramPath.c_str(),std::ofstream::out | std::ofstream::app);
	if(!normal_histo_file)
	{
		std::cout<<"file not open(normal_histogram)!!";
	}

	std::map<unsigned int, vector<float> >::iterator it = keyed_histogram_.begin();
	std::map<unsigned int, vector<float> >::iterator it2 = keyed_histogram_normal_.begin();
	for (; it!=keyed_histogram_.end() && it2!=keyed_histogram_normal_.end(); ++it,++it2)
	{
		unsigned int cur_key = it->first;
		for(int i=0;i<it->second.size();i++)
		{
			height_histo_file<<it->second[i]<<" ";
		}
		height_histo_file<<std::endl;

		for(int i=0;i<it2->second.size();i++)
		{
			normal_histo_file<<it2->second[i]<<" ";
		}
		normal_histo_file<<std::endl;

		Pose3 p = values_.at<Pose3>(cur_key);
		float x = p.translation().x() - x_offset;
		float y = p.translation().y() - y_offset;
		float yaw = p.rotation().yaw();
		pose_file<<x<<" "<<y<<" "<<yaw<<std::endl;
	}
}

bool LaserLoopClosure::addPoseAndKeyScan(const geometry_utils::Transform3 &delta,const PointCloud::ConstPtr &scan)
{
    unsigned int pose_key ;
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

    if (!AddBetweenFactor(delta,covariance,stamp,&pose_key))
    {
        isamMutexM.unlock();
        return false;
    }
    isamMutexM.unlock();


    if (!AddKeyScanPair(pose_key,scan))
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
    pcl::copyPointCloud(*tmpScan,*scan);
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

    NonlinearFactorGraph new_factor;
    new_factor.add(BetweenFactor<Pose3>(iCurHdlKeyM,iLoopKeyM,ToGtsam(loopDeltaM),ToGtsam(covariance)));
    isam_->update(new_factor, Values());
    values_ = isam_->calculateEstimate();//原来是有无回环都调用一次，但是感觉无回环没必要调用

    last_closure_key_ = iCurHdlKeyM;
    loop_edges_.push_back(std::make_pair(iCurHdlKeyM,iLoopKeyM));
    // 标示我们发现了一个闭环
    loop_closure_notifier_pub_.publish(std_msgs::Empty());
    dzlog_info("@@@@@@ updatePoseFactor() update loop pose between key %u and %u",iCurHdlKeyM,iLoopKeyM);
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
    gtsam::Values value = isam_->calculateEstimate();
    NonlinearFactorGraph factorGraph = isam_->getFactorsUnsafe();
    gtsam::writeG2o(factorGraph,value,outputFile);
}



bool LaserLoopClosure::doICPMatchForScans(const PointCloud::ConstPtr &scan1, const PointCloud::ConstPtr &scan2,
                                          gu::Transform3 *delta,double *fitnessReturn)
{
    if (delta == NULL)
    {
        dzlog_info("###### doICPMatchForScans() pointer are null !!!");
        return false;
    }

    // ICP：此处考虑了闭合时，机器人可能不是一个朝向，此时直接ICP可能匹配不上，这时候
    //利用pose1/2时的朝向的估计信息。转换后再匹配。
    icp_tf_epsilon_ = 0.0000000001;
    icp_corr_dist_ = 0.25;
    icp_iterations_ = 30;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setTransformationEpsilon(icp_tf_epsilon_);
    icp.setMaxCorrespondenceDistance(icp_corr_dist_);
    icp.setMaximumIterations(icp_iterations_);

    icp.setEuclideanFitnessEpsilon(0.1);
    
    icp.setRANSACIterations(50);
    icp.setInputSource(scan1);
    icp.setInputTarget(scan2);
    PointCloud unused_result;
    icp.align(unused_result);

    if (!icp.hasConverged())
    {
        dzlog_info("###### doICPMatchForScans() ICP isn't coverged !!!");
        return false;
    }



    *fitnessReturn = icp.getFitnessScore();
    if (*fitnessReturn > max_tolerable_fitness_)
    {
        dzlog_info("###### doICPMatchForScans() score = %f,maxTolerScore = %f",*fitnessReturn, max_tolerable_fitness_);
        return false;
    }

    const Eigen::Matrix4f T = icp.getFinalTransformation();
    gu::Transform3 delta_icp;
    delta_icp.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
    delta_icp.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                  T(1, 0), T(1, 1), T(1, 2),
                                  T(2, 0), T(2, 1), T(2, 2));

    delta_icp.translation = gu::Vec3(delta_icp.translation.X(), delta_icp.translation.Y(), 0);
    delta_icp.rotation = gu::Rot3(0,0,delta_icp.rotation.Yaw());
    //反转坐标系，输出才是对的！
    *delta = gu::PoseInverse(delta_icp);

    return true;
}


void LaserLoopClosure::isHasLoopClosure(unsigned int iKey)
{
    bool bIsLoopClosed = false;
    unsigned int keyResult = 0;
    double fitnessMin = max_tolerable_fitness_;
    gu::Transform3 delta;

    PointCloud::Ptr scan1(new PointCloud);
    getKeyedScan(iKey,scan1);


    // 与该帧之前所有的帧做检测
    for (unsigned int i = 0; i < iKey  ;i ++ )
    // 与当前所有插入的帧做检测
    // for (unsigned int i = 0; i < getLastScanKey()  ;i ++ )
    {
        // 是不是离的很近
        if (std::fabs(iKey - i) < skip_recent_poses_)
        {
            continue;
        }

        // 是不是关键帧
        if (!isKeyFrame(i))
        {
            continue;
        }
    
        const gu::Transform3 diffPose = getDiffPose(iKey,i);

        
        // 位姿变化是不是过小
        dzlog_info("@@@@@@ i %u ,iKey = %u, key_cnts = %u .  diffPose.translation.Norm() = %f ."  ,i , iKey, getLastScanKey(),  diffPose.translation.Norm() );
        
        if ((diffPose.translation.Norm() < proximity_threshold_) &&
            (fabs(diffPose.rotation.Yaw()) < maxLoopKeysYawM))
        {
            PointCloud::Ptr scan2(new PointCloud);
            getKeyedScan(i,scan2);

            gu::Transform3 tmpDelta;
            double fitnessScore = 100.0;

            if (doICPMatchForScans(scan1,scan2,&tmpDelta,&fitnessScore))
            {
                if (fitnessScore < fitnessMin)
                {
                    fitnessMin = fitnessScore;
                    keyResult = i;
                    delta = tmpDelta;
                }

                bIsLoopClosed = true;

                //add by ln
                last_closure_key_ = iKey;

                gu::MatrixNxNBase<double, 6> covariance;
                covariance.Zeros();
                for (int i = 0; i < 3; ++i)
                  covariance(i, i) = 0.01;
                for (int i = 3; i < 6; ++i)
                  covariance(i, i) = 0.004;
        
                NonlinearFactorGraph new_factor;
                new_factor.add(BetweenFactor<Pose3>(iKey, i, ToGtsam(diffPose),ToGtsam(covariance)));
                isam_->update(new_factor, Values());
                values_ = isam_->calculateEstimate();

                // dzlog_info("@@@@@@ %u th looooooop loop_cnts : i %u ,iKey = %u",loop_cnts++ , i , iKey);
            
                dzlog_info("@@@@@@ %u th looooooop :score = %f, diff = %f, i=%u ,iKey = %u, DIFF is %u \n ",loop_cnts++ ,fitnessScore, diffPose.translation.Norm(),  i , iKey, iKey - i  );
                std::cout << __FILE__ << ":" << __LINE__ << " : loop_cnts : " << loop_cnts << "  . loop_cnts DIFF : " << iKey - i  << std::endl;

            }
        }


    }    
    
    //发现闭环, 找到那个得分最高的
    if (bIsLoopClosed)
    {
        dzlog_info("\n @@@@@@ isHasLoopClosure() find a loop closure between key %u and %u fitnessMin=%f, \n ",iKey, keyResult, fitnessMin);
        iLoopKeyM = keyResult;
        loopDeltaM = delta;
        bIsLoopCheckedM = true;
    }
    

}



bool LaserLoopClosure::PerformICP(const PointCloud::ConstPtr& scan1,
                                  const PointCloud::ConstPtr& scan2,
                                  const gu::Transform3& pose1,
                                  const gu::Transform3& pose2,
                                  gu::Transform3* delta,
                                  LaserLoopClosure::Mat66* covariance,
                                  double* fitnessReturn) 
{
    if (delta == NULL || covariance == NULL)
    {
        dzlog_info("%s: Output pointers are null.", name_.c_str());
        return false;
    }

    // ICP：此处考虑了闭合时，机器人可能不是一个朝向，此时直接ICP可能匹配不上，这时候
    //利用pose1/2时的朝向的估计信息。转换后再匹配。
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp_tf_epsilon_ = 0.0000000001;
    icp_corr_dist_ = 0.25;
    icp_iterations_ = 30;

    icp.setTransformationEpsilon(icp_tf_epsilon_);
    icp.setMaxCorrespondenceDistance(icp_corr_dist_);
    icp.setMaximumIterations(icp_iterations_);
    icp.setRANSACIterations(50);

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
        // dzlog_info("score %f", icp.getFitnessScore());
        return false;
    }

    // delta_icp.translation =  gu::Vec3(delta_icp.translation.X(), delta_icp.translation.Y(), 0);
    // delta_icp.rotation = gu::Rot3(0,0,delta_icp.rotation.Yaw());

    //反转坐标系，输出才是对的！
    *delta = gu::PoseInverse(delta_icp);
    // 查看论文，寻找ICP求协方差矩阵的算法
    covariance->Zeros();
    for (int i = 0; i < 2; ++i)
      (*covariance)(i, i) = 0.01;
    for (int i = 2; i < 5; ++i)
      (*covariance)(i, i) = 0.001;
    for (int i = 5; i < 6; ++i)
      (*covariance)(i, i) = 0.04;
    
    // icp的结果 
	cout << "icp rotation :" << endl << delta->rotation.GetEulerZYX() << endl;

    static int cnts = 0;
    std::string name;
	name = "/root/Desktop/r2/src/icp/_" + std::to_string(cnts) + "_scan2.pcd";
	cout << "754 path is :" << name << endl;
	pcl::io::savePCDFile(name, *scan2);
    

	name = "/root/Desktop/r2/src/icp/_" + std::to_string(cnts) + "_scan1.pcd";
	cout << "759 path is :" << name << endl;
	pcl::io::savePCDFile(name, *scan1);


    name = "/root/Desktop/r2/src/icp/_" + std::to_string(cnts) + "_unused_result.pcd";
	cout << "764 path is :" << name << endl;
	pcl::io::savePCDFile(name, unused_result);

    cnts++;

    return true;
}


bool LaserLoopClosure::FindLoopClosures(unsigned int key_temp)
{
    int key = key_temp;
    // If loop closure checking is off, don't do this step. This will save some
    // computation time.

    // if (!check_for_loop_closures_)
    //     return false;


    // If a loop has already been closed recently, don't try to close a new one.

    if (std::fabs(key - last_closure_key_) < poses_before_reclosing_)
        return false;

    // Get pose and scan for the provided key.
    const gu::Transform3 pose1 = ToGu(values_.at<Pose3>(key));
    const PointCloud::ConstPtr scan1 = keyed_scans_[key];   

    // 遍历所有之前位姿，找到与当前位姿最匹配的一个（闭环）
    bool closed_loop = false;
    double fitnessMin= max_tolerable_fitness_;
    gu::Transform3 delta;
    LaserLoopClosure::Mat66 covariance;
    int keyResut =0;

    int other_key;
    
    for (const auto& keyed_pose : values_)
    {
        other_key = keyed_pose.key;

        // std::cout << __FILE__ << "other_key" << other_key << std::endl;
        // dzlog_info("@@@@@@ i %u ,iKey = %u, key_cnts = %u .  "  ,other_key , key, getLastScanKey() );

        // Don't self-check.
        // Don't compare against poses that were recently collected.
        // 
        if (key - other_key < skip_recent_poses_)
            break;

        if (other_key % 2 != 0 )
        {
            continue;
        }


        // 是不是关键帧
        if (!isKeyFrame(other_key))
            continue;

        if (other_key < 10 || other_key > key - 20 )
        {
            dzlog_info("@@@@@@ other_key %u ,Key = %u, key_cnts = %u .  "  ,other_key , key, getLastScanKey() );
        }

        const gu::Transform3 pose2 = ToGu(values_.at<Pose3>(other_key));
        const gu::Transform3 difference = gu::PoseDelta(pose1, pose2);        

        // 不能考虑Z值，要不然高度上偏移会导致，进不去
        float diff_xy = difference.translation.X() * difference.translation.X() + difference.translation.Y() * difference.translation.Y();
        diff_xy = sqrt(diff_xy);

        if(other_key % 500 == 0  )
            dzlog_info("@@@@@@ i= %u ,iKey = %u, key_cnts = %u .  diffPose.translation.xy() = %f difference.translation.Norm() = %f . fabs(difference.rotation.Yaw()) = %f ."  ,other_key , key, getLastScanKey(),  diff_xy, difference.translation.Norm(), fabs( difference.rotation.Yaw()) );

        //proximity_threshold_表示一个半径为这么大的球体，比较是否落在球里面，若是则后续ICP判断
        if ( ( diff_xy < proximity_threshold_ ) &&
            (fabs(difference.rotation.Yaw()) < maxLoopKeysYawM) )
        // if ((difference.translation.Norm() < proximity_threshold_) &&
        //    (fabs(difference.rotation.Yaw()) < maxLoopKeysYawM))
        {   
            // dzlog_info("now is in the circle, between poses %u and %u.",key, other_key);
            const PointCloud::ConstPtr scan2 = keyed_scans_[other_key];


            gu::Transform3 deltaTemp;
            LaserLoopClosure::Mat66 covarianceTemp;
            double fitnessReturn;
            if (PerformICP(scan1, scan2, pose1, pose2, &deltaTemp, &covarianceTemp,&fitnessReturn))
            {
                // 找到一个闭环位姿
                if(fitnessReturn<fitnessMin)
                {
                    fitnessMin = fitnessReturn;
                    keyResut = other_key;
                    delta = deltaTemp;
                    covariance = covarianceTemp;
                }
                std::cout << __FILE__ << ":" << __LINE__ << " : loop_cnts : " << loop_cnts << " ,  key = " << key << "  other_key= " <<  other_key << "  . loop_cnts DIFF : " << key  - other_key  << std::endl;
                dzlog_info(" %uth loop closure between key %u and %u , fitnessMin %f diff %f, DIFF key = %u .  ", loop_cnts, other_key , key, fitnessMin, difference.translation.Norm(), key  - other_key);
                printf(" %uth loop closure between key %u and %u , fitnessMin %f diff %f, DIFF key = %u .  \n\n ", loop_cnts, other_key , key, fitnessMin, difference.translation.Norm(), key  - other_key);
                closed_loop = true;
            }
        }
    }
    
    if(closed_loop)
    {
        std::cout << __FILE__ << ":" << __LINE__ << "  choose one min score ......, key is "<< key << ",  keyResut is " << keyResut << std::endl;

        std::cout << delta << std::endl;

        dzlog_info(" delta is: translation: %f, %f, %f .", delta.translation(0), delta.translation(1), delta.translation(2) );
        auto rota_temp = delta.rotation.GetEulerZYX() * 180 / M_PI;
        dzlog_info(" delta is: rotation: %f, %f, %f (deg).", rota_temp(0), rota_temp(1), rota_temp(2) );       
        
        NonlinearFactorGraph new_factor;

        new_factor.add(BetweenFactor<Pose3>(key, keyResut, ToGtsam(delta),ToGtsam(covariance)));
        isam_->update(new_factor, Values());

        // closure_keys用于存储查询到与当前key闭环检测到的other_key合集
        loop_edges_.push_back(std::make_pair(key, keyResut));

        dzlog_info("\n\n\n @@@@@@ %uth isHasLoopClosure() find a loop closure between key %u and %u fitnessMin=%f, DIFF key = %u . last_closure_key_ = %u \n\n ",loop_cnts, key, keyResut , fitnessMin, key  - keyResut, last_closure_key_);
        loop_cnts++;
        last_closure_key_ = key;

    }
    values_ = isam_->calculateEstimate();
    return closed_loop;

}


bool LaserLoopClosure::AddBetweenFactor(const gu::Transform3& delta,
                                        const LaserLoopClosure::Mat66& covariance,
                                        const ros::Time& stamp, unsigned int* key)
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

    // 不能考虑Z值，要不然高度上偏移会导致，进不去
    // float diff_xy = odometry_.translation().x() * odometry_.translation().x() + odometry_.translation().y() * odometry_.translation().y();
    // diff_xy = sqrt(diff_xy);
    // // dzlog_info("@@@@@@ diff pose : norm = %f , dist = %f,yaw = %f",odometry_.translation().norm(), diff_xy, odometry_.rotation().yaw());

    // if ( ( diff_xy < translation_threshold_ ) &&
    //     (fabs(odometry_.rotation().yaw()) < 0.3) )
    // {
    //     return false;
    // }


    NonlinearFactorGraph new_factor;
    Values new_value;
    new_factor.add(MakeBetweenFactor(odometry_/*new_odometry*/, ToGtsam(covariance)));

    Pose3 last_pose = values_.at<Pose3>(key_-1);
    new_value.insert(key_, last_pose.compose(odometry_/*new_odometry*/));

    // Store this timestamp so that we can publish the pose graph later.
    keyed_stamps_.insert(std::pair<unsigned int, ros::Time>(key_, stamp));

    // Update ISAM2.
    isam_->update(new_factor, new_value);
    values_ = isam_->calculateEstimate();

    // Assign output and get ready to go again!
    *key = key_++;

    odometry_ = Pose3::identity();
    return true;

}


bool LaserLoopClosure::AddKeyScanPair(unsigned int key,const PointCloud::ConstPtr& scan)
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    if (keyed_scans_.count(key))
    {
        dzlog_info("%s: Key %u already has a laser scan.", name_.c_str(), key);
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
    // Publish the inserted laser scan.
    // if (keyed_scan_pub_.getNumSubscribers() > 0)
    // {
    //     velodyne_slam::KeyedScan keyed_scan;
    //     keyed_scan.key = key;

    //     pcl::toROSMsg(*scan, keyed_scan.scan);
    //     keyed_scan_pub_.publish(keyed_scan);
    // }
    return true;
}


void LaserLoopClosure::loopClosureThread()
{
    dzlog_info("@@@@@@ loopClosureThread() loop thread IN !!!");
    
    // if (!check_for_loop_closures_)
    // {
    //     bIsLoopThreadExitM = true;
    //     return;
    // }

    unsigned int iKey = 1; //
    int wait_secs = 200;
    while(iKey < wait_secs)
    {
        usleep(1000*1000);
        std::cout << __FILE__ << ":" << __LINE__ << " wait for "<<  iKey++ << " s." << std::endl;
    }

    // 从第 几 帧 开始检测回环
    // iKey = wait_secs * 9;
    iKey = int ( getLastScanKey() / 2 );
    
    while(1)
    {
        usleep(2*1000);

        dzlog_info("@@@@@@ loopClosureThread() iCurHdlKeyM = %u,  total_key_ = %u",iCurHdlKeyM, key_);

        if (bIsLoopCheckedM)
        {
            // continue;
            bIsLoopCheckedM = false;
        }
        if (bIsGetSaveMapM)
        {
            dzlog_info("@@@@@@ loopClosureThread() loop checked DONE !!!");
            break;
        }

        if( iCurHdlKeyM > getLastScanKey() - 20 )
        {
            continue;
        }

        iKey = iKey + 2;
        // iCurHdlKeyM = iKey;
        
        iCurHdlKeyM = iKey ;

        if (iCurHdlKeyM - last_closure_key_ < poses_before_reclosing_)
            continue;

        if (key_ - iCurHdlKeyM  < skip_recent_poses_)
            break;
        // if (iCurHdlKeyM == iKey)
        //     continue;
       
        // isHasLoopClosure(iCurHdlKeyM);

        FindLoopClosures(iCurHdlKeyM);

    }
    
    bIsLoopThreadExitM = true;
    dzlog_info("@@@@@@ loopClosureThread() loop thread EXIT !!!");
    
    std::cout << __FILE__ << ":" << __LINE__ << "  loopClosureThread EXIT , DONE ......  " << std::endl;

}

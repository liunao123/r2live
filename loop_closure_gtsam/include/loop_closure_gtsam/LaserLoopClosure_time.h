#ifndef LIDAR_LOOP_CLOSURE_H
#define LIDAR_LOOP_CLOSURE_H

#include <ros/ros.h>
#include <map>
#include <vector>
#include <queue>
#include <fstream>
#include <mutex>
#include <geometry_utils/Matrix3x3.h>
#include <geometry_utils/Transform3.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <base_controller/CSGMapInfo.h>
#include <Util/util.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/Trigger.h>
#include "loop_closure_gtsam/LoopTimePair.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/OccupancyGrid.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <visualization_msgs/Marker.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <thread>
#include <zlog.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <sys/stat.h>

#include <nav_msgs/OccupancyGrid.h>
#include "loop_closure_gtsam/gridMap.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/tf.h>
#include <cstdint>
#include <cstring>
#include <png.h>

using namespace std;

typedef pcl::PointXYZRGB PointType;

typedef pcl::PointCloud<PointType> PointCloud;
typedef geometry_utils::MatrixNxNBase<double, 6> Mat66;
typedef gtsam::noiseModel::Gaussian Gaussian;
typedef gtsam::noiseModel::Diagonal Diagonal;
// typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;

class LidarLoopClosure
{
public:
    LidarLoopClosure();
    virtual ~LidarLoopClosure();

    bool Initialize(const ros::NodeHandle &n);
    void saveMap(int regionID,int mapID);
    bool save_g2o_file(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void setVisionLoopTime(const std::vector< std::pair< double , double > > & loop_time);
    void setOneLoopTime(const double first, const double second);
    void setMapThreadDone();
    void savePgm(nav_msgs::OccupancyGrid *map,std::string mapname);
    void saveOccupancyGridMap2png(const nav_msgs::OccupancyGrid& occupancyGrid, const std::string& filename);
    nav_msgs::OccupancyGrid genOccu(PointCloud::Ptr map_without_road,PointType minPt,PointType maxPt);
    PointCloud::Ptr pointFilter(PointCloud::Ptr map_data);

    bool AddBetweenFactor(const geometry_utils::Transform3 &delta,const Mat66 &covariance,
                          const ros::Time &stamp,unsigned int *key);
    bool AddKeyScanPair(unsigned int key, const PointCloud::ConstPtr &scan);
    bool FindLoopClosures(unsigned int key , const double &target_time);
    void GetMaximumLikelihoodPoints(PointCloud* map);
    geometry_utils::Transform3 GetLastPose() const;

    //***Added by chych 回环单独线程***//
    bool addPoseAndKeyScan(const geometry_utils::Transform3 &delta,const PointCloud::ConstPtr &scan);
    void loopClosureThread();
    unsigned int getLastScanKey();
    bool isKeyFrame(unsigned int iKey);
    void getKeyedScan(unsigned int iKey,PointCloud::Ptr scan);
    const geometry_utils::Transform3 getDiffPose(unsigned int iKey1,unsigned int iKey2);

    void updatePoseFactor();
    bool getIsLoopThreadExit();
    //***保存因子图***//
    void saveGtsam2G2oFile(string outputFile);
    void setWorkPath(const std::string work_dir);
    void setTranslationThreshold(const double translation_threshold);
private:
    bool LoadParameters();
    bool RegisterCallbacks(const ros::NodeHandle &n);
    void add_pose_and_scan_to_gtsam_callback(const geometry_msgs::PoseStampedConstPtr &pose,
                                             const sensor_msgs::PointCloud2ConstPtr &scan);
    bool startLoopDetect(loop_closure_gtsam::LoopTimePair::Request &req,
                         loop_closure_gtsam::LoopTimePair::Response &res);
    void saveMapCallBack(const base_controller::CSGMapInfo &saveMapMsg);

    // Pose conversion from/to GTSAM format.
    geometry_utils::Transform3 ToGu(const gtsam::Pose3& pose) const;
    gtsam::Pose3 ToGtsam(const geometry_utils::Transform3& pose) const;

    // Covariance conversion from/to GTSAM format.
    Mat66 ToGu(const Gaussian::shared_ptr& covariance) const;
    Gaussian::shared_ptr ToGtsam(const Mat66& covariance) const;

    // Create prior and between factors.
    gtsam::PriorFactor<gtsam::Pose3> MakePriorFactor(const gtsam::Pose3& pose,
                                                     const Diagonal::shared_ptr& covariance);
    gtsam::BetweenFactor<gtsam::Pose3> MakeBetweenFactor(const gtsam::Pose3& pose,
                                                         const Gaussian::shared_ptr& covariance);
    // Perform ICP between two laser scans.
    bool PerformICP(const PointCloud::ConstPtr& scan1,
                    const PointCloud::ConstPtr& scan2,
                    const geometry_utils::Transform3& pose1,
                    const geometry_utils::Transform3& pose2,
                    geometry_utils::Transform3* delta,
                    Mat66* covariance,double* fitnessReturn);
    void filter_pointcloud( PointCloud & pc);
private:
    // 保存关键帧, 直方图表示和关键时间戳
    std::map<unsigned int, PointCloud::ConstPtr> keyed_scans_;
    std::map<unsigned int, ros::Time> keyed_stamps_;

    // Aggregate odometry until we can update the pose graph.
    gtsam::Pose3 odometry_;

    // Pose graph and ISAM2 parameters.
    bool bIsCheckLoopClosureM;
    unsigned int key_;
    unsigned int last_closure_key_;
    unsigned int skip_recent_poses_;
    unsigned int keyframeCntM;
    double translation_threshold_;
    double proximity_threshold_;
    double max_tolerable_fitness_;
    double maxLoopKeysYawM;

    // ICP parameters.
    //double icp_ransac_thresh_;
    double icp_tf_epsilon_;
    double icp_corr_dist_;
    unsigned int icp_iterations_;

    // ISAM2 optimizer object, and best guess pose values.
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::Values values_;

    // Visualization publishers.
    ros::Publisher pubLoopFindM;
    ros::Publisher pubLoopCnt;    
    ros::Publisher pubSaveMapStsM;
    ros::Subscriber sub_saveMap;
    ros::ServiceServer server;
    ros::ServiceServer server_debug_g2o;

    message_filters::Subscriber<geometry_msgs::PoseStamped> *sub_lidar_pose;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_surf_points;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    //typedef std::pair<unsigned int, unsigned int> Edge;
    //std::vector<Edge> odometry_edges_;
    //std::vector<Edge> loop_edges_;

    //***added by chych 回环单独线程***//
    bool bIsLoopThreadExitM;
    unsigned int iCurHdlKeyM;
    unsigned int iLoopKeyM;
    mutable std::mutex isamMutexM;
    mutable std::mutex keyScanMutexM;
    geometry_utils::Transform3 loopDeltaM;
    geometry_utils::Transform3 lastPoseM;

    // add by ln
    unsigned int loop_cnts_;
    int detect_step_;
    double detect_time_regional_;
    double max_detect_range_;
    
    // 记录时间回环的时间戳 成对
    std::vector< std::pair< double , double > > loop_time_;
    std::queue< std::pair< double , double > > loop_time_queue;
    std::string strWorkDirM;
    std::mutex g_Mutex;
    std::vector< geometry_msgs::PoseStampedConstPtr > vecLidarPoseM;

    pcl::octree::OctreePointCloudSearch<PointType>::Ptr g_mapOctree;

};

#endif

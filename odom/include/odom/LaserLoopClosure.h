#ifndef LASER_LOOP_CLOSURE_H
#define LASER_LOOP_CLOSURE_H

#include <ros/ros.h>
#include <geometry_utils/Matrix3x3.h>
#include <geometry_utils/Transform3.h>
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
#include <Util/util.h>
#include <pcl_ros/point_cloud.h>
#include <map>
#include <vector>
#include <fstream>
#include <mutex>

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class LaserLoopClosure 
{
public:
    LaserLoopClosure();
    ~LaserLoopClosure();

    bool Initialize();
    void saveMap();


    // Typedef for 6x6 covariance matrices (x, y, z, roll, pitch, yaw).
    typedef geometry_utils::MatrixNxNBase<double, 6> Mat66;

    // Typedef for stored point clouds.

    // Call this every time the robot's pose has been updated via ICP or some
    // other form of odometry. A between factor will always be added, but this
    // function will only return true when the new pose is significantly
    // different from the most recently added pose to enforce graph sparsity.
    // A return value of true lets the caller know when they should call
    // AddKeyScanPair().
    bool AddBetweenFactor(const geometry_utils::Transform3& delta,const Mat66& covariance,
                          const ros::Time& stamp,unsigned int* key);


    // Upon successful addition of a new between factor, call this function to
    // associate a laser scan with the new pose.
    bool AddKeyScanPair(unsigned int key, const PointCloud::ConstPtr& scan);

    // After receiving an output key from 'AddBetweenFactor', call this to check
    // for loop closures with other poses in the pose graph.
    bool FindLoopClosures(unsigned int key);
    // bool FindLoopClosures(unsigned int key,std::vector<unsigned int>* closure_keys);


    // Build a 3D point cloud by concatenating all point clouds from poses along
    // the pose graph.
    void GetMaximumLikelihoodPoints(PointCloud* map);

    // Get the most recent pose in the pose graph.
    geometry_utils::Transform3 GetLastPose() const;

    // Publish pose graph for visualization.
    // void PublishPoseGraph();

    void Serialize2File(string posePath, string normal_HistogramPath,
                        string height_HistogramPath,float x_offset, float y_offset);

    //***Added by chych 回环单独线程***//
    bool addPoseAndKeyScan(const geometry_utils::Transform3 &delta,const PointCloud::ConstPtr &scan);
    void loopClosureThread();
    unsigned int getLastScanKey();
    bool isKeyFrame(unsigned int iKey);
    void getKeyedScan(unsigned int iKey,PointCloud::Ptr scan);
    void isHasLoopClosure(unsigned int iKey);
    const geometry_utils::Transform3 getDiffPose(unsigned int iKey1,unsigned int iKey2);
    bool doICPMatchForScans(const PointCloud::ConstPtr &scan1,
                            const PointCloud::ConstPtr &scan2,
                            geometry_utils::Transform3 *delta,
                            double *fitnessReturn);
    void updatePoseFactor();
    void setIsLoopChecked(bool bStatus);
    bool getIsLoopCheched();
    void setIsGetSaveMapFlag(bool bStatus);
    bool getIsLoopThreadExit();
    //***保存因子图***//
    void saveGtsam2G2oFile(string outputFile);
private:
    bool LoadParameters();
    bool RegisterCallbacks(const ros::NodeHandle& n);

    // Pose conversion from/to GTSAM format.
    geometry_utils::Transform3 ToGu(const gtsam::Pose3& pose) const;
    gtsam::Pose3 ToGtsam(const geometry_utils::Transform3& pose) const;

    // Covariance conversion from/to GTSAM format.
    typedef gtsam::noiseModel::Gaussian Gaussian;
    typedef gtsam::noiseModel::Diagonal Diagonal;
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

private:
    // Node name.
    std::string name_;

    // 保存关键帧, 直方图表示和关键时间戳
    std::map<unsigned int, PointCloud::ConstPtr> keyed_scans_;
    std::map<unsigned int, vector<float> > keyed_histogram_;
    std::map<unsigned int, vector<float> > keyed_histogram_normal_;
    std::map<unsigned int, ros::Time> keyed_stamps_;

    // Aggregate odometry until we can update the pose graph.
    gtsam::Pose3 odometry_;

    // Pose graph and ISAM2 parameters.
    bool check_for_loop_closures_;
    unsigned int key_;
    unsigned int last_closure_key_;
    unsigned int relinearize_interval_;
    unsigned int skip_recent_poses_;
    unsigned int poses_before_reclosing_;
    double translation_threshold_;
    double proximity_threshold_;
    double max_tolerable_fitness_;
    double maxLoopKeysYawM;

    // ICP parameters.
    double icp_ransac_thresh_;
    double icp_tf_epsilon_;
    double icp_corr_dist_;
    unsigned int icp_iterations_;

    // ISAM2 optimizer object, and best guess pose values.
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::Values values_;

    // Frames.
    std::string fixed_frame_id_;
    std::string base_frame_id_;

    // Visualization publishers.
    ros::Publisher odometry_edge_pub_;
    ros::Publisher loop_edge_pub_;
    ros::Publisher graph_node_pub_;
    ros::Publisher keyframe_node_pub_;
    ros::Publisher closure_area_pub_;
    // Pose graph publishers.
    ros::Publisher pose_graph_pub_;
    ros::Publisher keyed_scan_pub_;
    ros::Publisher loop_closure_notifier_pub_;

    typedef std::pair<unsigned int, unsigned int> Edge;
    std::vector<Edge> odometry_edges_;
    std::vector<Edge> loop_edges_;

    //***added by chych 回环单独线程***//
    bool bIsLoopCheckedM;
    bool bIsLoopThreadExitM;
    bool bIsGetSaveMapM;
    unsigned int iCurHdlKeyM;
    unsigned int iLoopKeyM;
    mutable std::mutex isamMutexM;
    mutable std::mutex keyScanMutexM;
    geometry_utils::Transform3 loopDeltaM;


    // add by ln
    unsigned int loop_cnts;
    
};

#endif

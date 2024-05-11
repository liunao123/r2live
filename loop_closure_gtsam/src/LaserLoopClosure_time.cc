#include "loop_closure_gtsam/LaserLoopClosure_time.h"
#include <algorithm>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

using namespace sensor_msgs;
using namespace message_filters;

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

LidarLoopClosure::LidarLoopClosure() :
key_(0),
last_closure_key_(0),
keyframeCntM(0),
bIsLoopThreadExitM(false),
bIsCheckLoopClosureM(true),
iCurHdlKeyM(0),
iLoopKeyM(0),
loop_cnts_(0),
detect_time_regional_(2.5), // 在视觉回环发生的多少时间范围内，进行点云匹配
detect_step_(2),            // 点云检测的步长，跳着去匹配
max_detect_range_(60.0),
strWorkDirM("/home/map/")
{
}

LidarLoopClosure::~LidarLoopClosure()
{
}

PointCloud::Ptr LidarLoopClosure::pointFilter(PointCloud::Ptr map_data)
{
    dzlog_info("beformap_data_.size() = %d \n",int(map_data->size()));
    pcl::PointCloud<PointType>::Ptr outputData(new pcl::PointCloud<PointType>());
    g_mapOctree->setInputCloud(outputData);
    for (size_t ii = 0; ii < map_data->points.size(); ++ii)
    {
        const PointType p = map_data->points[ii];
        double min_x, min_y, min_z, max_x, max_y, max_z;
        g_mapOctree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
        bool isInBox = (p.x >= min_x && p.x <= max_x) && (p.y >= min_y && p.y <= max_y) && (p.z >= min_z && p.z <= max_z);
        if (!isInBox || !g_mapOctree->isVoxelOccupiedAtPoint(p))
        {
            g_mapOctree->addPointToCloud(p,outputData);
        }
    }
    map_data->clear();
    pcl::copyPointCloud(*outputData, *map_data);

    dzlog_info("@@@@@@ after voxel grid = %d \n", int(map_data->size()));
    return map_data;
}

void LidarLoopClosure::saveOccupancyGridMap2png(const nav_msgs::OccupancyGrid& occupancyGrid, const std::string& filename) {
    int width = occupancyGrid.info.width;
    int height = occupancyGrid.info.height;

    std::string mapdatafile = filename + ".png";

    // 创建一个二维数组来保存占用地图数据
    std::vector<std::vector<uint8_t>> occupancyData(height, std::vector<uint8_t>(width, 255));

    // 将占用地图数据复制到二维数组中
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int index = j + (height - i - 1) * width;
            int8_t occupancyValue = occupancyGrid.data[index];
            // printf(" %d ", occupancyValue);
            if (occupancyValue == 0) {
                occupancyData[i][j] = 255; // 自由空间为白色
            } else if (occupancyValue == 100) {
                occupancyData[i][j] = 0; // 障碍物为黑色
            } 
            // else {
            //     occupancyData[i][j] = 200; // 未知区域为灰色 127
            // }
        }
    }

    // 打开PNG文件进行写入
    FILE* file = fopen(mapdatafile.c_str(), "wb");
    if (!file) {
        std::cout << "无法打开PNG文件" << std::endl;
        return;
    }

    // 初始化PNG结构
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        std::cout << "无法创建PNG写入结构" << std::endl;
        fclose(file);
        return;
    }

    // 初始化PNG信息
    png_infop info = png_create_info_struct(png);
    if (!info) {
        std::cout << "无法创建PNG信息结构" << std::endl;
        png_destroy_write_struct(&png, nullptr);
        fclose(file);
        return;
    }

    // 设置PNG文件IO
    png_init_io(png, file);

    // 设置PNG图像信息
    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    // 写入PNG头部
    png_write_info(png, info);

    // 写入PNG数据
    std::vector<png_bytep> rowPointers(height);
    for (int i = 0; i < height; ++i) {
        rowPointers[i] = reinterpret_cast<png_bytep>(occupancyData[i].data());
    }
    png_write_image(png, rowPointers.data());

    // 写入PNG结束
    png_write_end(png, nullptr);

    // 清理资源
    png_destroy_write_struct(&png, &info);
    fclose(file);

    std::cout << "占用地图已保存为PNG文件" << std::endl;

    // // 写入对应的 yaml
    // std::string mapmetadatafile = filename + ".yaml";
    // printf("Writing map yaml to %s \n", mapmetadatafile.c_str());
    // FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

    // geometry_msgs::Quaternion orientation = occupancyGrid.info.origin.orientation;
    // Eigen::Quaterniond quaternion(orientation.w,orientation.x, orientation.y, orientation.z); // (w, x, y, z)
    // // 将四元数转换为欧拉角表示
    // Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序

    // // 获取欧拉角的yaw、pitch和roll值
    // double yaw = euler[0];
    // // double pitch = euler[1];
    // // double roll = euler[2];

    // fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
    //         mapdatafile.c_str(), occupancyGrid.info.resolution, occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y, yaw);

    // fclose(yaml);
}

nav_msgs::OccupancyGrid LidarLoopClosure::genOccu(PointCloud::Ptr map_without_road,PointType minPt,PointType maxPt)
{
    nav_msgs::OccupancyGrid grid_msg;
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(map_without_road);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(3,5);
    pass.filter(*map_without_road);
    printf("after filter map_without_road.size() = %d \n", int(map_without_road->size()));

    GridMap *genOccuGrid_gridMap;
    pcl::getMinMax3D(*map_without_road, minPt, maxPt);
    printf("with minPt x %f y %f z %f \n", minPt.x, minPt.y, minPt.z);
    printf("with maxPt x %f y %f z %f \n", maxPt.x, maxPt.y, maxPt.z);
    printf("with minPt x %f y %f z %f \n", minPt.x, minPt.y, minPt.z);
    printf("with maxPt x %f y %f z %f \n", maxPt.x, maxPt.y, maxPt.z);

    int occu_x_dim = (maxPt.x - minPt.x) / 0.1 +1;
    int occu_y_dim = (maxPt.y - minPt.y) / 0.1 +1;
    printf("occu_x_dim %d \n",occu_x_dim);
    printf("occu_y_dim %d \n",occu_y_dim);
    genOccuGrid_gridMap = new GridMap(occu_x_dim, occu_y_dim,0.1);
    //初始化网格，用点云填充
    for(auto point: map_without_road->points)
    {
        pcl::PointXYZ pt; // 整体上用 pcl::PointXYZRGB 格式，这个单独用pcl::PointXYZ 不改变底层的结构
        pt.x = point.x - minPt.x;
        pt.y = point.y - minPt.y;
        genOccuGrid_gridMap->addToGrid(pt);
    }
    std::vector<signed char> ocGrid;
    genOccuGrid_gridMap->generateMem(ocGrid);
    delete genOccuGrid_gridMap;
    printf("ocGrid.size() %d \n",int(ocGrid.size()));
    grid_msg.header.seq++;
    grid_msg.header.frame_id = "map";
    grid_msg.header.stamp.sec = ros::Time::now().sec;
    grid_msg.header.stamp.nsec = ros::Time::now().nsec;
    grid_msg.info.map_load_time = ros::Time::now();
    grid_msg.info.resolution = 0.1;
    grid_msg.info.width = occu_x_dim;
    grid_msg.info.height = occu_y_dim;
    grid_msg.info.origin.position.x = minPt.x;
    grid_msg.info.origin.position.y = minPt.y;
    grid_msg.info.origin.position.z = minPt.z;
    double theta = atan2(minPt.y, minPt.x);
    printf(" origin theta %f \n", theta);
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
    grid_msg.info.origin.orientation = quat;
    grid_msg.data = ocGrid;
    printf("grid_msg %f %d \n", grid_msg.info.resolution,grid_msg.info.width);
    return grid_msg;
}

void LidarLoopClosure::savePgm(nav_msgs::OccupancyGrid *map,std::string mapname)
{
    std::string mapdatafile = mapname+".pgm";
    printf("Writing map occupancy data to %s \n", mapdatafile.c_str());
    FILE* out = fopen(mapdatafile.c_str(), "w");
    printf("CsgMapSaver:define out \n");
    if (!out)
    {
       printf("Couldn't save map file to %s", mapdatafile.c_str());

    return;
    }
    printf("CsgMapSaver:see the out value \n");

    fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);
    printf("write out success \n");
    printf("map->info.resolution %.3f\n",map->info.resolution);
    printf("map->info.width %d\n",map->info.width);
    printf("map->info.height %d\n",map->info.height);
    for(unsigned int y = 0; y < map->info.height; y++) {
    for(unsigned int x = 0; x < map->info.width; x++) {
        unsigned int i = x + (map->info.height - y - 1) * map->info.width;
        //printf("map->data[i] %d\n",map->data[i]);
        if (map->data[i] == 0) { //occ [0,0.1)
        fputc(254, out);
       // printf("write 254 \n");
        } else if (map->data[i] == +100) { //occ (0.65,1]
        fputc(000, out);
        //printf("write 000 \n");
        } else { //occ [0.1,0.65]
        fputc(205, out);
       // printf("write 205 \n");
        }
    }
    }
    printf("write finish \n");
    fclose(out);
    printf("close out \n");
    std::string mapmetadatafile = mapname + ".yaml";
    printf("Writing map yaml to %s \n", mapmetadatafile.c_str());
    FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

    geometry_msgs::Quaternion orientation = map->info.origin.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

    fclose(yaml);
}


void LidarLoopClosure::saveMap(int regionID, int mapID)
{
    dzlog_info("@@@@@@ saveMap() IN,regionID = %d,mapID = %d",regionID,mapID);

    PointCloud *points = new PointCloud();
    GetMaximumLikelihoodPoints(points);

    std::string strPath = strWorkDirM + "region" + std::to_string(regionID) + "-";
    strPath += std::to_string(mapID);

    struct stat info;
    if (stat(strPath.c_str(), &info) != 0) 
    { // stat()函数返回0表示成功
    }
    else if (info.st_mode & S_IFDIR)
    {   //S_IFDIR表示文件夹类型
        int iRtn = system( ("mv " + strPath + " " + strPath + "_bak" ).c_str() );//先备份一下
        dzlog_info("create a bak folder : %s.", (strPath + "_bak").c_str());
    }

    int iRtn2 = system(("mkdir -p " + strPath).c_str());//创建一个

    if(points->points.empty())
    {
        dzlog_info(" points is empty , return .");
        return;
    }

    dzlog_info("save pgm and yaml file done , file path is %s ...... ",strPath.c_str() );
    system( ("mv /home/map/pose_graph " + strPath).c_str() ); 
    dzlog_info(" move pose_graph to %s ...... ",strPath.c_str());

    system( ("mv /home/map/key_pose.txt " + strPath).c_str() );
    dzlog_info(" save key frame to %s ...... ",strPath.c_str());
    
    system( ("mv /home/map/fast_livo_body_path.txt " + strPath).c_str() );
    dzlog_info(" save fast_livo_body_path to %s ...... ",strPath.c_str());
    
    system( ("mv /home/map/GTSAM_pose.txt " + strPath).c_str() );
    dzlog_info(" save GTSAM_pose to %s ...... ",strPath.c_str());
    
    system( ("mv /home/map/vins_result_loop_time.txt " + strPath+"/vins_result_loop_time.sh").c_str() );
    dzlog_info(" save vins_result_loop_time to %s ...... ",strPath.c_str());

    system( ("mv /home/map/faster_lio_path.txt " + strPath).c_str() );
    dzlog_info(" save faster_lio_path to %s ...... ",strPath.c_str());

    std::string name = strPath + "/saveMap_all.pcd";
    std::string name2 = strPath + "/saveMap.pcd";
    pcl::io::savePCDFile(name, *points);
    dzlog_info("@@@@@@ saveMap() name = %s,name2 = %s",name.c_str(),name2.c_str());

    dzlog_info("@@@@@@ saveMap() before filter size is %d:",points->points.size());
    pcl::VoxelGrid<PointType> downSizeFilterTempMap;
    downSizeFilterTempMap.setLeafSize(0.1f , 0.1f, 0.1f);
    downSizeFilterTempMap.setInputCloud((*points).makeShared());
    downSizeFilterTempMap.filter(*points);
    dzlog_info("@@@@@@ saveMap() after  filter size is %d:",points->points.size());

    pcl::io::savePCDFile(name2, *points);
    dzlog_info("@@@@@@ saveMap() save map pcd file ok !!!");
    dzlog_info("try to run pcd2pgm to save pgm and yaml file ...... ");

    PointType minPt,maxPt;
    g_mapOctree.reset(new pcl::octree::OctreePointCloudSearch<PointType>(0.003));

    PointCloud::Ptr map_data (new pcl::PointCloud<PointType>);
    *map_data = *points;

    // PointCloud::Ptr map_data = points->makeShared();
    map_data = pointFilter(map_data);
    nav_msgs::OccupancyGrid tmpGrid = genOccu(map_data,minPt,maxPt);
    savePgm(&tmpGrid, strPath + "/map" );
    saveOccupancyGridMap2png(tmpGrid, strPath + "/map"  );

}

void LidarLoopClosure::setVisionLoopTime(const std::vector<std::pair<double, double>> &loop_time)
{
    loop_time_ = loop_time;
    dzlog_info("Vision loop counts is : %d .", loop_time_.size());
}

void LidarLoopClosure::setOneLoopTime(const double first, const double second)
{
    // 加锁 ，防止和后面pop的动作冲突
    std::lock_guard<std::mutex> lock(keyScanMutexM);

    if (loop_time_queue.empty())
    {
        // g_Mutex.lock();
        loop_time_queue.push(std::make_pair(first, second));
        // g_Mutex.unlock();
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

    dzlog_info("get a vision loop time pair. loop_time_queue size is: %d" , loop_time_queue.size() );
}

void LidarLoopClosure::setWorkPath(const std::string work_dir)
{
    strWorkDirM = work_dir;
    dzlog_info("@@@@@@ strWorkDirM is %s",strWorkDirM.c_str());
}

void LidarLoopClosure::setMapThreadDone()
{
    bIsLoopThreadExitM = true;
}

void LidarLoopClosure::setTranslationThreshold(const double translation_threshold)
{
    translation_threshold_ = translation_threshold;
}

bool LidarLoopClosure::Initialize(const ros::NodeHandle &n)
{
    if (!LoadParameters())
    {
        dzlog_info("@@@@@@ Initialize() Failed to load parameters.");
        return false;
    }

    if (!RegisterCallbacks(n))
    {
        dzlog_info("@@@@@@ Initialize() Failed to register callbacks.");
        return false;
    }

    //***create loopclosure thread***//
    std::thread t1(&LidarLoopClosure::loopClosureThread, this);
    t1.detach();

    std::string folder_temp = "mv " + strWorkDirM + "pose_graph " + strWorkDirM + "pose_graph_bak/";
    int iRtn = system(folder_temp.c_str());
    // folder_temp =  "mkdir -p " +  strWorkDirM  + "SC/" ;
    // iRtn = system(folder_temp.c_str());
    folder_temp =  "mkdir -p " +  strWorkDirM  + "icp/" ;
    iRtn = system(folder_temp.c_str());
    return true;
}

void LidarLoopClosure::filter_pointcloud(PointCloud &pc)
{
    // downsample clouds
    // dzlog_info("original <size: %ld > ", pc.size() );
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.setInputCloud(pc.makeShared());
    vg.filter(pc);
    // dzlog_info("filtered <size: %ld > ", pc.size() );

}

bool LidarLoopClosure::LoadParameters()
{
    // Should we turn loop closure checking on or off?
    if (!pu::Get("check_for_loop_closures", bIsCheckLoopClosureM)) return false;

    // Load ISAM2 parameters.
    unsigned int relinearize_skip = 1; // 1
    double relinearize_threshold = 0.01;
    // if (!pu::Get("relinearize_skip", relinearize_skip)) return false;
    // if (!pu::Get("relinearize_threshold", relinearize_threshold)) return false;

    // Load loop closing parameters.
    translation_threshold_ =  0.5;  // 0.75
    proximity_threshold_ = 25;     // 10
    max_tolerable_fitness_ = 0.80;  // 0.36; 不要太高< less 0.5 >，否则错误的约束加到GTSAM里面后，无法优化出结果 ,园区环境 0.5  默认的匹配参数
    skip_recent_poses_ = 10;       // 20
    maxLoopKeysYawM = 1.0; //1.05

    pu::Get("translation_threshold", translation_threshold_);
    dzlog_info("translation_threshold_ : %f", translation_threshold_ );

    pu::Get("gicp_fitness", max_tolerable_fitness_);
    dzlog_info("max_tolerable_fitness_ : %f", max_tolerable_fitness_ );

    pu::Get("detect_time_regional", detect_time_regional_);
    dzlog_info("detect_time_regional_ : %f", detect_time_regional_ );

    pu::Get("detect_step", detect_step_);
    dzlog_info("detect_step_ : %d", detect_step_ );

    pu::Get("max_detect_range", max_detect_range_);
    dzlog_info("max_detect_range_ : %f", max_detect_range_ );   

    // if (!pu::Get("translation_threshold", translation_threshold_)) return false;
    // if (!pu::Get("proximity_threshold", proximity_threshold_)) return false;
    // if (!pu::Get("max_tolerable_fitness", max_tolerable_fitness_)) return false;
    // if (!pu::Get("skip_recent_poses", skip_recent_poses_)) return false;
    // if (!pu::Get("max_loopkeys_yaw", maxLoopKeysYawM)) return false;
    // dzlog_info("@@@@@@ LoadParameters() maxLoopKeysYawM = %f",maxLoopKeysYawM);

    // Load ICP parameters.
    icp_tf_epsilon_ = 0.0000000001;
    icp_corr_dist_ = 0.25;
    icp_iterations_ = 30;
    
    pu::Get("gicp_tf_epsilon", icp_tf_epsilon_);
    dzlog_info("icp_tf_epsilon_ : %f", icp_tf_epsilon_ );

    pu::Get("gicp_corr_dist", icp_corr_dist_);
    dzlog_info("icp_corr_dist_ : %f", icp_corr_dist_ );

    pu::Get("gicp_iterations", icp_iterations_);
    dzlog_info("icp_iterations_ : %d", icp_iterations_ );

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
    Diagonal::shared_ptr covariance(Diagonal::Sigmas(noise));

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

    return true;
}

bool LidarLoopClosure::RegisterCallbacks(const ros::NodeHandle &n)
{
    ros::NodeHandle nl(n);
    // message_filters::Subscriber<geometry_msgs::PoseStamped> sub_lidar_pose(nl, "/lidar_pose", 10);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> sub_surf_points(nl, "/livox_surf_point", 10);

    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    // typedef sync_policies::ExactTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> MySyncPolicy;
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_body_pose, sub_surf_points);

    sub_lidar_pose = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nl, "/map_current_pose", 10000);
    sub_surf_points  = new  message_filters::Subscriber<sensor_msgs::PointCloud2>(nl, "/livox_surf_point_at_imu", 10000);
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *sub_lidar_pose, *sub_surf_points);
    sync->registerCallback(boost::bind(&LidarLoopClosure::add_pose_and_scan_to_gtsam_callback,this, _1, _2));

    server   = nl.advertiseService("/get_loop_closure", &LidarLoopClosure::startLoopDetect,this);
    sub_saveMap   = nl.subscribe("/save_map", 1, &LidarLoopClosure::saveMapCallBack,this);
    pubLoopFindM   = nl.advertise<std_msgs::Empty>("/map_loop_closure", 100);
    pubSaveMapStsM = nl.advertise<std_msgs::Int8>("/map_save_status", 100);

    return true;
}

void LidarLoopClosure::add_pose_and_scan_to_gtsam_callback(const geometry_msgs::PoseStampedConstPtr &pose,
                                                           const PointCloud2ConstPtr &scan)
{
    geometry_utils::Vector3Base<double> posi_i(pose->pose.position.x,
                                               pose->pose.position.y,
                                               pose->pose.position.z);
    geometry_utils::Rotation3Base<double> ori_i(geometry_utils::QuaternionBase<double>(pose->pose.orientation.w,
                                                                                       pose->pose.orientation.x,
                                                                                       pose->pose.orientation.y,
                                                                                       pose->pose.orientation.z));
    geometry_utils::Transform3 pose_i(posi_i, ori_i);

    //输出的 pcl 类型
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*scan, *cloud);
    pcl_conversions::toPCL(scan->header.stamp , cloud->header.stamp);
    cloud->header.frame_id = cloud->header.frame_id;

    static float range = 1.0 ;
    static pcl::CropBox<PointType> cropBoxFilter (true);
    cropBoxFilter.setInputCloud (cloud);
    cropBoxFilter.setMin (Eigen::Vector4f  (-range, -range, -range, 1.0f));
    cropBoxFilter.setMax (Eigen::Vector4f  (range, range, range, 1.0f));
    cropBoxFilter.setNegative(true);
    cropBoxFilter.filter (*cloud);

    cropBoxFilter.setInputCloud (cloud);
    cropBoxFilter.setMin (Eigen::Vector4f  (-max_detect_range_, -max_detect_range_, -max_detect_range_, 1.0f));
    cropBoxFilter.setMax (Eigen::Vector4f  (max_detect_range_, max_detect_range_, max_detect_range_, 1.0f));
    cropBoxFilter.setNegative(false);
    cropBoxFilter.filter (*cloud);

    // PointCloud scan1_filter = *cloud;
    filter_pointcloud(*cloud);

    geometry_utils::Transform3 delta = geometry_utils::PoseDelta(lastPoseM, pose_i);
    lastPoseM = pose_i;

    if (!addPoseAndKeyScan(delta, cloud))
    {
        return;
    }

    vecLidarPoseM.push_back(pose);

    // save sth for HBA
    static std::ofstream lio_path_file("/home/map/key_pose.txt", std::ios::out);

    // lio_path_file.open("/home/map/key_pose.txt", std::ios::app);

    lio_path_file.setf(std::ios::fixed, std::ios::floatfield);
    lio_path_file.precision(3);
    lio_path_file << scan->header.stamp.toSec() << " ";
    lio_path_file.precision(5);
    lio_path_file
        << pose->pose.position.x << " "
        << pose->pose.position.y << " "
        << pose->pose.position.z << " "
        << pose->pose.orientation.x << " "
        << pose->pose.orientation.y << " "
        << pose->pose.orientation.z << " "
        << pose->pose.orientation.w << std::endl;
    // lio_path_file.close();

    // save sth for interactive slam
    std::stringstream ss;
    ss << setw(6) << setfill('0') << keyframeCntM;
    string one_path = strWorkDirM + "pose_graph/" + ss.str();
    std::string mkf =  "mkdir -p " + one_path;
    int iRtn = system(mkf.c_str());
    pcl::io::savePCDFile( one_path + "/cloud.pcd", *cloud);
    // save data file
    ofstream pose_data( one_path + "/data", ios::out);
    pose_data
        << "stamp " << int( scan->header.stamp.toSec() ) << " 0" << endl //一定要是 int 的 time
        <<  "estimate" << endl << fixed << setprecision(10)
        << pose_i.rotation(0) << " " << pose_i.rotation(1) << " " << pose_i.rotation(2) << " " << pose_i.translation(0) << endl
        << pose_i.rotation(3) << " " << pose_i.rotation(4) << " " << pose_i.rotation(5) << " " << pose_i.translation(1) << endl
        << pose_i.rotation(6) << " " << pose_i.rotation(7) << " " << pose_i.rotation(8) << " " << pose_i.translation(2) << endl
        << "0 0 0 1" << endl
        << "odom" << endl
        << pose_i.rotation(0) << " " << pose_i.rotation(1) << " " << pose_i.rotation(2) << " " << pose_i.translation(0) << endl
        << pose_i.rotation(3) << " " << pose_i.rotation(4) << " " << pose_i.rotation(5) << " " << pose_i.translation(1) << endl
        << pose_i.rotation(6) << " " << pose_i.rotation(7) << " " << pose_i.rotation(8) << " " << pose_i.translation(2) << endl
        << "0 0 0 1" << endl
        << "id " << keyframeCntM << endl;
    pose_data.close();

    ROS_WARN( "keyframeCntM %d", keyframeCntM );
    // dzlog_info("pose->header.stamp %f. scan->header.stamp %f", pose->header.stamp.toSec() , scan->header.stamp.toSec());

    keyframeCntM++;
}
bool  LidarLoopClosure::save_g2o_file(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    saveGtsam2G2oFile(strWorkDirM + "graph_debug.g2o");
    ROS_WARN( "save graph_debug.g2o done ." );
    return true;
}

bool LidarLoopClosure::startLoopDetect(loop_closure_gtsam::LoopTimePair::Request &req,
                                       loop_closure_gtsam::LoopTimePair::Response &res)
{
    // 遍历出 距离这个两个时间戳最近的位姿，如果距离过大就不处理这个时间戳对
    float x_first  = 0;
    float y_first  = 0;
    float x_second = 0;
    float y_second = 0;

    for(auto item : vecLidarPoseM )
    {
        if( std::fabs(item->header.stamp.toSec() - req.first) < 5.0 )
        {
            x_first = item->pose.position.x;
            y_first = item->pose.position.y;
        }

        if( std::fabs(item->header.stamp.toSec() - req.second) < 5.0 )
        {
            x_second = item->pose.position.x;
            y_second = item->pose.position.y;
        }
        
    }

    // 时间戳对应的位姿，足够近，再去加入回环
    // if (std::fabs( x_first - x_second ) < 15.0 && std::fabs( y_first - y_second ) < 15.0 && std::fabs( req.first - req.second )  > 30.0 )
    if (std::fabs( req.first - req.second )  > 30.0 )
    {
        setOneLoopTime(req.first, req.second);
        dzlog_info("get a vision loop time pair. time pair is %f. %f",req.first,req.second);
    }
    // dzlog_info("get one possible time loop pair. " );
    // dzlog_info("Dx , Dy position is: %f. %f ",x_first - x_second,y_first - y_second);
    return true;
}

void LidarLoopClosure::saveMapCallBack(const base_controller::CSGMapInfo &saveMapMsg)
{
    dzlog_info("@@@@@@ saveMapCallBack() regionID = %d , mapID = %d , iNavType = %d",saveMapMsg.iRegionID,saveMapMsg.iMapID, saveMapMsg.iNavType);

    std::string dataPath = "/home/map/mapinfo.yaml";
    std::fstream fs;
    fs.open(dataPath.c_str(), std::fstream::out);
    if (fs.is_open()) {
      fs << saveMapMsg.iRegionID << std::endl;
      fs << saveMapMsg.iMapID << std::endl;
      fs << saveMapMsg.iNavType << std::endl;
      fs.close();
      dzlog_info("@@@@@@ saveMapInfo() write /home/map/mapinfo.yaml OK!!!");
    } else {
      dzlog_error("###### saveMapInfo() cannot open mapinfo.yaml !!!");
    }

    saveMap(saveMapMsg.iRegionID,saveMapMsg.iMapID);

    std_msgs::Int8 msg;
    msg.data = 100;
    pubSaveMapStsM.publish(msg);
    dzlog_info("@@@@@@ pubSaveMapStsM 100% " );
    setMapThreadDone();
}

void LidarLoopClosure::GetMaximumLikelihoodPoints(PointCloud *points)
{
    if (points == NULL)
    {
        dzlog_info("@@@@@@ GetMaximumLikelihoodPoints() point container is null.");
        return;
    }
    points->points.clear();

    // 会覆盖已经有的文件
    ofstream GTSAM_pose(strWorkDirM + "GTSAM_pose.txt", ios::out);

    // 先清空文件夹
    // int iRtn = system( ("rm -r " + strWorkDirM + "pose_graph/" ).c_str() );
    // string SC_path = strWorkDirM  + "SC/" ;
    // std::string mk_sc_f =  "mkdir -p " + SC_path;
    // iRtn = system(mk_sc_f.c_str());

    // Iterate over poses in the graph, transforming their corresponding laser
    // scans into world frame and appending them to the output.
    for (const auto &keyed_pose : values_)
    {
        const unsigned int key = keyed_pose.key;
        if (key % 500 == 0)
        {
            cout  << int ((1.0 * key) / (1.0 * key_) * 100) << "%. " ;
            dzlog_info("saving map %d %", int ( (1.0 * key) / (1.0 * key_) * 100) );
            std_msgs::Int8 msg;
            msg.data = int ( (1.0 * key) / (1.0 * key_) * 100 - 5 ) ;
            pubSaveMapStsM.publish(msg);
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
/*
        // save sth for interactive slam
        std::stringstream ss;
        ss << setw(6) << setfill('0') << key;
        string one_path = strWorkDirM + "pose_graph/" + ss.str();
        std::string mkf =  "mkdir -p " + one_path;
        int iRtn = system(mkf.c_str());

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
*/
        // 跳过第一个，因为第一个是0
        // ROS_WARN("key is %u  ", key);

        if (key < 1)
            continue;

        // 往上面的文件里写
        GTSAM_pose.setf(ios::fixed, ios::floatfield);
        GTSAM_pose.precision(4);
        //  GTSAM_pose << key  << " ";

        const ros::Time pose_stamp = pcl_conversions::fromPCL( keyed_scans_[key]->header.stamp );
        // dzlog_info("pose_stamp %f ",   pose_stamp.toSec() );

        GTSAM_pose << pose_stamp.toSec() << " ";
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
    // 把上面的文件关闭
    GTSAM_pose.close();

    values_ = isam_->calculateEstimate();
    saveGtsam2G2oFile(strWorkDirM + "pose_graph/graph.g2o");
}

gu::Transform3 LidarLoopClosure::GetLastPose() const
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

gu::Transform3 LidarLoopClosure::ToGu(const Pose3 &pose) const
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

Pose3 LidarLoopClosure::ToGtsam(const gu::Transform3 &pose) const
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

Mat66 LidarLoopClosure::ToGu(
    const Gaussian::shared_ptr &covariance) const
{
    gtsam::Matrix66 gtsam_covariance = covariance->covariance();

    Mat66 out;
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            out(i, j) = gtsam_covariance(i, j);

    return out;
}

Gaussian::shared_ptr LidarLoopClosure::ToGtsam(
    const Mat66 &covariance) const
{
    gtsam::Matrix66 gtsam_covariance;

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            gtsam_covariance(i, j) = covariance(i, j);

    return Gaussian::Covariance(gtsam_covariance);
}

PriorFactor<Pose3> LidarLoopClosure::MakePriorFactor(
    const Pose3 &pose,
    const Diagonal::shared_ptr &covariance)
{
    return PriorFactor<Pose3>(key_, pose, covariance);
}

BetweenFactor<Pose3> LidarLoopClosure::MakeBetweenFactor(
    const Pose3 &delta,
    const Gaussian::shared_ptr &covariance)
{
    //odometry_edges_.push_back(std::make_pair(key_ - 1, key_));
    return BetweenFactor<Pose3>(key_ - 1, key_, delta, covariance);
}

bool LidarLoopClosure::addPoseAndKeyScan(const geometry_utils::Transform3 &delta, const PointCloud::ConstPtr &scan)
{
    unsigned int pose_key;
    gu::MatrixNxNBase<double, 6> covariance;
    covariance.Zeros();
    for (int i = 0; i < 3; ++i)
        covariance(i, i) = 0.01;
    for (int i = 3; i < 6; ++i)
        covariance(i, i) = 0.004;

    isamMutexM.lock();

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

unsigned int LidarLoopClosure::getLastScanKey()
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    if (!keyed_scans_.empty())
    {
        return (unsigned int)keyed_scans_.rbegin()->first;
    }
    return 0;
}



bool LidarLoopClosure::isKeyFrame(unsigned int iKey)
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    if (!keyed_scans_.count(iKey))
    {
        return false;
    }
    return true;
}

void LidarLoopClosure::getKeyedScan(unsigned int iKey, PointCloud::Ptr scan)
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    const PointCloud::ConstPtr tmpScan = keyed_scans_[iKey];
    pcl::copyPointCloud(*tmpScan, *scan);
}

const geometry_utils::Transform3 LidarLoopClosure::getDiffPose(unsigned int iKey1, unsigned int iKey2)
{
    isamMutexM.lock();
    const gu::Transform3 pose1 = ToGu(values_.at<Pose3>(iKey1));
    const gu::Transform3 pose2 = ToGu(values_.at<Pose3>(iKey2));
    isamMutexM.unlock();

    const gu::Transform3 diffPose = gu::PoseDelta(pose1, pose2);

    return diffPose;
}

void LidarLoopClosure::updatePoseFactor()
{
    // 查看论文，寻找ICP求协方差矩阵的算法
    Mat66 covariance;
    covariance.Zeros();
    for (int i = 0; i < 2; ++i)
        covariance(i, i) = 0.01;
    for (int i = 2; i < 5; ++i)
        covariance(i, i) = 0.001;
    for (int i = 5; i < 6; ++i)
        covariance(i, i) = 0.04;

    isamMutexM.lock();
    
    NonlinearFactorGraph new_factor;
    new_factor.add(BetweenFactor<Pose3>(iCurHdlKeyM, iLoopKeyM, ToGtsam(loopDeltaM), ToGtsam(covariance)));
    isam_->update(new_factor, Values());
    values_ = isam_->calculateEstimate(); //原来是有无回环都调用一次，但是感觉无回环没必要调用
    last_closure_key_ = iCurHdlKeyM;
    
    isamMutexM.unlock();
    
    // loop_edges_.push_back(std::make_pair(iCurHdlKeyM, iLoopKeyM));
    dzlog_info("@@@@@@ updatePoseFactor() update loop pose between key %u and %u", iCurHdlKeyM, iLoopKeyM);
}

bool LidarLoopClosure::getIsLoopThreadExit()
{
    return bIsLoopThreadExitM;
}

void LidarLoopClosure::saveGtsam2G2oFile(string outputFile)
{
    isamMutexM.lock();
    gtsam::Values value = isam_->calculateEstimate();
    NonlinearFactorGraph factorGraph = isam_->getFactorsUnsafe();
    gtsam::writeG2o(factorGraph, value, outputFile);
    isamMutexM.unlock();
}

bool LidarLoopClosure::PerformICP(const PointCloud::ConstPtr &scan1,
                                  const PointCloud::ConstPtr &scan2,
                                  const gu::Transform3 &pose1,
                                  const gu::Transform3 &pose2,
                                  gu::Transform3 *delta,
                                  Mat66 *covariance,
                                  double *fitnessReturn)
{
    if (delta == NULL || covariance == NULL)
    {
        dzlog_info("@@@@@@ PerformICP() Output pointers are null.");
        return false;
    }

    // ICP：此处考虑了闭合时，机器人可能不是一个朝向，此时直接ICP可能匹配不上，这时候
    //利用pose1/2时的朝向的估计信息。转换后再匹配。
    static pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
    icp.setTransformationEpsilon(icp_tf_epsilon_);
    icp.setMaxCorrespondenceDistance(icp_corr_dist_);
    icp.setMaximumIterations(icp_iterations_);
    icp.setRANSACIterations(50);
    
    /*dzlog_info("GICP param: icp.getTransformationEpsilon(): %f , icp.getMaxCorrespondenceDistance(): %f , icp.getMaximumIterations(): %d " 
            // , icp.getEuclideanFitnessEpsilon(): %f , icp.getRANSACOutlierRejectionThreshold(): %f . ", 
               , icp.getTransformationEpsilon(), icp.getMaxCorrespondenceDistance() , icp.getMaximumIterations()
            // , icp.getEuclideanFitnessEpsilon() , icp.getRANSACOutlierRejectionThreshold()
               );
    */

    icp.setInputSource(scan1);  //source
    icp.setInputTarget(scan2);

    // Perform ICP.
    PointCloud unused_result;
    icp.align(unused_result); //将2个关键帧配准
    unused_result.height = 1;
    unused_result.width = unused_result.size();

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

    for (int i = 2; i < 5; ++i)
        (*covariance)(i, i) = 0.001;

    for (int i = 5; i < 6; ++i)
        (*covariance)(i, i) = 0.04;

    // icp的结果
    static int cnts = 1;
    // dzlog_info("@@@@@@ %d th score %f", cnts, icp.getFitnessScore());

    // cout << "icp rotation (deg) :" << endl
    //      << delta->rotation.GetEulerZYX() * 180 / M_PI << endl;

    std::string name;
    name = strWorkDirM + "icp/_" + std::to_string(cnts) + "_scan2.pcd";
    pcl::io::savePCDFile(name, *scan2);

    name = strWorkDirM + "icp/_" + std::to_string(cnts) + "_scan1.pcd";
    pcl::io::savePCDFile(name, *scan1);

    name = strWorkDirM + "icp/_" + std::to_string(cnts) + "_unused_result.pcd";
    pcl::io::savePCDFile(name, unused_result);

    dzlog_info("%dth score %f , file is %s ", cnts, icp.getFitnessScore(), name.c_str());
    cnts++;

    return true;
}

bool LidarLoopClosure::FindLoopClosures(unsigned int key_temp, const double &target_time)
{
    dzlog_info("try to FindLoopClosures. ");
    int key = key_temp;

    // If loop closure checking is off, don't do this step. This will save some
    // computation time.

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
    Mat66 covariance;
    int keyResut = 0;
    int other_key;
    int cnts_use_scan = 0;

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
        PointCloud::Ptr scan2(new PointCloud);
        try
        {
            getKeyedScan(other_key, scan2);
        }
        catch (const std::exception &e)
        {
            std::cerr << "get points fails: " << other_key << " error code:" << e.what() << '\n';
            continue;
        }

        // 时间戳是不是在视觉回环的附近
        // 2 就可以，在远的匹配得分就很高了
    
        const ros::Time scan2_stamp = pcl_conversions::fromPCL(scan2->header.stamp);
        // dzlog_info("stamp %f  %f",   scan2_stamp.toSec() , target_time);
        if (std::fabs(scan2_stamp.toSec() - target_time) >= detect_time_regional_ )
        {
            continue;
        }
        
        if( cnts_use_scan++ % detect_step_ != 0 )
        {
            continue;
        }

        dzlog_info("try DO GICP , with other_key is %u <size: %ld >, key is %u <size: %ld > ", other_key, scan2->size(), key,  scan1->size() );
        // 从头到尾 依次检测，有可能出现：
        // 中间的回环将尾的位置 纠正到 离头的 位置很远，这样 位置差就不满足以下的关系了。
        // 由于已经从视觉回环上确定了大概的时间，此时对应的位置也基本确定，将其注释掉，问题不大。
        // 在测试 20221011
        // if (  (difference.translation.Norm() < proximity_threshold_) )
            //  && (fabs(difference.rotation.Yaw()) < maxLoopKeysYawM)
        // if( fabs(difference.rotation.Yaw()) < maxLoopKeysYawM )
        if(1)
        {
            gu::Transform3 deltaTemp;
            Mat66 covarianceTemp;
            double fitnessReturn;
        
            if (PerformICP(scan1, scan2, pose1, pose2, &deltaTemp, &covarianceTemp, &fitnessReturn))
            {
                dzlog_info("do icp ok . fitnessReturn is %f .", fitnessReturn);
                static int icp_ok_cnts = 1;
                auto rotation_temp = deltaTemp.rotation.GetEulerZYX() * 180 / M_PI;
                dzlog_info(" delta is: rotation: %f, %f, %f (deg).", rotation_temp(0), rotation_temp(1), rotation_temp(2));

                // 找到一个闭环位姿
                if (fitnessReturn < fitnessMin && std::fabs(rotation_temp(0) ) <  10.0 && std::fabs(rotation_temp(1) ) <  10.0   )
                // if (fitnessReturn < fitnessMin )
                {
                    fitnessMin = fitnessReturn;
                    keyResut = other_key;
                    delta = deltaTemp;
                    covariance = covarianceTemp;
                    closed_loop = true;
                    dzlog_info(" %uth icp ok, loop closure between other_key: %u and key: %u , fitnessMin %f diff_dist %f , diff_yaw %f , DIFF key = %u .  ", icp_ok_cnts++, other_key, key, fitnessMin, difference.translation.Norm(), difference.rotation.Yaw() , key - other_key);
                }
            }
        }
    }

    if (closed_loop)
    {
        closed_loop = false;
        dzlog_info(" delta is: translation: %f, %f, %f .", delta.translation(0), delta.translation(1), delta.translation(2));
        auto rota_temp = delta.rotation.GetEulerZYX() * 180 / M_PI;
        dzlog_info(" delta is: rotation: %f, %f, %f (deg).", rota_temp(0), rota_temp(1), rota_temp(2));
        // printf(" %dth closed_loop. delta is: rotation: %f, %f, %f (deg).", loop_cnts_, rota_temp(0), rota_temp(1), rota_temp(2));
        cout << loop_cnts_ <<  "th closed_loop ok .icp rotation (deg) :" << endl  << rota_temp << endl;
        cout << loop_cnts_ <<  "th closed_loop ok .icp translation (m) :" << endl  << delta.translation << endl;

        std::lock_guard<std::mutex> lock(isamMutexM);
        // g_Mutex.lock();
        NonlinearFactorGraph new_factor;
        new_factor.add(BetweenFactor<Pose3>(key, keyResut, ToGtsam(delta), ToGtsam(covariance)));
        isam_->update(new_factor, Values());
        values_ = isam_->calculateEstimate();

        // g_Mutex.unlock();
        
        loop_cnts_++;
        dzlog_info("@@@@@@ %uth isHasLoopClosure() find a loop between key %u and %u,fitnessMin=%f, DIFF key = %u,last_closure_key_ = %u",
                   loop_cnts_, key, keyResut, fitnessMin, key - keyResut, last_closure_key_);
        last_closure_key_ = key;
        //找到回环
        pubLoopFindM.publish(std_msgs::Empty());
    }
    return true;
}

bool LidarLoopClosure::AddBetweenFactor(const gu::Transform3 &delta,
                                        const Mat66 &covariance,
                                        const ros::Time &stamp,
                                        unsigned int *key)
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    if (key == NULL)
    {
        dzlog_info("@@@@@@ PerformICP() Output key is null.");
        return false;
    }

    // Append the new odometry.
    Pose3 new_odometry = ToGtsam(delta);

    // 轨迹累加
    odometry_ = odometry_.compose(new_odometry);

    // 两帧很近
    if(key_ != 1)
    {
        float shui_range_sqrt = odometry_.translation().x() * odometry_.translation().x() + 
                                odometry_.translation().y() * odometry_.translation().y() + 
                                odometry_.translation().z() * odometry_.translation().z()  ;
        if ( (shui_range_sqrt < translation_threshold_ * translation_threshold_) &&
        // if ((odometry_.translation().norm() < translation_threshold_) &&
             (fabs(odometry_.rotation().yaw()) < 0.15)  )
        {
            return false;
        }
    }

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

    // Assign output and get ready to go again!
    *key = key_++;

    odometry_ = Pose3::identity();
    return true;
}

bool LidarLoopClosure::AddKeyScanPair(unsigned int key, const PointCloud::ConstPtr &scan)
{
    std::lock_guard<std::mutex> lock(keyScanMutexM);
    if (keyed_scans_.count(key))
    {
        dzlog_info("@@@@@@ AddKeyScanPair() Key %u already has a laser scan.", key);
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

void LidarLoopClosure::loopClosureThread()
{
    dzlog_info("@@@@@@ loopClosureThread() loop thread IN !!!");
    if (!bIsCheckLoopClosureM)
    {
        dzlog_info("@@@@@@ loopClosureThread() Do Not Need loop thread !!!");
        return;
    }

    // 循环判断视觉有无回环进来
    while (1)
    {
        std_msgs::String str_temp;
        str_temp.data = "loop cnts : " + std::to_string( loop_cnts_ )   ;
        pubLoopCnt.publish(str_temp);

        // TODO 如何退出这个循环
        // 如何判断 建图程序已经结束，视觉不再有回环了

        // 有视觉的回环还有就进行 GICP 
        if (loop_time_queue.empty())
        {
            dzlog_info("Do not get vision loopclosure .loopClosureThread sleep 30s . key frame %d ......", getLastScanKey() );          
            usleep(30000 * 1000);
            if (bIsLoopThreadExitM)
            {
                break;
            }
            continue;
        }

        // 把最前面的回环时间拿出来
        double loop_time_first = loop_time_queue.front().first;
        double loop_time_second = loop_time_queue.front().second;

        double last_key_frame_stamp = vecLidarPoseM.back()->header.stamp.toSec();

        // 里程计延迟的话，就等待
        if ( loop_time_first - 3*detect_time_regional_ >  last_key_frame_stamp )
        {
            dzlog_info(" vision time <%f> gt key scan time <%f>. waiting odom ......", loop_time_first , last_key_frame_stamp  );

            dzlog_info(" if odom is stoped and loop_time_queue size is 0 .you can save map by running save_map.sh ."  );
            dzlog_info(" waiting 10s"  );
            usleep(10000 * 1000);
            dzlog_info(" waiting 10s DONE"  );

            // if ( last_key_frame_stamp == vecLidarPoseM.back()->header.stamp.toSec() )
            // {
            //     dzlog_info(" 30s odom is stop . pop loop_time_queue. "  );
            //     g_Mutex.lock();
            //     loop_time_queue.pop();
            //     g_Mutex.unlock();
            // }

            // int last_total_key_frame = 0;
            // if ( last_total_key_frame == getLastScanKey() )
            // {
            //     auto  sleep_time = (loop_time_first - last_key_frame_stamp) > 30  ?  (loop_time_first - last_key_frame_stamp) : 30 ;
            //     dzlog_info("odometry is not get realtime . wait %lf s ...... %f . %f " , sleep_time , loop_time_first , last_key_frame_stamp  );
            //     usleep( int(sleep_time) * 1000);
            //     dzlog_info(" wait done ");
            //     if ( last_total_key_frame == getLastScanKey() )
            //     {
            //         dzlog_info("!!!!!! odom seem stop ......"  );
            //         while ( loop_time_queue.front().first > vecLidarPoseM.back()->header.stamp.toSec()  )
            //         {
            //             if ( !loop_time_queue.empty() )
            //             {
            //                 dzlog_info("remove early vision loop time pair ."  );
            //                 g_Mutex.lock();
            //                 loop_time_queue.pop();
            //                 g_Mutex.unlock();
            //             }
            //         }
            //     }
            // }
            continue;
        }
        dzlog_info("now loop_time_queue size is %ld .", loop_time_queue.size() );

        // 加锁 ，防止和前面push的动作冲突
        // g_Mutex.lock();
        // if ( !loop_time_queue.empty() ) 
        // {
        //     loop_time_queue.pop();
        // }
        if ( !loop_time_queue.empty() ) 
        {
            loop_time_queue.pop();
        }
        // g_Mutex.unlock();

        // 每次进来，还从 第一个 开始
        unsigned int iKey = 0;
        // iKey = last_closure_key_;

        while (1)
        {
            if (bIsLoopThreadExitM)
            {
                break;
            }

            // 跳帧 加快计算
            iKey = iKey + detect_step_;

            // 现在的点云id 不能太靠近最后几个
            if (iKey > key_ - skip_recent_poses_)
                break;

            if ( std::fabs( iKey - last_closure_key_ ) < skip_recent_poses_)
                continue;

            PointCloud::Ptr scan(new PointCloud);
            try
            {
                getKeyedScan(iKey, scan);
            }
            catch (const std::exception &e)
            {
                std::cerr << "get points fails: " << iKey << " error code:" << e.what() << '\n';
                break;
            }

            // 三种情况，1、比 loop_time_first 早的点云
            // 三种情况，2、在 loop_time_first 附近的点云 取这个情况
            // 三种情况，3、比 loop_time_first 晚的点云
            // 认为视觉回环的 detect_time_regional_ s内的点云 ， 也应该可以检测到回环,这样来缩小的搜索的范围
            // dzlog_info("time diff %f. scan->header.stamp %f", std::fabs( scan->header.stamp - loop_time_first ), scan->header.stamp);
            const ros::Time scan_stamp = pcl_conversions::fromPCL(scan->header.stamp);
            // dzlog_info("stamp %f  %f",   scan_stamp.toSec() , loop_time_first);
            if ( std::fabs( scan_stamp.toSec() - loop_time_first ) <= detect_time_regional_)
            {
                FindLoopClosures(iKey, loop_time_second);
            }
        }
    }
    dzlog_info("@@@@@@ loopClosureThread() loop thread EXIT !!!");
}

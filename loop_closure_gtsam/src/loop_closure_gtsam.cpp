#include <zlog.h>
#include "loop_closure_gtsam/LaserLoopClosure_time.h"

int initZlog()
{
    if (-1 == access("/home/roslog", F_OK))
    {
        mkdir("/home/roslog", 0777);
    }
    if (dzlog_init("/home/config/zlog.conf", "loop_cat") != 0)
    {
        printf("loop_cat init zlog failed\n");
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    //初始化zlog
    if (0 == initZlog())
    {
        dzlog_info("@@@@@@ loopClosure init zlog success !!!");
    }

    ros::init(argc, argv, "loop_closure_gtsam");
    ros::NodeHandle nh;

    // 回环检测的对象
    LidarLoopClosure llc;
    if (!llc.Initialize(nh))
    {
        dzlog_info("@@@@@@ loopClosure Initialize Failed !!!");
        return EXIT_FAILURE;
    }

    ros::spin();

    return 0;
}

/*
 * @Description: 前端里程计的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <memory>
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/saveMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/front_end/front_end_flow.hpp"

using namespace lidar_localization;

// global variable
bool save_map = false;

// 查看srv文件夹，里面有saveMap的request和response 定义
// 参考　胡春旭　<ros 开发实践>　3.7 服务中的Server和Client
bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
    save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    // register service save_map: 增加这个sevice，需要在terminal中 source 一下才能用。
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

    // register front end processing workflow:
    // 这个就是整个前端的进程入口
    std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    // process rate: 10Hz
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        // 只要node在运行，前端就在run，如果接收到调用savemap的信息就save并且发布map
        // front end workflow就是这三个pub的函数，其他都是private函数
        front_end_flow_ptr->Run();

        if (save_map) {
            front_end_flow_ptr->SaveMap();
            front_end_flow_ptr->PublishGlobalMap();

            save_map = false;
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}
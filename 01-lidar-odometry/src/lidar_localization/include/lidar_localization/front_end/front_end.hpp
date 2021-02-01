/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_

#include <deque>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/registration/icp_registration.hpp"
#include "lidar_localization/models/registration/ndt_registration.hpp"
// TODO: include your custom registration method interface here

namespace lidar_localization {
class FrontEnd {
  public:
    // 先定义一个关键帧结构体，其实就是 点云 加 位姿矩阵
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();

    //   FrontEnd::FrontEnd 构造函数里面用的．
    bool InitWithConfig();

    // 重要
    // 　FrontEndFlow::UpdateLaserOdometry 中
    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    
    // 　FrontEndFlow::UpdateLaserOdometry 中，用gnss odo 的初始位置来初始化lidar odo位置
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

    //   FrontEndFlow::SaveMap
    bool SaveMap();
    //   FrontEndFlow::PublishData 调用了filter 类，将local_map_ptr_点云取出来并进行滤波
    bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
    //   FrontEndFlow::PublishGlobalMap()
    bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
    //   FrontEndFlow::PublishData 调用了filter 类　，将当前帧点云取出来并进行滤波
    bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

  private:
    // 预定义参数，没有调用它
    bool InitParam(const YAML::Node& config_node);
    
    // 以下三个在 FrontEnd::InitWithConfig()　的调用的函数．
    bool InitDataPath(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    
    // FrontEnd::Update调用
    bool UpdateWithNewFrame(const Frame& new_key_frame);

  private:
    std::string data_path_ = "";
    //　申明，在实现中会用到这几个指向　CloudFilterInterface的指针
    // CloudFilterInterface是一个抽象类，VoxelFilter来重载这个抽象类
    // 以后还是有其它的滤波方法加入，保留这个接口．
    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> display_filter_ptr_;

    // 每两帧的点云配准通过这个指帧对应的函数完成
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    // Frame 关键帧结构体，其实就是在点云基础上加了个位姿矩阵
    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;

    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR result_cloud_ptr_;
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;
};
}

#endif
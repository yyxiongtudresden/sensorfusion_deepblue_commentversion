/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */
#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_

#include <memory>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>


// 接收和发布消息的 实现
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

// 这个是主要的前端实现的hpp文件
#include "lidar_localization/front_end/front_end.hpp"

namespace lidar_localization {
class FrontEndFlow {
  public:
    FrontEndFlow(ros::NodeHandle& nh);

    // 正常情况下，run
    bool Run();
    // 调用service情况下执行这两个
    bool SaveMap();
    bool PublishGlobalMap();

  private:
  　//  FrontEndFlow 构造函数中使用
    bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);
　　
　　// Run 中使用到的
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool UpdateGNSSOdometry();
    bool UpdateLaserOdometry();
    bool PublishData();

  　// 不知道在哪里
    bool anime();

  private:
    // subcriber
    // 在FrontEndFlow::InitSubscribers　中初始化
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    

    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;

    // 执行frontend的
    std::shared_ptr<FrontEnd> front_end_ptr_;

    // 存信息的deque容器
    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    GNSSData current_gnss_data_;

    // 在cloud_data.hpp中可以通过递推知
    // CLOUD_PTR　= pcl::PointCloud< pcl::PointXYZ>::Ptr
    // pcl 官方: 
    // using pcl::PointCloud< PointT >::Ptr = shared_ptr<PointCloud<PointT>>
    // Returns: shared pointer to the copy of the cloud 
    
    // 指向点云的指针
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    // 4*4的单位矩阵
    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
};
}

#endif
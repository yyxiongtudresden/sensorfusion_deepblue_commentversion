/*
 * @Description: 点云匹配模块的基类
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:25:11
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class RegistrationInterface {
  public:
    virtual ~RegistrationInterface() = default;
    // 我们定义了一个基类RegistrationInterface，它执行匹配的函数是ScanMatch()，
    // NDTRegistration和ICPRegistration都是RegistrationInterface的子类，
    // 定义registration_ptr作为类对象的指针，
    // 那么registration_ptr->ScanMatch()执行的到底是ndt匹配还是icp匹配，
    // 取决于定义指针时用哪个子类做的实例化 --> front_end.cpp 中的registration_ptr
    
    
    //void pcl::Registration< PointSource, PointTarget, Scalar >::setInputTarget 	( 	const PointCloudTargetConstPtr &  	cloud	) 	
    // Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to) 
    virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
    
    
    virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
};
} 

#endif
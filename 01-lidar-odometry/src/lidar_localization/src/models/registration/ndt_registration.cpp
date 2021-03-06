/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 * 匹配类NDTRegistration内部主要函数为SetInputTarget和ScanMatch
 * 作用分别是输入目标点云和执行点云匹配，并输出匹配后位姿。
 */
#include "lidar_localization/models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {

NDTRegistration::NDTRegistration(const YAML::Node& node)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
    
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    LOG(INFO) << "NDT params:" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    // Provide a pointer to the input target
    // void pcl::Registration< PointSource, PointTarget, Scalar >::setInputTarget 	( 	const PointCloudTargetConstPtr &  	cloud	) 	
    ndt_ptr_->setInputTarget(input_target);

    return true;
}


// 
bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    // Provide a pointer to the input source (e.g., the point cloud that we want to align to the target) 
    // void pcl::Registration< PointSource, PointTarget, Scalar >::setInputSource 	( 	const PointCloudSourceConstPtr &  	cloud	) 	
    ndt_ptr_->setInputSource(input_source);

    // 这里的ndt_ptr包含了两个点云，一个是currentscan另一个是localmap
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    // Get the final transformation matrix estimated by the registration method. 
    result_pose = ndt_ptr_->getFinalTransformation();

    return true;
}
}
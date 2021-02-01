/*
 * @Description: voxel filter 模块实现
 * 滤波类VoxelFilter内部主要函数就是Filter，这个函数参数同时包含输入和输出。
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:53:20
 */
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

VoxelFilter::VoxelFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();
    
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    LOG(INFO) << "Voxel Filter params:" << std::endl
              << leaf_size_x << ", "
              << leaf_size_y << ", "
              << leaf_size_z 
              << std::endl << std::endl;

    return true;
}

bool VoxelFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    // 继承了基类pcl::PCLBase< PointT > Class
    // Provide a pointer to the input dataset.
    // 传入PCLBase 的　protected attributes  input_ (The input point cloud dataset. )
    // PCLPointCloud2ConstPtr pcl::PCLBase< pcl::PCLPointCloud2 >::input_
    voxel_filter_.setInputCloud(input_cloud_ptr);
    
    //  pcl::VoxelGrid 继承(inherited)了　基类　pcl::Filter　这个filter　是在　基类中定义的
    // void pcl::Filter< PointT >::filter 	( 	PointCloud &  	output	)
    // Calls the filtering method and returns the filtered dataset in output. 
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}
} 
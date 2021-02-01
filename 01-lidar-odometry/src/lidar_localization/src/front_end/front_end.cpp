/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CLOUD()),
     global_map_ptr_(new CloudData::CLOUD()),
     result_cloud_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig();
}

bool FrontEnd::InitWithConfig() {
    // front end node 的yaml是在 dataset这个文件夹下的，主要是定义了subscriber 的　ros topic 的名称
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    // 创造文件夹　slam_data以及里面的子文件夹用来存储数据
    InitDataPath(config_node);
    // 根据config-node中的信息确定使用那种前端配准方式，通过定义指向不同方法的指针registration_ptr实现
    InitRegistration(registration_ptr_, config_node);

    // 由于小地图滤波和当前帧滤波采用的格子大小不一样，所以类内为这两个功能各定义了一个滤波器。
    // 读取　config．yaml中的　local_map_filter对应的参数　,建立local_map_filter_ptr_ 为　local_map服务
    // 在VoxelFilter::VoxelFilter　构造函数中会用到config.yaml 的不同的leaf_size　参数
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    //  display_filter
    InitFilter("frame", frame_filter_ptr_, config_node);
    // frame_filter
    InitFilter("display", display_filter_ptr_, config_node);

    return true;
}

// 未被调用，key_frame_distance_和local_frame_num_在hpp中被定义，
bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

// 创造文件夹　slam_data以及里面的子文件夹用来存储数据
bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
    data_path_ = config_node["data_path"].as<std::string>();
    if (data_path_ == "./") {
        data_path_ = WORK_SPACE_PATH;
    }
    data_path_ += "/slam_data";

    if (boost::filesystem::is_directory(data_path_)) {
        boost::filesystem::remove_all(data_path_);
    }

    boost::filesystem::create_directory(data_path_);
    if (!boost::filesystem::is_directory(data_path_)) {
        LOG(WARNING) << "Cannot create directory " << data_path_ << "!";
        return false;
    } else {
        LOG(INFO) << "Point Cloud Map Output Path: " << data_path_;
    }

    std::string key_frame_path = data_path_ + "/key_frames";
    boost::filesystem::create_directory(data_path_ + "/key_frames");
    if (!boost::filesystem::is_directory(key_frame_path)) {
        LOG(WARNING) << "Cannot create directory " << key_frame_path << "!";
        return false;
    } else {
        LOG(INFO) << "Key Frames Output Path: " << key_frame_path << std::endl << std::endl;
    }

    return true;
}

// 根据config-node中的信息确定使用那种前端配准方式，通过定义指向不同方法的指针registration_ptr实现
bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    LOG(INFO) << "Point Cloud Registration Method: " << registration_method;

    if (registration_method == "NDT") {
        // 使用ndt匹配
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else if (registration_method == "ICP") {
        // // 使用icp匹配
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);
    }
    /*
    TODO: register your custom implementation here
    else if (registration_method == "YOUR_CUSTOM_REGISTRATION_METHOD") {
        registration_ptr = nullptr;
    }
     */
    else {
        LOG(ERROR) << "Point cloud registration method " << registration_method << " NOT FOUND!";
        return false;
    }

    return true;
}

// 展示了如何定义一个滤波器　真正执行滤波是在VoxelFilter　中的Filter函数中调用pcl完成的
// 定义不同的filter_ptr　为不同的部分执行滤波
bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << "Point Cloud Filter Method: " << filter_mothod;

    // 点云滤波是直接采用了pcl中的voxel_filter
    // 它的基本原理就是把三维空间划分成等尺寸的立方体格子，在一个立方体格子内最多只留一个点，这样就起到稀疏作用。
    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        LOG(ERROR) << "Point cloud filter method " << filter_mothod << " NOT FOUND!";
        return false;
    }

    return true;
}

bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    // 要加一个位姿矩阵，所以要从新对行的frame结构体赋值一下
    current_frame_.cloud_data.time = cloud_data.time;
    
    //　把cloud_data去除NaN值以后赋值到current_frame_中
    // pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices)
    // Removes points with x, y, or z equal to NaN, 函数有三个参数，分别为输入点云，输出点云及对应保留的索引。
    // current_frame_.cloud_data.cloud_ptr 输出点云的指针
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    //　新建一个指向滤波后点云的指针
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());

    // pcl中的voxel_filter，它的基本原理就是把三维空间划分成等尺寸的立方体格子，在一个立方体格子内最多只留一个点，这样就起到稀疏作用
    // VoxelFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) 
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    // 新建几个静态pose，init_pose_　也是单位矩阵
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    // 用指定方法ndt 或者　icp 来进行点云匹配，返回值是current_frame_.pose
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;　// current 这一帧减去原来那一帧的结果，current和前面一帧的距离．
    // 猜测两帧之间的速度不变，给一个相对有效的预测值
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    // 目前设置的是每隔key_frame_distance_ = 2米取一个关键帧，为了方便，直接采用曼哈顿距离。
    // delta x + delta y + delta z > 2.0
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        
        // 这里的传入参数为点云配准以后的结果　current_frame_.pose
        // 在函数里面它被叫做　　new_key_frame
        UpdateWithNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}

// 　FrontEndFlow::UpdateLaserOdometry 中，用gnss odo 的初始位置来初始化lidar odo位置
bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

// 只有这一帧已经被判定为是关键帧以后才会调用这个函数，修改局部地图和全局地图．
// 储存关键帧到硬盘　　更新局部地图　　更新指向局部地图的指针　　增加全局地图
bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    // 把关键帧点云存储到硬盘里，节省内存
    std::string file_path = data_path_ + "/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);
    // frame  关键帧结构体，其实就是在点云基础上加了个位姿矩阵
    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // 更新局部地图 滑窗
    // 实现滑窗就是用一个deque容器local_map_frames_把关键帧存储起来，关键帧超过一定数量，就把时间最靠前的关键帧给踢出去。
    local_map_frames_.push_back(key_frame);
    // zhihu上写的是　　while (local_map_frames_.size() > 20) {
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    // 没有找到直接的定义，意思很好理解，释放指针
    // void pcl::CloudIterator< PointT >::reset
    local_map_ptr_.reset(new CloudData::CLOUD());

    for (size_t i = 0; i < local_map_frames_.size(); ++i) {

        //	pcl::transformPointCloud (const pcl::PointCloud< PointT > &cloud_in, pcl::PointCloud< PointT > &cloud_out, const Eigen::Transform< Scalar, 3, Eigen::Affine > &transform, bool copy_all_fields=true) 
        //  应用由特征变换定义的仿射变换。
        //  有点没理解　TODO
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10) {

        //更新指针，指针指向　新的帧将要匹配的local map 他也是一个点云
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    // 保存所有关键帧信息在容器里
    // 存储之前，点云要先释放，因为已经存到了硬盘里，不释放也达不到节省内存的目的
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
    global_map_frames_.push_back(key_frame);

    return true;
}

//   FrontEndFlow::SaveMap
bool FrontEnd::SaveMap() {
    global_map_ptr_.reset(new CloudData::CLOUD());

    std::string key_frame_path = "";
    CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    for (size_t i = 0; i < global_map_frames_.size(); ++i) {
        key_frame_path = data_path_ + "/key_frames/key_frame_" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);

        pcl::transformPointCloud(*key_frame_cloud_ptr, 
                                *transformed_cloud_ptr, 
                                global_map_frames_.at(i).pose);
        *global_map_ptr_ += *transformed_cloud_ptr;
    }
    
    std::string map_file_path = data_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr_);
    has_new_global_map_ = true;

    return true;
}

bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    if (has_new_local_map_) {
        display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    if (has_new_global_map_) {
        has_new_global_map_ = false;
        display_filter_ptr_->Filter(global_map_ptr_, global_map_ptr);
        global_map_ptr_.reset(new CloudData::CLOUD());
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
    display_filter_ptr_->Filter(result_cloud_ptr_, current_scan_ptr);
    return true;
}
}
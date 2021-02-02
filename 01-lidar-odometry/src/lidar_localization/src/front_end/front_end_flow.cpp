/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/front_end/front_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"

// global def 里面没有啥
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {

    // 读取 config的配置文件并且存放在 YAML::Node 中
    // yaml-cpp是常用的yaml库
    std::string config_file_path = WORK_SPACE_PATH + "/config/dataset/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    // []的用法有点像json中就是将yaml中measuments这个信息提取出来
    // 将所有的subcriber初始化
    InitSubscribers(nh, config_node["measurements"]);

    // 将所有的subcriber初始化
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);

    front_end_ptr_ = std::make_shared<FrontEnd>();

    // using POINT = pcl::PointXYZ;
    // using CLOUD = pcl::PointCloud<POINT>;
    // reset 用法在官网上没有．．．　pcl::PointCloud<PointT>::Ptr::reset
    // stack overflow ：pcl::PointCloud<PointT>::Ptr::reset function then the memory is actually freed.
    local_map_ptr_.reset(new CloudData::CLOUD());
    global_map_ptr_.reset(new CloudData::CLOUD());
    current_scan_ptr_.reset(new CloudData::CLOUD());
}

bool FrontEndFlow::Run() {
    // 正常 if内都是 !true = false
    // 有问题就 return false
    if (!ReadData()) {  // 读取数据并将数据imu 和 gnss数据同步到　点云　数据所在的时间点
        return false;   // 如果是init还要将deque的front数据做一下同时，把不同时的数据pop掉
    }
        
    if (!InitCalibration()) {  // 根据输入的　frames 名称计算出   Eigen::Matrix4f　lidar_to_imu_
        return false;           // 使用一次，因为 calibration_received = true;
    }


    if (!InitGNSS()) {  // 将gnss坐标init 通过操作geo_converter
        return false;   // 使用一次，因为 gnss_inited = true;
    }

    while(HasData()) {  // 如果　每种传感器的deque中都有数据，进行下面的循环
       
        // while 循环，如果用数据就不断的进行　update gnss 和　laser的　odo 

        if (!ValidData()) { //　如果检测知道　deque 中的数据时间是同步，后将数据取出来
            continue;       // 放到current_xxx_data_ 里面　
        }                   // 　ValisDara返回fause 代表有数据被丢弃，！fause = true, 不做下面的Update . Pub Save等操作，再次循环，然后判断
            
        UpdateGNSSOdometry();   // 用gnss提供rigid transform中的t , 而　imu 提供旋转举证R
                                //　得到的是针对这一时刻跟新过的　gnss_odometry_　是gnss相对原点的pose
                                //　这里将gnss当做ground truth
       
        if (UpdateLaserOdometry()) {    // 这是前端的核心，这是核心，这是核心，用front_end_ptr_　指向的函数实现
                                        // 是一个bool函数，如果返回true 就是没有问题，将信息发布到指定的topic上
                                        // Updata这个函数主要就是跟新laser_odometry_　这个值他是一个rigid transform ,用　pub 和save 发布和保存这个值
            PublishData();          // 将当前帧点云和localmap ，在滤波后，发送到指定的　rostopic中
            SaveTrajectory();       // 不断地将laser_odometry 和　gnss_odometry_的数据输入　txt文本中
        } else {
            LOG(INFO) << "UpdateLaserOdometry failed!" << std::endl;
        }
    }

    return true;
}

bool FrontEndFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {

    // 初始化所有的subcriber，信息由传入的　YAML::Node　提供
    // hpp 中已经定义了ptr指向的对象类型，这里告诉这个对象应该读取的数据名称和相关信息
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(
        nh, 
        config_node["lidar"]["topic_name"].as<std::string>(), config_node["lidar"]["queue_size"].as<int>()
    );

    lidar_to_imu_ptr_ = std::make_shared<TFListener>(
        nh, 
        config_node["imu"]["frame_id"].as<std::string>(), config_node["lidar"]["frame_id"].as<std::string>()
    );

    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(
        nh, 
        config_node["imu"]["topic_name"].as<std::string>(), config_node["imu"]["queue_size"].as<int>()
    );
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(
        nh, 
        config_node["gnss"]["topic_name"].as<std::string>(), config_node["gnss"]["queue_size"].as<int>()
    );
    return true;
}

bool FrontEndFlow::ReadData() {
    // 将cloud_sub_ptr_中的点云存储信息到　cloud_data_buff　容器
    // _
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    // imu 和　gnss的信息不和点云同步，所以要通过 SyncData　函数插值(等比例缩放)计算同步时间时候的imu gnss值
    static std::deque<IMUData> unsynced_imu_;
    static std::deque<GNSSData> unsynced_gnss_;
    // 取得不同步时间的值
    imu_sub_ptr_->ParseData(unsynced_imu_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if (cloud_data_buff_.size() == 0) {
        return false;
    }
    
    // 找到需要同步的时间
    double cloud_time = cloud_data_buff_.front().time;
    // 函数中，通过比较imu gnss信号中自带的时间戳，找到需要的两组数据，分别是最接近cloud_time的两个
    // 通过对每一个元素的等比例缩放来算出数据存入　xxx_data_buff_
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

    static bool sensor_inited = false;
    if (!sensor_inited) {
        // 如果在同步时候出现错误，将会丢弃这一帧的点云信息并且用LOG记录
        // 这是在数据开头时候做的检测，怕数据记录的时候不是同时开始的．
        // 只在最初做一次，因为数据在deque中的最front的元素已同时
        if (!valid_imu || !valid_gnss) {//逻辑或
            LOG(INFO) << "Validity check: " << std::endl
                      << "IMU: " << valid_imu << ", "
                      << "GNSS: " << valid_gnss << std::endl;
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool FrontEndFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        // 和　00 一样，　找到lidar_to_imu_的转换矩阵　在　tf_listener.cpp中．
        // 主要是　lookupTransform　和　　TransformToMatrix　两个函数，找到转换矩阵并且
        // 存入   Eigen::Matrix4f　lidar_to_imu_　中
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool FrontEndFlow::InitGNSS() {
    static bool gnss_inited = false;
    
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool FrontEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;
    
    return true;
}

bool FrontEndFlow::ValidData() {
    //　获取deque最头上的一个数据
    // 判断时间差是否大于阈值，大于就将数据pop　
    // 符合条件就使用，并将deque 中的信息丢弃，因为已经到　current_xx_data 中．
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double d_time = current_cloud_data_.time - current_imu_data_.time;
    if (d_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (d_time > 0.05) {
        imu_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateGNSSOdometry() {
    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0,3) = current_gnss_data_.local_E;
    gnss_odometry_(1,3) = current_gnss_data_.local_N;
    gnss_odometry_(2,3) = current_gnss_data_.local_U;
    // 把imu的四元数转换成旋转矩阵送出去
    gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    // 转换到lidar坐标系
    gnss_odometry_ *= lidar_to_imu_;

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool front_end_pose_inited = false;
    // 将gnss提供的初始位置也视为lidar odo的初始位置
    // 然后就不管他了．
    if (!front_end_pose_inited) {
        front_end_pose_inited = true;
        front_end_ptr_->SetInitPose(gnss_odometry_);
    }

    laser_odometry_ = Eigen::Matrix4f::Identity();
    // update 是核心．
    // 输入currentscan ，更新　laser_odometry_　矩阵　并且用UpdateWithNewFrame更新local map
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    // 发布odo信息到指定的topic
    laser_odom_pub_ptr_->Publish(laser_odometry_);  // laser_odom topic
    gnss_pub_ptr_->Publish(gnss_odometry_);         // gnss topic
    // 调用了filter 类　，将当前帧点云取出来并进行滤波
    front_end_ptr_->GetCurrentScan(current_scan_ptr_);
    // 将点云current_scan_ptr_发布到指定的topic_name中
    // 这里是"current_scan"
    cloud_pub_ptr_->Publish(current_scan_ptr_);
    // 同样调用了filter 类　，将local_map_ptr_点云取出来并进行滤波
    if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
        local_map_pub_ptr_->Publish(local_map_ptr_);

    return true;
}

bool FrontEndFlow::SaveTrajectory() {
    // 简单的不断存储信息
    static std::ofstream ground_truth, laser_odom;
    static bool is_file_created = false;

    if (!is_file_created) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
            return false;
        if (!FileManager::CreateFile(ground_truth, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt"))
            return false;
        if (!FileManager::CreateFile(laser_odom, WORK_SPACE_PATH + "/slam_data/trajectory/laser_odom.txt"))
            return false;
        is_file_created = true;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth << gnss_odometry_(i, j);
            laser_odom << laser_odometry_(i, j);
            if (i == 2 && j == 3) {
                ground_truth << std::endl;
                laser_odom << std::endl;
            } else {
                ground_truth << " ";
                laser_odom << " ";
            }
        }
    }

    return true;
}

bool FrontEndFlow::SaveMap() {
    return front_end_ptr_->SaveMap();
}

bool FrontEndFlow::PublishGlobalMap() {
    // 如果检测到新的global map　那么调用　global_map_pub_ptr　发布它
    if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) { 
        global_map_pub_ptr_->Publish(global_map_ptr_);
        global_map_ptr_.reset(new CloudData::CLOUD());
    }

    return true;
}

}
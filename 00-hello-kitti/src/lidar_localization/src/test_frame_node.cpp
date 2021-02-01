/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    //  记录代码日志
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    // 先创建一个对象，make_shared创建了一个指针， 指针指向某个对象，这个对象就是 subcriber
    // shard ptr 智能指针，指向类型为 <CloudSubscriber> 中的对象指针 ，名叫cloud_sub_ptr
    // make_shared 在动态内存中分配一个对象并初始化它，返回指向此对象的shared_ptr，与智能指针一样 
    // 当要用make_shared时，必须指定创建的对象类型，定义方式与模板类相同，在函数名之后跟一个尖括号，在其中给出类型；
    // 后面的括号则是创建对象的构造函数要求的信息。

    // 执行subcriber的构建函数时候,自动调用private的msg_callback函数.也就是接收和处理信息的地方,接收从ros中输出的数据.
    // 以 GNSS为例注释
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    // velo 应该就是雷达的位置把？
    std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "imu_link", "velo_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

    // 建立容器储存数据
    std::deque<CloudData> cloud_data_buff;
    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;
    // 建立 4*4的矩阵 --> 4f ， 赋值为单位矩阵
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
    bool transform_received = false;
    bool gnss_origin_position_inited = false;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        //将数据储存到对应的data_buff
        //类中的函数ParseData就是实现从类(subcriber)里取数据的功能
        cloud_sub_ptr->ParseData(cloud_data_buff);
        //没有用到imudata,这就是个寂寞,但是后面会用的
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if (!transform_received) {
            
            //if内返回的是lidar_to_imu的矩阵,通过定义 lidar_to_imu_ptr时候的输入frame( "imu_link" "velo_link")
            // 用rosrun rqt_tf_tree rqt_tf_tree  查看
            if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
                transform_received = true;
                // LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu;
                // 算出了lidar_to_imu 的transformmatrix就在下一次循环中进行 else ,这里lidar_to_imu 的的transformmatrix不会变
            }
        } else {
            // 三个rostopic都有数据的时候进行数据时间判断，如果deque首个数据时间不一致，将时间靠前的数据丢弃
            // 不能呢个保证时间完全一样的嘛，毕竟是不同的传感器有不同的频率。允许0.05秒的时间差
            while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
                
                // 取数据
                CloudData cloud_data = cloud_data_buff.front();
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();

                // 算数据的时间差
                double d_time = cloud_data.time - imu_data.time;
                if (d_time < -0.05) {
                    
                    // 丢弃数据
                    cloud_data_buff.pop_front();
                } else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else {

                    // 更新容器，意思是将会使用容器中最靠前的数据，使用过容器就要跟新了
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    Eigen::Matrix4f odometry_matrix;

                    if (!gnss_origin_position_inited) {

                        // 设置 gnss数据的初始点
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }
                    // 将经纬度转化成xyz
                    gnss_data.UpdateXYZ();
                    //1行4列元素，从0开始数
                    odometry_matrix(0,3) = gnss_data.local_E;
                    //2行4列元素，从0开始数
                    odometry_matrix(1,3) = gnss_data.local_N;
                    odometry_matrix(2,3) = gnss_data.local_U;
                    // block矩阵块操作,把右边得到的旋转矩阵R赋值到左边odometry_matrix 从 0，0开始的 3*3矩阵 <3,3>(0,0)
                    // 构建成一个4*4的rigid transformation 
                    odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();
                    //矩阵相乘,
                    odometry_matrix *= lidar_to_imu;

                    // 把雷达数据沾到 odo上。
                    pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, odometry_matrix);

                    cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
                    odom_pub_ptr->Publish(odometry_matrix);
                }
            }
        }

        rate.sleep();
    }

    return 0;
}
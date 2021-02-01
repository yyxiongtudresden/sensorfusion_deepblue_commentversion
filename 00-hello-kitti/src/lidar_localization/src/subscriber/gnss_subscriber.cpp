/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-03-31 13:10:51
 */
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) 
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
}


//  nav_sat_fix_ptr这个是哪里来的？
//  应该是上面 topic_name中指定的ros topic，按照NavSatFixConstPtr的格式读取信息并赋赋值给gnssdata
void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr) {
    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->altitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;
    // .hpp std::deque<GNSSData> new_gnss_data_;
    // new_gnss_data_ 是一个储存未使用过的gnss数据的容器deque
    new_gnss_data_.push_back(gnss_data);
}

void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff) {
    if (new_gnss_data_.size() > 0) {
        gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }
}
}
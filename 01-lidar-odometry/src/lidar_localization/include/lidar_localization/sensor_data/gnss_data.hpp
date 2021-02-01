/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:25:13
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

#include <GeographicLib/LocalCartesian.hpp>

namespace lidar_localization {
class GNSSData {
  public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

  private:

    //静态成员变量必须在类外初始化
    // 两个相同类型的对象 a、b，它们都有一个成员变量 m_name
    // 那么修改 a.m_name 的值不会影响 b.m_name 的值。
    // 可是有时候我们希望在多个对象之间共享数据--> 静态成员变量
    // static 成员变量属于类，不属于某个具体的对象，即使创建多个对象，也只为 m_total 分配一份内存 
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

  public: 
    void InitOriginPosition();
    void UpdateXYZ();
    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};
}
#endif
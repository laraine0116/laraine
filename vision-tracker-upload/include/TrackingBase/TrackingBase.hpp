#pragma once

#include <cstdint>

#include "data_types.hpp"

namespace rm {

// 跟踪基类：保存所有目标共有的状态（编号、类型、位置、速度、预测值、丢失计数）
// 丢失阈值和角度预测由派生类决定，基类提供公共辅助函数
class TrackingBase {
public:
    TrackingBase(int id, TargetType type);
    virtual ~TrackingBase() = default;   // 通过基类指针销毁派生对象，必须 virtual

    // 本帧被观测到：更新位置/速度/预测值，missed_count 清零
    void observe(const Observation& obs, std::int64_t ts_ns,
                 std::int64_t prev_ts_ns);

    // 本帧未被观测：missed_count++，未达阈值则推进预测，达到阈值转 LOST
    void missed(std::int64_t ts_ns, std::int64_t prev_ts_ns);

    // 导出快照
    TargetSnapshot snapshot() const;

    int        id()   const { return id_; }
    TargetType type() const { return type_; }

protected:
    // 派生类实现：丢失阈值（普通 3 / 旋转 5）
    virtual int lostThreshold() const = 0;

    // 派生类实现：是否维护角度（普通 false / 旋转 true）
    virtual bool hasAngle() const = 0;

    // 派生类构造函数专用：写入初始状态
    void initPose(double x, double y, double angle);
    void initVelocity(double vx, double vy, double w);
    void initPrediction(double px, double py, double pa);
    void initTrackStatus();
    void initLastObs(double x, double y, double angle, std::int64_t ts_ns);

    // 派生类只读访问
    double x()     const { return x_; }
    double y()     const { return y_; }
    double angle() const { return angle_; }
    TargetStatus status() const { return status_; }

private:
    // 进入 LOST：清零位置、角度、速度、预测值
    void enterLost();

    int          id_           = 0;
    TargetType   type_         = TargetType::ARMOR;
    TargetStatus status_       = TargetStatus::TRACK;

    double x_     = 0.0;
    double y_     = 0.0;
    double angle_ = 0.0;

    double velocity_x_       = 0.0;
    double velocity_y_       = 0.0;
    double angular_velocity_ = 0.0;

    double predicted_x_     = 0.0;
    double predicted_y_     = 0.0;
    double predicted_angle_ = 0.0;

    int missed_count_ = 0;

    // 上一次真实观测（用于算 dt_obs 和速度差值）
    double       last_obs_x_     = 0.0;
    double       last_obs_y_     = 0.0;
    double       last_obs_angle_ = 0.0;
    std::int64_t last_obs_ts_    = 0;
    bool         has_last_obs_   = false;
};

}  // namespace rm

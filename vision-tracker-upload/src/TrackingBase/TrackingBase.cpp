#include "TrackingBase/TrackingBase.hpp"

namespace rm {

namespace {
// 帧间隔（秒）：首帧取 0，之后按时间戳真实差值
double frameGapSec(std::int64_t ts_ns, std::int64_t prev_ts_ns) {
    if (prev_ts_ns < 0) {
        return 0.0;   // 首帧 dt 取 0
    }
    const double d = static_cast<double>(ts_ns - prev_ts_ns) / 1e9;
    return d > 0.0 ? d : 0.0;
}
}  // namespace

TrackingBase::TrackingBase(int id, TargetType type)
    : id_(id), type_(type) {}

// ---------------- 派生类初始化辅助 ----------------

void TrackingBase::initPose(double x, double y, double angle) {
    x_     = x;
    y_     = y;
    angle_ = angle;
}

void TrackingBase::initVelocity(double vx, double vy, double w) {
    velocity_x_       = vx;
    velocity_y_       = vy;
    angular_velocity_ = w;
}

void TrackingBase::initPrediction(double px, double py, double pa) {
    predicted_x_     = px;
    predicted_y_     = py;
    predicted_angle_ = pa;
}

void TrackingBase::initTrackStatus() {
    status_       = TargetStatus::TRACK;
    missed_count_ = 0;
}

void TrackingBase::initLastObs(double x, double y, double angle,
                               std::int64_t ts_ns) {
    last_obs_x_     = x;
    last_obs_y_     = y;
    last_obs_angle_ = angle;
    last_obs_ts_    = ts_ns;
    has_last_obs_   = true;
}

// ---------------- 逐帧行为 ----------------

void TrackingBase::observe(const Observation& obs, std::int64_t ts_ns,
                           std::int64_t prev_ts_ns) {
    const double dt = frameGapSec(ts_ns, prev_ts_ns);

    // ---- 从 LOST 重新追踪：速度清零，predicted_* = 本次观测 ----
    if (status_ == TargetStatus::LOST) {
        status_       = TargetStatus::TRACK;
        missed_count_ = 0;

        x_     = obs.x;
        y_     = obs.y;
        angle_ = hasAngle() ? obs.angle : 0.0;

        velocity_x_       = 0.0;
        velocity_y_       = 0.0;
        angular_velocity_ = 0.0;

        predicted_x_     = x_;
        predicted_y_     = y_;
        predicted_angle_ = angle_;

        last_obs_x_     = obs.x;
        last_obs_y_     = obs.y;
        last_obs_angle_ = angle_;
        last_obs_ts_    = ts_ns;
        has_last_obs_   = true;
        return;
    }

    // ---- 正常 TRACK 更新：按相邻观测时间差算速度 ----
    if (has_last_obs_) {
        const double dt_obs =
            static_cast<double>(ts_ns - last_obs_ts_) / 1e9;
        if (dt_obs > 0.0) {
            velocity_x_ = (obs.x - last_obs_x_) / dt_obs;
            velocity_y_ = (obs.y - last_obs_y_) / dt_obs;
            if (hasAngle()) {
                angular_velocity_ = (obs.angle - last_obs_angle_) / dt_obs;
            }
        }
    }

    x_     = obs.x;
    y_     = obs.y;
    angle_ = hasAngle() ? obs.angle : 0.0;

    // predicted_* = 当前坐标 + 速度 × 当前帧间隔
    predicted_x_ = x_ + velocity_x_ * dt;
    predicted_y_ = y_ + velocity_y_ * dt;
    if (hasAngle()) {
        predicted_angle_ = angle_ + angular_velocity_ * dt;
    } else {
        angular_velocity_ = 0.0;
        predicted_angle_  = 0.0;
    }

    status_       = TargetStatus::TRACK;
    missed_count_ = 0;

    last_obs_x_     = obs.x;
    last_obs_y_     = obs.y;
    last_obs_angle_ = angle_;
    last_obs_ts_    = ts_ns;
    has_last_obs_   = true;
}

void TrackingBase::missed(std::int64_t ts_ns, std::int64_t prev_ts_ns) {
    // 已完成 LOST：不再推进、不再累加（冻结在阈值）
    if (status_ == TargetStatus::LOST) {
        return;
    }

    ++missed_count_;

    if (missed_count_ >= lostThreshold()) {
        // 达到阈值这一帧：不再推进预测，直接转 LOST 并清零
        enterLost();
        return;
    }

    // 未达阈值：只写入 predicted_*，不覆盖 x_ / y_
    const double dt = frameGapSec(ts_ns, prev_ts_ns);

    predicted_x_ += velocity_x_ * dt;
    predicted_y_ += velocity_y_ * dt;

    if (hasAngle()) {
        angle_           += angular_velocity_ * dt;   // 先推进角度
        predicted_angle_  = angle_ + angular_velocity_ * dt;
    } else {
        predicted_angle_ = 0.0;
    }
}

void TrackingBase::enterLost() {
    status_           = TargetStatus::LOST;
    x_                = 0.0;
    y_                = 0.0;
    angle_            = 0.0;
    velocity_x_       = 0.0;
    velocity_y_       = 0.0;
    angular_velocity_ = 0.0;
    predicted_x_      = 0.0;
    predicted_y_      = 0.0;
    predicted_angle_  = 0.0;
}

TargetSnapshot TrackingBase::snapshot() const {
    TargetSnapshot s;
    s.id               = id_;
    s.type             = type_;
    s.status           = status_;
    s.x                = x_;
    s.y                = y_;
    s.velocity_x       = velocity_x_;
    s.velocity_y       = velocity_y_;
    s.predicted_x      = predicted_x_;
    s.predicted_y      = predicted_y_;
    s.angle            = hasAngle() ? angle_ : 0.0;
    s.angular_velocity = hasAngle() ? angular_velocity_ : 0.0;
    s.predicted_angle  = hasAngle() ? predicted_angle_ : 0.0;
    s.missed_count     = missed_count_;
    return s;
}

}  // namespace rm

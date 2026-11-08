#include "SpinningTarget/SpinningTarget.hpp"

namespace rm {

SpinningTarget::SpinningTarget(int id, const Observation& obs,
                               std::int64_t ts_ns)
    : TrackingBase(id, TargetType::SPINNING) {
    // 新建：位置与角度都取观测
    initPose(obs.x, obs.y, obs.angle);

    // 速度与角速度均为 0
    initVelocity(0.0, 0.0, 0.0);

    // 预测值等于观测值
    initPrediction(obs.x, obs.y, obs.angle);

    // 状态 TRACK、missed_count = 0
    initTrackStatus();

    // 记录首次观测，供下一帧计算 dt_obs
    initLastObs(obs.x, obs.y, obs.angle, ts_ns);
}

}  // namespace rm

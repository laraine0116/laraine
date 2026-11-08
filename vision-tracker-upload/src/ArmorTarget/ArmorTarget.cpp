#include "ArmorTarget/ArmorTarget.hpp"

namespace rm {

ArmorTarget::ArmorTarget(int id, const Observation& obs, std::int64_t ts_ns)
    : TrackingBase(id, TargetType::ARMOR) {
    // 新建：位置取观测，普通装甲角度固定 0
    initPose(obs.x, obs.y, 0.0);

    // 速度与角速度均为 0
    initVelocity(0.0, 0.0, 0.0);

    // 预测值等于观测值，角度预测固定 0
    initPrediction(obs.x, obs.y, 0.0);

    // 状态 TRACK、missed_count = 0
    initTrackStatus();

    // 记录首次观测，供下一帧计算 dt_obs
    initLastObs(obs.x, obs.y, 0.0, ts_ns);
}

}  // namespace rm

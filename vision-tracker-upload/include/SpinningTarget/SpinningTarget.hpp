#pragma once

#include "TrackingBase/TrackingBase.hpp"

namespace rm {

// 旋转装甲：丢失阈值 5 帧，维护角度与角速度
class SpinningTarget : public TrackingBase {
public:
    explicit SpinningTarget(int id, const Observation& obs, std::int64_t ts_ns);

protected:
    int  lostThreshold() const override { return 5; }
    bool hasAngle()      const override { return true; }
};

}  // namespace rm

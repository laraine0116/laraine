#pragma once

#include "TrackingBase/TrackingBase.hpp"

namespace rm {

// 普通装甲：丢失阈值 3 帧，不维护角度
class ArmorTarget : public TrackingBase {
public:
    explicit ArmorTarget(int id, const Observation& obs, std::int64_t ts_ns);

protected:
    int  lostThreshold() const override { return 3; }
    bool hasAngle()      const override { return false; }
};

}  // namespace rm

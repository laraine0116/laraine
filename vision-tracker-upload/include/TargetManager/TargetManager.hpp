#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "TrackingBase/TrackingBase.hpp"
#include "data_types.hpp"

namespace rm {

class TargetManager {
public:
    Result onFrame(const InputFrame& frame);

private:
    bool validate(const InputFrame& frame) const;

    Result buildResult(int frame_id, bool accepted) const;

    TrackingBase* find(int id);

    std::vector<std::unique_ptr<TrackingBase>> trackers_;
    int          last_ok_frame_id_ = -1;
    std::int64_t last_ok_ts_       = -1;
};

}  // namespace rm

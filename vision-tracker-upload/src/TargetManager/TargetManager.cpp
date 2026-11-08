#include "TargetManager/TargetManager.hpp"

#include <cmath>
#include <set>

#include "ArmorTarget/ArmorTarget.hpp"
#include "SpinningTarget/SpinningTarget.hpp"

namespace rm {

TrackingBase* TargetManager::find(int id) {
    for (auto& p : trackers_) {
        if (p->id() == id) return p.get();
    }
    return nullptr;
}

bool TargetManager::validate(const InputFrame& frame) const {
    // 1. 帧号严格大于最近成功帧
    if (frame.frame_id <= last_ok_frame_id_) {
        return false;
    }

    // 2. 时间戳严格大于最近成功帧
    if (last_ok_ts_ >= 0 && frame.timestamp_ns <= last_ok_ts_) {
        return false;
    }

    std::set<int> ids;
    for (const auto& obs : frame.observations) {
        // 3. 负数编号
        if (obs.id < 0) return false;

        // 4. 同帧重复编号
        if (!ids.insert(obs.id).second) return false;

        // 5. 已创建目标类型不一致
        for (const auto& p : trackers_) {
            if (p->id() == obs.id && p->type() != obs.type) {
                return false;
            }
        }
    }

    return true;
}

Result TargetManager::buildResult(int frame_id, bool accepted) const {
    Result result;
    result.accepted = accepted;
    result.frame_id = frame_id;

    result.targets.clear();
    for (const auto& p : trackers_) {
        result.targets.push_back(p->snapshot());
    }

    result.tracking_target_count = 0;
    result.lost_target_count     = 0;
    result.armor_target_count    = 0;
    result.spinning_target_count = 0;

    for (const auto& t : result.targets) {
        if (t.status == TargetStatus::TRACK) {
            ++result.tracking_target_count;
        } else {
            ++result.lost_target_count;
        }
        if (t.type == TargetType::ARMOR) {
            ++result.armor_target_count;
        } else {
            ++result.spinning_target_count;
        }
    }

    return result;
}

Result TargetManager::onFrame(const InputFrame& frame) {
    // 校验失败：accepted=false，返回未变化的快照
    if (!validate(frame)) {
        return buildResult(frame.frame_id, false);
    }

    const std::int64_t prev_ts = last_ok_ts_;

    // 本帧观测到的编号集合
    std::set<int> observed_ids;
    for (const auto& obs : frame.observations) {
        observed_ids.insert(obs.id);
    }

    // 1) 处理被观测到的目标
    for (const auto& obs : frame.observations) {
        TrackingBase* t = find(obs.id);
        if (t != nullptr) {
            t->observe(obs, frame.timestamp_ns, prev_ts);
        } else {
            // 新建目标
            if (obs.type == TargetType::ARMOR) {
                trackers_.push_back(std::make_unique<ArmorTarget>(
                    obs.id, obs, frame.timestamp_ns));
            } else {
                trackers_.push_back(std::make_unique<SpinningTarget>(
                    obs.id, obs, frame.timestamp_ns));
            }
        }
    }

    // 2) 处理本帧未被观测到的目标
    for (auto& p : trackers_) {
        if (observed_ids.count(p->id()) == 0) {
            p->missed(frame.timestamp_ns, prev_ts);
        }
    }

    // 3) 全部通过后更新最近成功帧
    last_ok_frame_id_ = frame.frame_id;
    last_ok_ts_       = frame.timestamp_ns;

    return buildResult(frame.frame_id, true);
}

}  // namespace rm

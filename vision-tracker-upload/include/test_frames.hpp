/*
* @FilePath: include/test_frames.hpp
* @Description: 测试帧序列，覆盖普通/旋转装甲的跟踪、丢失、预测、重新追踪，
*               以及五类非法帧；编号首次出现时创建目标。
*               观测时间按 frame_id * 100 ms 设置（帧 24 的时间戳回退用例除外）；被拒绝的帧不更新时间基准。
*/

#pragma once

#include "data_types.hpp"


namespace
{
    constexpr std::int64_t frameTimeNs(int frame_id)
    {
        return static_cast<std::int64_t>(frame_id) * 100'000'000LL; // 0.1 s
    }
}

inline const std::vector<rm::InputFrame>& testFrames()
{
    static const std::vector<rm::InputFrame> frames = {
        // ---- 1、2 首次出现并创建；正常跟踪、普通装甲 3 帧丢失、旋转装甲预测 ----
        rm::InputFrame{0, frameTimeNs(0), {{1, rm::TargetType::ARMOR, 1.0, 2.0, 0.0}, {2, rm::TargetType::SPINNING, 5.0, 0.0, 0.0}}},
        rm::InputFrame{1, frameTimeNs(1), {{1, rm::TargetType::ARMOR, 2.0, 2.0, 0.0}, {2, rm::TargetType::SPINNING, 5.0, 0.0, 0.2}}},
        rm::InputFrame{2, frameTimeNs(2), {{1, rm::TargetType::ARMOR, 3.0, 2.0, 0.0}}},
        rm::InputFrame{3, frameTimeNs(3), {{1, rm::TargetType::ARMOR, 4.0, 2.0, 0.0}}},
        rm::InputFrame{4, frameTimeNs(4), {}},
        rm::InputFrame{5, frameTimeNs(5), {}},
        rm::InputFrame{6, frameTimeNs(6), {{1, rm::TargetType::ARMOR, 5.0, 2.0, 0.0}}},

        // ---- 普通装甲继续丢失直到 LOST ----
        rm::InputFrame{7, frameTimeNs(7), {}},
        rm::InputFrame{8, frameTimeNs(8), {}},
        rm::InputFrame{9, frameTimeNs(9), {{1, rm::TargetType::ARMOR, 6.0, 2.0, 0.0}}},
        rm::InputFrame{10, frameTimeNs(10), {}},
        rm::InputFrame{11, frameTimeNs(11), {}},
        rm::InputFrame{12, frameTimeNs(12), {}},

        // ---- LOST 后再次观测：重新追踪 ----
        rm::InputFrame{13, frameTimeNs(13), {{1, rm::TargetType::ARMOR, 7.0, 2.0, 0.0}}},
        rm::InputFrame{14, frameTimeNs(14), {{3, rm::TargetType::ARMOR, 1.0, 1.0, 0.0}}}, // 3 首次出现 → 创建
        rm::InputFrame{15, frameTimeNs(15), {{1, rm::TargetType::ARMOR, 8.0, 2.0, 0.0}, {3, rm::TargetType::ARMOR, 2.0, 1.0, 0.0}}},

        // ---- 16/19/15/24 四类非法帧；23 类型不一致，全部应被整帧拒绝 ----
        rm::InputFrame{16, frameTimeNs(16), {{1, rm::TargetType::ARMOR, 9.0, 2.0, 0.0}, {1, rm::TargetType::ARMOR, 9.0, 2.0, 0.0}}},  // 重复编号
        rm::InputFrame{19, frameTimeNs(19), {{-1, rm::TargetType::ARMOR, 0.0, 0.0, 0.0}}},                                    // 负数编号
        rm::InputFrame{15, frameTimeNs(15), {{1, rm::TargetType::ARMOR, 99.0, 99.0, 0.0}}},                                   // 帧号回退

        // ---- 拒绝后状态不变，且能继续处理 ----
        rm::InputFrame{16, frameTimeNs(16), {}},
        rm::InputFrame{17, frameTimeNs(17), {{1, rm::TargetType::ARMOR, 9.0, 2.0, 0.0}, {3, rm::TargetType::ARMOR, 3.0, 1.0, 0.0}, {9, rm::TargetType::ARMOR, 4.0, 4.0, 0.0}}}, // 9 首次出现 → 创建
        rm::InputFrame{18, frameTimeNs(18), {}},
        rm::InputFrame{19, frameTimeNs(19), {}},
        rm::InputFrame{20, frameTimeNs(20), {}},

        // ---- 旋转装甲 LOST 后重新追踪；22 验证创建目标在 LOST 后重新追踪 ----
        rm::InputFrame{21, frameTimeNs(21), {{2, rm::TargetType::SPINNING, 5.0, 0.0, 3.0}}},
        rm::InputFrame{22, frameTimeNs(22), {{9, rm::TargetType::ARMOR, 5.0, 5.0, 0.0}}}, // 创建的 9 重新追踪

        // ---- 类型不一致：默认整帧拒绝；启用附加二时接受并重建 ----
        rm::InputFrame{23, frameTimeNs(23), {{1, rm::TargetType::SPINNING, 8.0, 2.0, 0.0}}},

        // ---- 时间戳回退：帧号更大，但时间戳不晚于最近成功帧 ----
        rm::InputFrame{24, frameTimeNs(22), {}},
    };
    return frames;
}

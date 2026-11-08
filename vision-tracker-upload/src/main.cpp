/*
 * @FilePath: src/main.cpp
 * @Description: 程序入口：创建节点、逐帧调用 onFrame 并打印结果。
 */

#include <iomanip>
#include <iostream>
#include "TargetManager/TargetManager.hpp"
#include "data_types.hpp"
#include "test_frames.hpp"

using rm::InputFrame;
using rm::Result;
using rm::TargetSnapshot;
using rm::TargetStatus;
using rm::TargetManager;



// 打印一个目标的信息
void printTarget(const TargetSnapshot &target)
{
    std::cout << "id=" << target.id
              << " type=" << static_cast<int>(target.type)
              << " status=" << static_cast<int>(target.status)
              << " position=(";

    if (target.status == TargetStatus::LOST && target.x == 0.0 && target.y == 0.0)
    {
        std::cout << "--,--";
    }
    else
    {
        std::cout << std::fixed << std::setprecision(2) << target.x << ',' << target.y;
    }

    std::cout << ")";
    std::cout << " velocity=(" << std::fixed << std::setprecision(2)
              << target.velocity_x << ',' << target.velocity_y
              << ") predicted_position=(";
    if (target.status == TargetStatus::LOST)
    {
        std::cout << "--,--";
    }
    else
    {
        std::cout << target.predicted_x << ',' << target.predicted_y;
    }

    std::cout << ") angle=" << std::fixed << std::setprecision(2) << target.angle
              << " angular_velocity=" << target.angular_velocity
              << " predicted_angle=" << target.predicted_angle
              << " missed=" << target.missed_count << '\n';
}

/// 打印一帧的结果
void printResult(const Result &result)
{
    std::cout << "frame=" << result.frame_id
              << " accepted=" << (result.accepted ? "true" : "false") << '\n';
    std::cout << "tracking_target_count=" << result.tracking_target_count
              << " lost_target_count=" << result.lost_target_count
              << " armor_target_count=" << result.armor_target_count
              << " spinning_target_count=" << result.spinning_target_count << '\n';

    for (const auto &target : result.targets)
    {
        printTarget(target);
    }
}

int main()
{
    TargetManager target_manager;

    for (const auto &frame : testFrames())
    {
        rm::Result result = target_manager.onFrame(frame);

        printResult(result); // 用于输出结果的函数
    }
    return 0;
}

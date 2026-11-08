/*
 * @FilePath: include/data_types.hpp
 * @Description: 跟踪框架的数据结构约定：禁止修改
 */

#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

namespace rm
{

    /// 目标类型
    enum class TargetType
    {
        ARMOR,   // 普通装甲
        SPINNING // 旋转装甲
    };

    /// 目标状态
    enum class TargetStatus
    {
        LOST, // 丢失：位置、角度、速度与预测值均已清零，等待重新观测
        TRACK // 正在跟踪：最近一次观测仍在丢失阈值内
    };

    /// 一帧中检测到的一个目标（原始检测值，不含跟踪状态）
    struct Observation
    {
        int id;          // 装甲板编号；已创建则更新，未创建则按本帧类型新建目标
        TargetType type; // 目标类型；已创建时必须一致，新建时作为新目标的类型
        double x;        // 检测到的 x 坐标，单位 m
        double y;        // 检测到的 y 坐标，单位 m
        double angle;    // 检测到的角度，单位度；普通装甲该字段无意义
    };

    /// 框架交给节点的一帧数据
    struct InputFrame
    {
        int frame_id;                          // 帧编号；需大于最近一次成功处理的帧号
        std::int64_t timestamp_ns;             // 观测时刻，纳秒；必须严格大于最近成功帧，否则整帧拒绝
        std::vector<Observation> observations; // 本帧检测到的目标；同一帧内编号不得重复
    };

    /// 单个目标的当前快照，用于结果输出
    struct TargetSnapshot
    {
        int id;                  // 装甲板编号
        TargetType type;         // 目标类型
        TargetStatus status;     // 当前跟踪状态
        double x;                // 当前 x 坐标，单位 m；LOST 后为 0
        double y;                // 当前 y 坐标，单位 m；LOST 后为 0
        double velocity_x;       // x 轴速度，单位 m/s；相邻观测的坐标差除以时间差
        double velocity_y;       // y 轴速度，单位 m/s；相邻观测的坐标差除以时间差
        double predicted_x;      // 按当前帧时间间隔预测的下一帧 x，单位 m；仅 TRACK 时有效；未观测时继续累加
        double predicted_y;      // 按当前帧时间间隔预测的下一帧 y，单位 m；仅 TRACK 时有效；未观测时继续累加
        double angle;            // 当前角度，单位度；普通装甲固定为 0；LOST 后为 0
        double angular_velocity; // 角速度，单位度/秒；相邻观测的角度差除以时间差；普通装甲固定为 0
        double predicted_angle;  // 按当前帧时间间隔预测的下一帧角度，单位度；仅旋转装甲 TRACK 时有效
        int missed_count;        // 连续未观测到的帧数；LOST 后是否继续增加由实现决定；重新观测到时清零
    };

    /// 节点处理完一帧后的结果
    struct Result
    {
        bool accepted{};                     // 本帧是否通过校验并被接受；false 表示整帧拒绝
        int frame_id{};                      // 本帧输入帧号；被拒绝时也填该帧号
        std::vector<TargetSnapshot> targets; // 当前所有目标的快照（含 LOST）；被拒绝时保持不变
        int tracking_target_count;           // targets 中状态为 TRACK 的数量
        int lost_target_count;               // targets 中状态为 LOST 的数量
        int armor_target_count;              // targets 中类型为 ARMOR 的数量（含 LOST）
        int spinning_target_count;           // targets 中类型为 SPINNING 的数量（含 LOST）
    };

} // namespace rm

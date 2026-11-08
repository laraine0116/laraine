# 视觉目标管理与跟踪系统

创梦之翼视觉组 2026 招新培训 · 第二次作业（C++17）

## 1. 项目简介

本项目实现一个逐帧运行的视觉目标管理器：接收每帧的检测结果（`rm::InputFrame`），维护普通装甲与旋转装甲两类目标，处理正常跟踪、丢失、重新追踪和非法帧，并输出每一帧所有目标的快照（`rm::Result`）。

数据契约 `include/data_types.hpp` 与固定测试序列 `include/test_frames.hpp` 保持原样，未做修改。

## 2. 功能

- 普通装甲：连续 3 帧未观测进入 `LOST`；跟踪期间按 x/y 速度预测位置。
- 旋转装甲：连续 5 帧未观测进入 `LOST`；跟踪期间预测位置和角度。
- 速度按相邻观测的时间差 `dt_obs` 计算；预测按当前成功帧的时间间隔 `dt` 计算。
- 整帧校验：帧号回退、时间戳回退、负数编号、同帧重复编号、已创建目标类型不一致，任一命中则整帧拒绝，且不修改任何目标。
- `LOST` 目标保留在容器中；重新观测后恢复 `TRACK`，速度清零，预测值取本次观测值。
- 输出 `tracking_target_count`、`lost_target_count`、`armor_target_count`、`spinning_target_count` 四类计数。

## 3. 目录结构

```text
vision-tracker/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── .vscode/
│   └── c_cpp_properties.json
├── include/
│   ├── data_types.hpp
│   ├── test_frames.hpp
│   ├── TrackingBase/TrackingBase.hpp
│   ├── ArmorTarget/ArmorTarget.hpp
│   ├── SpinningTarget/SpinningTarget.hpp
│   └── TargetManager/TargetManager.hpp
└── src/
    ├── main.cpp
    ├── TrackingBase/TrackingBase.cpp
    ├── ArmorTarget/ArmorTarget.cpp
    ├── SpinningTarget/SpinningTarget.cpp
    └── TargetManager/TargetManager.cpp
```

`build/` 为本地构建产物，不纳入版本管理。

## 4. 类设计

### 4.1 `TrackingBase`

跟踪基类，保存所有目标共有的状态：

- 编号、类型、当前状态（`TRACK` / `LOST`）
- 当前位置、角度
- x/y 速度、角速度
- x/y 预测值、角度预测值
- 连续未观测帧数
- 上一次真实观测的位置、角度和时间戳

对外提供 `observe()`、`missed()` 和 `snapshot()`；派生类通过 `lostThreshold()` 与 `hasAngle()` 决定丢失阈值和是否维护角度。析构函数声明为 `virtual`，保证通过基类指针销毁派生对象时行为正确。所有数据成员均为 `private`，通过成员函数访问。

### 4.2 `ArmorTarget`

普通装甲，继承 `TrackingBase`：

- 丢失阈值 3 帧
- 不维护角度，`angle`、`angular_velocity`、`predicted_angle` 始终为 0
- 创建时位置取观测值，速度与预测值初始化

### 4.3 `SpinningTarget`

旋转装甲，继承 `TrackingBase`：

- 丢失阈值 5 帧
- 维护角度与角速度
- 未观测但仍在 `TRACK` 时，先执行 `angle += angular_velocity * dt`，再计算 `predicted_angle = angle + angular_velocity * dt`
- 达到丢失阈值的那一帧不再推进角度

### 4.4 `TargetManager`

目标管理器，负责：

- 整帧校验
- 遍历本帧观测，更新已有目标或创建新目标
- 遍历本帧未观测目标，更新丢失计数和预测值
- 统计四类计数并构造 `Result`

目标容器为 `std::vector<std::unique_ptr<TrackingBase>>`，通过基类指针多态调用派生类实现。

## 5. 关键实现细节

### 5.1 时间与速度

- `dt = (t_ns - t_prev_ns) / 1e9`：当前成功帧与上一成功帧的时间差；首帧取 0。
- `dt_obs = (t_ns - 上次观测时间戳) / 1e9`：目标相邻两次真实观测的时间差。
- 速度使用 `dt_obs`，预测使用 `dt`，两者不能混用。
- 被拒绝的帧不更新最近成功帧号和时间戳，因此不会污染后续 `dt`。

### 5.2 预测规则

- 本帧观测到 `TRACK` 目标：`predicted_x = x + velocity_x * dt`，`predicted_y = y + velocity_y * dt`。
- 本帧未观测且仍为 `TRACK`：`predicted_x += velocity_x * dt`，`predicted_y += velocity_y * dt`。
- 旋转装甲在观测到时先更新角速度，再计算预测角；未观测时先推进当前角度，再计算预测角。
- 预测值只写入 `predicted_*`，不覆盖当前 `x` / `y` / `angle`。

### 5.3 丢失与重新追踪

- 普通装甲连续 3 帧、旋转装甲连续 5 帧未观测时进入 `LOST`。
- 进入 `LOST` 时清零位置、角度、速度、角速度和全部预测值。
- `LOST` 目标不从容器删除；重新观测时状态恢复为 `TRACK`，`missed_count` 清零，速度清零，预测值等于本次观测值。

### 5.4 整帧拒绝

以下情况整帧拒绝，返回 `accepted=false` 和未变化的目标快照：

1. `frame_id` 不大于最近成功帧号；
2. `timestamp_ns` 不大于最近成功帧时间戳；
3. 观测中存在负数编号；
4. 同一帧内编号重复；
5. 观测类型与已创建目标的类型不一致。

## 6. 构建与运行

工程使用 CMake + C++17。当前 VS Code 配置为 WSL 环境，因此请在 WSL 终端中执行：

```bash
cd /mnt/d/Users/Laraine/Desktop/vision-tracker
cmake -S . -B build
cmake --build build -j
./build/tracker
```

也可以直接用 `g++`：

```bash
g++ -std=c++17 -Wall -Wextra -Iinclude \
    src/main.cpp \
    src/TrackingBase/TrackingBase.cpp \
    src/ArmorTarget/ArmorTarget.cpp \
    src/SpinningTarget/SpinningTarget.cpp \
    src/TargetManager/TargetManager.cpp \
    -o build/tracker
```

## 7. 自测结果

程序按 `testFrames()` 顺序处理 28 帧，逐项核对作业第 10 节的验收点，结果全部通过：

| 帧 | 检查点 | 结果 |
| --- | --- | --- |
| 0–1 | 目标 1 速度 `(10,0)`、预测 `(3,2)`；目标 2 角速度 `2.0`、预测角 `0.4` | 通过 |
| 2–5 | 目标 2 未观测时角度每 0.1 s 增加 0.2，预测角为 0.6 / 0.8 / 1.0 / 1.2 | 通过 |
| 6 | 目标 2 第 5 帧丢失进入 `LOST`，位置、角度、速度、预测值全部清零；目标 1 速度 `3.33`、预测 `(5.33,2)` | 通过 |
| 7–8 | 目标 1 预测位置继续推进到 `(5.67,2)`、`(6.00,2)` | 通过 |
| 9 | 目标 1 速度 `3.33`、预测 `(6.33,2)`、`missed_count` 清零 | 通过 |
| 12 | 目标 1 连续丢失 3 帧后进入 `LOST` | 通过 |
| 13 | 目标 1 重新追踪，速度清零，预测位置 `(7,2)` | 通过 |
| 15、17 | 目标 1、3 速度分别为 `(5,0)`、`(10,0)` 和 `(5,0)`、`(5,0)`，预测位置符合预期 | 通过 |
| 16 第一次、19 第一次、15 第二次、23、24 | 五类非法帧全部 `accepted=false`，目标状态、帧号和时间戳记录不变 | 通过 |
| 20 | 目标 1、3、9 连续丢失 3 帧后一起进入 `LOST` | 通过 |
| 21、22 | 目标 2、9 在 `LOST` 后重新追踪，速度清零，预测位置等于当前观测 | 通过 |
| 全部帧 | `tracking/lost/armor/spinning` 计数与 `targets` 中的状态、类型一致 | 通过 |

## 8. 附加项说明

- **附加一（LOST 超时删除）**：未实现。`LOST` 目标会一直保留，以便重新追踪。
- **附加二（类型变化时重建）**：未实现。观测类型与已创建目标不一致时仍按基础要求整帧拒绝。
- **附加三（static）**：业务代码未使用 `static`；固定测试序列内部的函数局部 `static` 属于框架自带代码。
- **附加四（智能指针）**：`TargetManager` 使用 `std::vector<std::unique_ptr<TrackingBase>>` 持有目标，创建时使用 `std::make_unique`。选择 `unique_ptr` 是因为目标所有权唯一、容器析构时自动释放，避免手动 `new` / `delete`；多态销毁依赖基类的虚析构函数。目标不存在共享所有权，因此没有使用 `shared_ptr`。
- **附加五（目标排序）**：未实现。`targets` 按目标创建顺序输出。

## 9. 说明

本项目为基础要求实现。代码结构按“数据契约 / 跟踪基类 / 两个派生目标类 / 目标管理器 / 程序入口”拆分，便于独立阅读和扩展。

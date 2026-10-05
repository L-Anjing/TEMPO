# 本版本 Point-LIO 与初版（上游 Point-LIO）的差异及实测效果

本版本面向**全向轮式底盘 + 高频 MPC 反馈**：需要毫秒级延迟、低噪声的底盘中心位姿，
以及一份与点云同坐标系、可直接喂给 ROGMap 的稠密世界系点云。
初版 Point-LIO 是面向手持/无人机、以“定位精度”为目标设计的，两者取舍不同。
下面逐项列出行为差异、实现位置和实测三效果。

---

## 1. 状态更新：逐点 ESIKF → 毫秒级 batch 联合更新

| | 初版 | 本版本 |
| --- | --- | --- |
| 更新粒度 | 每个点做一次 ESIKF 更新 | 按点采样时间切 `batch.dt`（默认 1 ms）时间窗，**一个 batch 只做一次联合更新** |
| H/z 组装 | 逐点 | 一个 batch 内所有点联合构造 |
| 参考时刻 | 点自身时刻 | batch 末点时刻 |
| 去畸变 | 整帧级（占位实现） | batch 内按末端时刻补偿：`p_imu(t_end) = Exp(-omg·τ)·(p_imu(t_i) − R_wlᵀ·vel·τ)` |
| 最近邻/平面拟合 | 串行 | `batch.omp=true` 时按点 OpenMP 并行（H/z 组装与滤波数学仍串行） |
| 回退 | — | `batch.enable=false` 或 `batch.dt<=0` 回到初版逐点路径（`pose_update_time_bin_ms`） |

实现：`include/time_batching.h`（切窗）、`laserMapping.cpp`（`deskewBatchInto` / `rebuildBatchPointCaches`）、
`Estimator.cpp`（`h_model_input` / `h_model_output`）、`parameters.cpp`（`mapping.batch.*`）。

**实测**（静止，`taskset -c 4-7`、`nice -n 10`、`OMP_NUM_THREADS=4`、`OMP_WAIT_POLICY=PASSIVE`）：

| 雷达频率 | 20 Hz | 40 Hz | 60 Hz | 100 Hz |
| --- | --- | --- | --- | --- |
| `process_ms` | 19.33 | 11.46 | 7.84 | 4.83 |
| 占帧预算 | 39% | 46% | 47% | 48% |
| 实时率 RTF | 1.000 | 1.000 | 1.000 | 1.000 |

**关键约束**：`batch.omp=true` 必须配 `OMP_WAIT_POLICY=PASSIVE`。OpenMP 默认 ACTIVE 自旋会让工作线程
在并行区之间空转、与跟踪线程抢核，实测 4 核受限下实时率从 `1.000` 掉到 `0.309`。两个 launch 入口用
`additional_env` 强制 PASSIVE，`launch.sh` 也导出；节点检测到非 PASSIVE 会打印告警。

---

## 2. 输出坐标系：初版无归零 → 延迟冻结的输出零点

初版的 `camera_init` 就是 IMU 初始化时的世界系。IMU 初始化只用重力方向定姿态
（`Quaterniond::FromTwoVectors`），**yaw 是任意值**；在它上面直接冻结零点，会把还没被 LiDAR 校正过的
偏置当成零点写进所有对外输出。

本版本：

- 滤波器内部世界系 `W` 从 IMU 初始化起固定，**全程不重写**：状态、协方差、iVox 地图都留在 `W`；
- 等 LiDAR IEKF 跑够 `odometry.zero_delay_sec`（默认 1.5 s）后记录 `T_WB0`，之后所有世界系输出统一用
  `T_OB(t) = (T_WB0)⁻¹ · T_WB(t)`；
- 冻结前 `/aft_mapped_to_init`、`/OdometryHighFreq`、`camera_init→base_link` TF、`/cloud_registered*`
  一律不发布（对外即“无位姿”），避免出现一次性跳变。

实现：`include/output_frame.h`（新增）、`laserMapping.cpp::updateOutputFrame()`。
`/aft_mapped_to_init`、`/OdometryHighFreq`、`/point_lio/odometry_scan`、TF、`/cloud_registered`、
`/cloud_registered_full`、`/path`、`/Laser_map`、PCD 落盘走的是**同一个** `T_OB`。

**实测**（静止，100 Hz，三次运行）：

| 运行 | 冻结瞬间内部位姿（过去会泄漏出去的偏置） | 首个 `/aft_mapped_to_init` | 首个 `/OdometryHighFreq` | 冻结前样例 | TF vs odom |
| --- | --- | --- | --- | --- | --- |
| 1 | `(0.017, −0.531, 0.100)` | 0.7 mm / 0.007° | 4.1 mm / 0.030° | 0 | 0.000 mm / 0.000° |
| 2 | `(0.046, −0.179, −0.122)` | 0.4 mm / 0.013° | 0.3 mm / 0.020° | 0 | 0.000 mm / 0.000° |
| 3 | `(0.015, 0.017, 0.009)` | 0.8 mm / 0.039° | 2.5 mm / 0.132° | 0 | 0.000 mm / 0.000° |

内部零点三次相差 **0.55 m**（y 方向），而对外首帧始终 `≤0.8 mm / ≤0.04°`。
初版在这个测试下起始 yaw 在不同雷达频率间能差 8°、平移差 0.35 m。

---

## 3. 输出位姿：IMU 原点 → 底盘中心（base_link）

初版公开的是 IMU/雷达原点状态。本版本通过 `robot_extrinsic.lidar_to_robot_R/T` 把状态换算到底盘中心：

- `robotKinematics()` 做位置、旋转和**杆臂速度修正**（`ω × r`），线速度在 `base_link` 系表达；
- 静态 TF：`base_link → livox_frame → imu_link`；动态 TF：`camera_init → base_link`；
- 导航 launch 从**同一份** YAML 读外参注入 ROGMap 射线原点，不重复维护。

`lidar_to_robot_T` 必须保持为实测值：把它置零会把 `base_link` 挪到雷达原点，ROGMap 的 footprint
净空圈和车身滤除盒会整体偏 0.32 m（实测直接把起点判成不可通行）。

---

## 4. 高频里程计：初版没有 → `/OdometryHighFreq`

初版只有按雷达帧输出的里程计。本版本增加 `ImuOdometryPredictor`：以最近一次校正后的 posterior
为起点，逐 IMU 样本（MID-360 约 200 Hz）传播，输出 `/OdometryHighFreq`（child frame 独立为
`base_link_hf`，避免与低频位姿在 TF 上打架）。

- `odometry.max_correction_age_sec`（0.3 s）：雷达校正过旧时停止高频输出，不用纯预测掩盖处理积压；
- batch 模式下**每个 batch 更新后**都重新播种 predictor，而不是等整帧结束；
- `/point_lio/odometry_scan` 发布与实际校正时刻一致的位姿，便于对照。

**实测状态龄 P95**：1.19 / 1.23 / 1.08 / 2.13 ms（20/40/60/100 Hz），同在 `.freq_cmp` 条件下
FAST-LIO 是 2.29 / 5.30 / 2.72 / 2.29 ms。

---

## 5. 点云输出：初版 `cloud_registered` → 增加稠密 `cloud_registered_full`

初版的 `/cloud_registered` 是降采样后的定位点云。本版本把职责拆开：

- `/cloud_registered`：降采样世界系点云（定位/RViz 用）；
- `/cloud_registered_full`：**完整帧**去畸变点云，使用独立状态快照 + 按点时间稳定排序，
  以 `SensorDataQoS().keep_last(1)` 发布，作为 ROGMap 的主输入；
- `/cloud_registered_body`：可选，`imu_link` 系（默认关闭）。

---

## 6. 工程与配置

| | 初版 | 本版本 |
| --- | --- | --- |
| 坐标系命名 | 硬编码 `camera_init` / `body` | `frames.{odom,robot,robot_high_frequency,lidar,imu}`，启动时校验非空且互不相同 |
| launch | 含拉起驱动的入口 | 只起估计器；`livox_ros_driver2` 仅作为 `CustomMsg` 编译依赖 |
| 参数组织 | 平铺 | `common` / `preprocess` / `mapping` / `sensor_extrinsic` / `robot_extrinsic` / `frames` / `odometry` / `diagnostics` / `publish` / `pcd_save`，逐参数带行注释 |
| 性能统计 | 基本无 | `RuntimeStatistics`：IMU、状态、逐帧性能 CSV、低频摘要，可整体关闭 |
| 测试 | 无 | 5 个 gtest 套件共 17 个用例：外参/输出零点、发布限频、高频预测器、卡尔曼增益、iVox 最近邻 |

---

## 7. 已知代价与未解决项（选型时要知道）

1. **速度噪声明显大于 FAST-LIO**。静止去均值 RMS：`vx` 17–22 mm/s、`vy` 11–22 mm/s，
   而 FAST-LIO 是 3.4–5.8 / 2.6–3.1 mm/s（`wz` 两者都约 2.0 mrad/s）。
   位置/z 抖动同级（本版本 100 Hz：x 1.1–1.8、y 1.6–2.6、z 4.1–5.2 mm）。
   给 MPC 用时，速度通道需要下游自己滤波或改用 FAST-LIO。
2. **`filter_size_map` 与 iVox 的耦合**：`filter_size_map` 会作为 iVox 的 `point_resolution_`。
   `ivox_grid_resolution 2.0 / filter_size_map 0.1` 时单个体素最多可容纳约 8000 个代表点，
   rosbag 回放实时率只有 0.47；降到与实际点密度匹配后可回到 0.99。改这两个参数要成对看。
3. **`/OdometryHighFreq` 的速度非单调**：20 Hz 时速度噪声最小，位置精度却在 100 Hz 最好。
   频率选择要按下游更在意哪一路来定，建议 20/100 Hz 各复测 2–3 次。
4. **速度通道依赖 IMU 零偏收敛**：启动静止假设与 FAST-LIO 相同，开机后需要约 1.5 s
   （`zero_delay_sec` 在这段时间内不发布输出）。
5. 硬件验证中发现并已修复的两个问题，记录在此以免回归：发布出去的四元数曾是
   `Matrix3d * Quaterniond` 表达式导致的非单位四元数（`|q|=0.8264`）；TF 曾把已变换的 state
   再变换一次，导致 TF 比 odom 多转 2×冻结倾角（14.8°）。两者都有测试断言。

---

## 8. 相关文档

- [README.md](README.md)：当前接口、配置、TF 链与 FAQ。
- [BATCH_LIO_STAGE1.md](BATCH_LIO_STAGE1.md)：batch 更新的第一轮实测数据与结论。
- `../../.freq_cmp/RESULT.md`：20/40/60/100 Hz 下本版本与 FAST-LIO 的对照。
- `../../.zero_check/RESULT.md`：延迟冻结输出零点的实机验证。

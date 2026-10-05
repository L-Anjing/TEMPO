# Batch-LIO 化 Point-LIO —— 第一阶段（实机静止）实验记录

本文件记录 Batch-LIO 改造的实现范围、实机静止对照数据与结论。**尚未做动态验证**，下一步实验方案见文末。

## 1. 实现范围

| 文件 | 内容 |
| --- | --- |
| `src/parameters.h` / `.cpp` | `mapping.batch.{enable,dt,deskew,omp}` 声明、校验、启动日志；`OMP_WAIT_POLICY` 非 PASSIVE 时告警 |
| `src/laserMapping.cpp` | bin 宽度接线；`deskewBatchInto()`；`rebuildBatchPointCaches()`；每个 batch 刷新 `/OdometryHighFreq` 的 posterior；`seedHighFrequencyOdometry(bool publish_scan)` |
| `src/Estimator.h` / `.cpp` | `feats_down_body_raw`；`h_model_input` / `h_model_output` 的 KNN+平面拟合 OpenMP 并行（`reduction` + thread-local） |
| `config/mapping/mid360.yaml` | `mapping.batch.*` 参数块 |
| `launch/mapping_mid360.launch.py`、`navigation/launch/online_navigation.launch.py`、`launch.sh` | 固定 `OMP_WAIT_POLICY=PASSIVE` |

关键路径：时间窗切分沿用 `time_batching.h`（`[t0, t0+dt)`，`dt<=0` 逐时间戳）；一个 batch 一次 `update_iterated_dyn_share_modified()`，`Estimator.cpp` 的循环上界是 `time_seq[k]`（组内点联合构造 H/z）；参考时刻取组末点；去畸变 `p_imu(t_end) = Exp(-omg*tau) * (p_imu(t_i) - R_wl^T * vel * tau)`，写回 LiDAR 系并重建 `pbody_list`/`crossmat_list`，以 `feats_down_body_raw` 为不可变基准保证幂等。

未改动：`filter_size_map=0.1`、`filter_size_surf=0.1`、`point_filter_num=1`、`ivox_grid_resolution=2.0`、`match_s`、`plane_thr`、`lidar_meas_cov`、全部外参、FAST-LIO 源码、MPC。

## 2. 实机静止对照（同一机器人、同一 YAML、CPU 限 4 核、窗口 5–24 s）

| 方法 | Hz | state age P95 (ms) | xyz RMS (mm) | roll RMS | pitch RMS | yaw RMS | vx RMS (mm/s) | vy RMS (mm/s) | wz RMS (mrad/s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Point-LIO：batch off（2 ms bin） | 200.0 | 1.00 | 5.26 | 0.029 | 0.065 | 0.021 | 17.13 | 14.58 | 2.13 |
| Batch-LIO：2 ms + deskew + omp | 200.0 | 1.55 | 5.14 | 0.037 | 0.050 | 0.018 | 14.63 | 11.59 | 2.09 |
| Batch-LIO：1 ms + deskew + omp | 200.1 | 1.63 | **4.87** | 0.030 | **0.050** | **0.016** | **14.18** | **9.32** | 2.08 |
| FAST-LIO `/OdometryHighFreq` | 200.1 | 5.19 | 5.39 | 0.075 | 0.121 | 0.076 | 15.02 | **3.45** | 2.31 |
| FAST-LIO `/Odometry`（100 Hz） | 100.0 | 7.73 | 4.90 | 0.075 | 0.120 | 0.075 | 0\* | 0\* | 0\* |

`*` FAST-LIO `/Odometry` 的 twist 恒为 0，速度只在 `/OdometryHighFreq` 上。

实时率用 state age 斜率测量（避免启动积压污染）：三种 Point-LIO 配置均为 **RTF = 1.000**，稳态 age ≤ 25 ms，无持续积压。

## 3. 计算耗时（`runtime_pos_log_enable=true`，含额外 IO，仅作诊断）

| 阶段（mean ms/帧） | batch off | batch 1 ms |
| --- | ---: | ---: |
| `process_ms` | 7.158 | **4.988** |
| `ekf_update_ms` | 4.936 | **3.494** |
| `ekf_update_max_ms` | 1.700 | **0.909** |
| `pose_update_attempt_count` | 5.31 | 9.15 |
| `pose_update_points_avg` | 107.0 | 61.6 |
| `map_incremental_ms` | 0.021 | 0.022 |
| `preprocess_ms` | 0.081 | 0.095 |
| `lidar_backlog_ms` | 15.62 | **6.78** |
| `ivox_points_per_grid_max` | 54.6（峰值 1225） | 59.4（峰值 1326） |

## 4. 结论

1. `filter_size_map=0.1` 下 Batch-LIO 可以真实实时，state age P95 ≤ 2 ms。
2. 1 ms 优于 2 ms：xyz 4.87 vs 5.14 mm、vy 9.32 vs 11.59 mm/s。
3. 相比原路径降低 position/velocity jitter：vx 17.13→14.18 mm/s、vy 14.58→9.32 mm/s。
4. **`batch.omp=true` 必须配 `OMP_WAIT_POLICY=PASSIVE`**：默认 ACTIVE 自旋在 4 核预算下把 RTF 从 1.000 打到 0.309，表现为 `/OdometryHighFreq` 持续 `withheld`；PASSIVE 后恢复 1.000。
5. batch 机制本身几乎免费（`enable=true, omp=false, deskew=false` 与 batch off 同为 RTF 1.000）。
6. 与 FAST-LIO 100 Hz 相比：位置噪声同量级，Batch-LIO 的 yaw/pitch 与 vx 更好，FAST-LIO 的 vy 更好；Point-LIO 侧 state age 更低。

## 5. 未解决

- `batch.deskew` 只在静止数据上验证（静止时 `tau≈0`，补偿量近零），动态未验证；且只在 `use_imu_as_input=false` 分支实现。
- 启动追赶期偶发 `/aft_mapped_to_init` 与 `/OdometryHighFreq` 以 ~400 Hz 输出（按传感器时间限频，追赶时墙钟速率翻倍），稳态 200 Hz。
- `deskewBatchInto` 用常数 ω/v 一阶近似，未做二阶补偿。

## 6. 复现方法

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 本机 Fast DDS SHM 不可用时
export OMP_WAIT_POLICY=PASSIVE OMP_NUM_THREADS=4
taskset -c 4-7 nice -n 10 \
  ros2 run point_lio pointlio_mapping --ros-args \
    --params-file src/point_lio/config/mapping/mid360.yaml \
    -p mapping.batch.enable:=true -p mapping.batch.dt:=0.001
# 另起订阅端记录 /aft_mapped_to_init 与 /OdometryHighFreq 的
# (header.stamp, 接收时刻, pose, twist)，再按 state age 斜率与去均值 RMS 统计。
```

## 7. 下一步：动态阶跃/加减速实验（未实现）

1. 直线 ±0.2 / ±0.5 m/s 方波各 3 周期；原地 ±0.5 rad/s yaw 方波。
2. 指标：速度阶跃 10%–90% 上升时间、超调、稳态误差；`/OdometryHighFreq` 相对 `/aft_mapped_to_init` 的相位超前；A/B/C 三者对比。
3. 同步记录 `correction_age` 与 `lidar_backlog_ms`，确认动态下不出现积压。
4. 只评价动态延迟，不引入 normalized residual / robust kernel / dynamic covariance。

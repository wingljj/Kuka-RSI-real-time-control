# 通信健壮性验证矩阵

主机侧行为验证工具：
- `tools/verify_robustness.sh` —— krc_simulator 故障注入 + loopback_test 端到端
- `tools/pcap_replay.py` —— 真实抓包回放（非闭环）

## 主机侧故障注入矩阵（本机可跑）

| 场景 | 工具/开关 | 预期 | 判定 |
|---|---|---|---|
| 正常环回 | `verify_robustness.sh` | 700/700 应答、零丢包 | `replies=700 missed=0` |
| 重复帧 | `--ipoc-dup 50` | 回零增量、不推进 lastGood、丢包+1、回显正确 | `missed>0` 且 `ipoc_mismatch=0` |
| 回退帧 | `--ipoc-back 50` | 同上 | 同上 |
| 前向跳号 | `--ipoc-gap 50` | 正常修正、丢包 += 缺口、回显正确 | `missed>0` 且 `ipoc_mismatch=0` |
| 丢包 | `--drop 50` | 主机计丢包 | `missed>0` |
| 乱序 | `--reorder 50` | 按序回显不崩溃 | 无 `ipoc_mismatch` 暴涨 |
| 迟到 | `--late-ms 5` | 回包延迟可测 | simulator `rtt_max_us` 可见 |
| 错误 SENTYPE | `--ignore-replies` | KRC Delay 增长 → 主机 Fault | `state=Fault` |
| 断网 | 运行中杀 simulator / 停监听 | 看门狗置未连接 | 状态栏回 `◐ 监听中` |
| 长时间运行 | `--cycles 10000` | 无泄漏、丢包不漂移 | 长时间窗口 missed 稳定 |

## 真机 T1 验证（待执行）

分别完成 **12 ms** 与 **4 ms** 周期真机 T1 联机后，再评估自动模式。
每档按 [real-machine-deployment.md](real-machine-deployment.md) §5 执行，
并额外核对：
- 实测周期回填 `cycle_ms`（联锁会校验偏差 ≤ 10%）
- KRC `<Delay>` 全程为 0（非 0 即链路异常，联锁会 Fault）
- KRC 侧层 3~5 限值梯度仍单调（主机第 2 层已移除，见 real-machine-deployment.md §4）

## 本轮新增验证项

| 场景 | 工具 | 预期 | 判定 |
|---|---|---|---|
| 目标轨迹 | `test_target_trajectory` 4 用例 + `test_pose_controller` targetTrajectory_* 5 用例 | 端点精确 / quintic 端点零速 / Slerp 中点与最短弧 / 零时长直通 / 阶跃逐步趋近、稳态不变、reset 同步、角跳变走最短路径 | 单测 PASS |
| 异常帧剔除 | `test_pose_ops` exceedsPhysicalJump_* + `verify_kinematics.sh`/`verify_robustness.sh` 正常运动场景 | 跳变超物理极限（位置欧氏距离、姿态 SO(3) 最短角）判陈旧：回零增量、连续 `stale_frame_limit` 帧 Fault；正常运动不触发 | 单测 PASS；正常运动 e2e 无误报（simulator 暂无跳变注入开关） |
| 7 态状态机 | `test_shared_state` state_* 2 用例 + `verify_robustness.sh` | 默认 Disconnected、状态快照往返；e2e `state=Fault`；优先级 Fault > StaleFrame > Syncing > Tracking > Ready > WaitingFirstFrame > Disconnected | 单测 PASS + e2e |
| UI 状态卡 | 手动驱动 GUI | 7 态颜色分级：Fault 红 / StaleFrame 黄 / Tracking+Ready 绿 / Syncing+WaitingFirstFrame 蓝 / Disconnected 灰，另保留丢包/周期偏离黄警告 | 手动 |
| 两阶段使能 | 手动驱动 GUI | 准备→确认→已使能；Fault→归零并复位 | 手动 |

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
- 五层限值梯度仍单调

## 本轮新增验证项

| 场景 | 工具 | 预期 | 判定 |
|---|---|---|---|
| 姿态多圈累计 | `test_pose_controller` rotatedOverLimit_firesViaCommandedSumWhenRistDoesNotFollow | commandedSum 兜底触发 Fault | 单测 PASS |
| 目标平滑 | `test_pose_controller` smoothing_* 5 用例 | 阶跃削平 / τ=0 直通 / reset 同步 / 稳态不变 / 角跳变走最短路径 | 单测 PASS |
| UI 状态卡 | 手动驱动 GUI | 颜色分级 + 诊断字段 | 手动 |
| 两阶段使能 | 手动驱动 GUI | 准备→确认→已使能；Fault→归零并复位 | 手动 |

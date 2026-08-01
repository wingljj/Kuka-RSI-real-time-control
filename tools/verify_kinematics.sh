#!/usr/bin/env bash
# KR210 运动学 + 真实约束端到端验证：用 krc_simulator 的关节模型（正解回报
# RIst/AIPos/ASPos）驱动 loopback_test。覆盖：运动学自检、默认闭环、关节限位、
# 速度限制、会话重启。全部通过返回 0。
# 用法: bash tools/verify_kinematics.sh
# 脚本自动配置 Qt 运行环境；若调用者已设 QTBIN/MINGW/NINJA 则尊重其取值。
set -u
cd "$(dirname "$0")/.."

# 运行环境：默认本项目路径；若调用者已设 QTBIN/MINGW/NINJA 则尊重之。
if [ -z "${QTBIN:-}" ]; then
    export MINGW=/d/Software/QT/content/Tools/mingw1120_64/bin
    export NINJA=/d/Software/QT/content/Tools/Ninja
    export QTBIN=/d/Software/QT/content/6.5.3/mingw_64/bin
    export PATH="$MINGW:$NINJA:$QTBIN:$PATH"
fi

BUILD=build
HOST=127.0.0.1
PORT=59152
CYCLES=400
SECS=6

pass=0
fail=0

# run <tag> <lb-extra> <sim args...>：后台起 loopback_test（主机），前台跑
# krc_simulator。主机必须先绑定 socket 再让 simulator 起跑——simulator 一启动
# 就按周期发帧，若对端尚未 bind，前几帧会被丢弃。lb-extra 是传给 loopback_test
# 的额外参数（如 "--track 10"）。
run() {
    local tag=$1; shift
    local lb=$1; shift
    # shellcheck disable=SC2086
    "$BUILD/tools/loopback_test/loopback_test.exe" --seconds "$SECS" $lb \
        > "/tmp/kx_$tag.log" 2>&1 &
    local host_pid=$!
    sleep 0.2
    "$BUILD/tools/krc_simulator/krc_simulator.exe" --host "$HOST" --port "$PORT" \
        --cycles "$CYCLES" "$@" > "/tmp/kx_${tag}_sim.log" 2>&1
    wait "$host_pid"
}

# check <描述> <tag> <grep 表达式>：在 simulator 日志里断言。
check() {
    local desc=$1 tag=$2 pat=$3
    if grep -q "$pat" "/tmp/kx_${tag}_sim.log"; then
        echo "PASS  $desc"
        pass=$((pass + 1))
    else
        echo "FAIL  $desc  (grep '$pat' 于 /tmp/kx_${tag}_sim.log)"
        fail=$((fail + 1))
    fi
}

# check_lb <描述> <tag> <grep 表达式>：在 loopback_test（主机）日志里断言。
check_lb() {
    local desc=$1 tag=$2 pat=$3
    if grep -q "$pat" "/tmp/kx_$tag.log"; then
        echo "PASS  $desc"
        pass=$((pass + 1))
    else
        echo "FAIL  $desc  (grep '$pat' 于 /tmp/kx_$tag.log)"
        fail=$((fail + 1))
    fi
}

# err_final <tag>：取 loopback 最终快照（[final]）的 err X 数值（含符号）。
err_final() {
    grep "\[final\]" "/tmp/kx_$1.log" \
        | grep -o 'err X=[-0-9.]*' | head -1 | cut -d= -f2
}

# 1. 运动学自检：正解与 DH 已知位形一致。
"$BUILD/tools/krc_simulator/krc_simulator.exe" --self-test > "/tmp/kx_selftest.log" 2>&1
if grep -q "self-test OK" "/tmp/kx_selftest.log"; then
    echo "PASS  运动学自检（正解与 DH 已知位形一致）"
    pass=$((pass + 1))
else
    echo "FAIL  运动学自检"
    fail=$((fail + 1))
fi

# 2. 默认（无约束）闭环：400/400 应答、零丢包、零 IPOC 不匹配，位姿由正解回报。
run normal ""
check "默认闭环 400/400 零丢包零不匹配" normal "replies=$CYCLES missed=0 ipoc_mismatch=0"
check_lb "默认闭环主机收满帧" normal "frames=$CYCLES"

# 3. 关节限位：所有关节锁死在初始位形（相当于物理挡块把机器人顶住），主机给
#    X 目标 → 关节 clamp 挡住修正，位姿不再前移，err X 保持非零不收敛；位移
#    =0 不触发 POSCORR 限位，主机保持 Tracking（不 Fault）。
#    注：单关节"靠近限位"挡不住 X——A1 量程 ±185°，伪逆可绕行，误差仍会收敛；
#    把限位带宽收敛为 0 才能确定性地验证"限位挡住机器人"。
run joint "--track 10" \
    --init-joints "0 -60 30 0 90 0" \
    --joint-limits "0 0 -60 -60 30 30 0 0 90 90 0 0"
JERR=$(err_final joint)
if [ -n "$JERR" ] && awk -v v="$JERR" 'BEGIN{exit !(v!=0)}'; then
    echo "PASS  关节限位下误差不收敛（err X=$JERR，限位挡住机器人）"
    pass=$((pass + 1))
else
    echo "FAIL  关节限位下误差仍收敛为 0（限位未生效？err X=${JERR:-?}）"
    fail=$((fail + 1))
fi
check_lb "关节限位下主机保持 Tracking 不 Fault" joint "state=Tracking"

# 4. 速度限制：主机给 X 目标 → 模拟器限速响应（慢而不破）→ 误差最终收敛为 0，
#    主机保持 Tracking。
run vel "--track 20" \
    --max-vel-pos 20 --max-accel-pos 2000 --max-vel-rot 5 --max-accel-rot 500
VERR=$(err_final vel)
if [ -n "$VERR" ] && awk -v v="$VERR" 'BEGIN{exit !(v<0.001 && v>-0.001)}'; then
    echo "PASS  速度限制下误差收敛（err X=$VERR，限速不影响收敛）"
    pass=$((pass + 1))
else
    echo "FAIL  速度限制下误差未收敛（err X=${VERR:-?}）"
    fail=$((fail + 1))
fi
check_lb "速度限制下主机保持 Tracking" vel "state=Tracking"

# 5. 会话重启：模拟器中断（KRL 重启，IPOC/q 复位）→ 主机重新建立会话，回显
#    正确。注意：gap 期间模拟器直接跳过回复等待，所以日志表现是 replies<cycles
#    （如 replies=316<400）、missed 保持 0——不是 missed>0。这也正是 host 端
#    看到的帧数变少的来源。
run restart "" --restart-at-ms 2000 --restart-gap-ms 1000
check "会话重启后回显正确" restart "ipoc_mismatch=0"
check "会话重启确实发生（gap 后会话复位）" restart "session resumed"
REP=$(grep -o 'replies=[0-9]*' "/tmp/kx_restart_sim.log" | head -1 | cut -d= -f2)
if [ -n "$REP" ] && [ "$REP" -lt "$CYCLES" ]; then
    echo "PASS  会话重启 gap 期间停发（replies=$REP < $CYCLES）"
    pass=$((pass + 1))
else
    echo "FAIL  会话重启未造成停发（replies=${REP:-?} vs cycles=$CYCLES）"
    fail=$((fail + 1))
fi

echo "----"
echo "PASS=$pass FAIL=$fail"
[ "$fail" -eq 0 ]

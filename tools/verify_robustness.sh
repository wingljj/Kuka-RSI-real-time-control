#!/usr/bin/env bash
# 通信健壮性端到端验证：用 krc_simulator 故障注入驱动 loopback_test。
# 用法: bash tools/verify_robustness.sh
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
CYCLES=700
SECS=10

pass=0
fail=0

# run <tag> <lb-extra> <sim args...>：后台起 loopback_test（主机），前台跑
# krc_simulator。主机必须先绑定 socket 再让 simulator 起跑——simulator 一启动
# 就按周期发帧，若对端尚未 bind，前几帧会被丢弃（Task 2 实测不稳）。
# lb-extra 是传给 loopback_test 的额外参数（如 "--track 1"）。
run() {
    local tag=$1; shift
    local lb=$1; shift
    # shellcheck disable=SC2086
    "$BUILD/tools/loopback_test/loopback_test.exe" --seconds "$SECS" $lb \
        > "/tmp/lb_$tag.log" 2>&1 &
    local host_pid=$!
    sleep 0.2
    "$BUILD/tools/krc_simulator/krc_simulator.exe" --host "$HOST" --port "$PORT" \
        --cycles "$CYCLES" "$@" > "/tmp/sim_$tag.log" 2>&1
    local rc=$?
    wait "$host_pid" || rc=1
    return "$rc"
}

# check <描述> <tag> <grep 表达式>：在 simulator 日志里断言。
check() {
    local desc=$1 tag=$2 pat=$3
    if grep -q "$pat" "/tmp/sim_$tag.log"; then
        echo "PASS  $desc"
        pass=$((pass + 1))
    else
        echo "FAIL  $desc  (grep '$pat' 于 /tmp/sim_$tag.log)"
        fail=$((fail + 1))
    fi
}

# check_lb <描述> <tag> <grep 表达式>：在 loopback_test（主机）日志里断言。
check_lb() {
    local desc=$1 tag=$2 pat=$3
    if grep -q "$pat" "/tmp/lb_$tag.log"; then
        echo "PASS  $desc"
        pass=$((pass + 1))
    else
        echo "FAIL  $desc  (grep '$pat' 于 /tmp/lb_$tag.log)"
        fail=$((fail + 1))
    fi
}

# 1. 正常环回：700/700 应答、零丢包、零 IPOC 不匹配
run normal ""
check "正常环回 700/700 零丢包" normal "replies=$CYCLES missed=0 ipoc_mismatch=0"

# 2. --ipoc-dup：重复帧仍被原样回显。主机每帧必回（重复帧也回），且把重复帧
#    计入自身丢包计数；端到端可断言的是：主机收满全部帧、回显零不匹配。
run dup "" --ipoc-dup 50
check "重复帧回显正确" dup "ipoc_mismatch=0"
check "重复帧不导致丢帧（主机收满）" dup "replies=$CYCLES"

# 3. --drop 50：主机看到丢包且回显仍正确（simulator 收不到被丢帧的回包，
#    故 simulator 侧 missed 非零）
run drop "" --drop 50
check "丢包计入" drop "missed=[1-9][0-9]*"
check "丢包后回显正确" drop "ipoc_mismatch=0"

# 4. --ipoc-gap 50：前向跳号。主机对跳号帧仍原样回显、收满全部帧。
run gap "" --ipoc-gap 50
check "跳号帧回显正确" gap "ipoc_mismatch=0"
check "跳号后主机收满不丢帧" gap "replies=$CYCLES"

# 5. --ignore-replies：模拟 SENTYPE 错配。必须使能跟踪（--track 1）——
#    KRC Delay 增长是运行中保护，只在 Tracking 状态触发 Fault。
run ignore "--track 1" --ignore-replies
check_lb "主机 KRC Delay 增长 → Fault" ignore "state=Fault"

echo "----"
echo "PASS=$pass FAIL=$fail"
[ "$fail" -eq 0 ]

#!/usr/bin/env python3
"""诊断探针：测量 KRC（或模拟器）的发帧间隔，并可注入指定大小的 RKorr 增量。

用途是回答一个具体问题——「主机侧看到的断流，是不是由机器人开始运动引起的」。
它替代 rsi_host 充当上位机：收帧、立即回包、记录每两帧之间的墙钟间隔，
最后打印分布与所有超过阈值的间隔。

  # 先起探针再起模拟器；探针要先占住端口
  python tools/probe_cadence.py --port 59152 --seconds 20 --step 0.6 &
  sleep 0.5
  ./build/tools/krc_simulator/krc_simulator.exe \
      --host 127.0.0.1 --port 59152 --cycles 1500

--step 是每帧发出的 X 增量（mm）。0 表示机器人不动（对照组），
0.6 是 12ms 周期下 50mm/s 的满额增量（实验组）。

两个容易踩的坑：

* 起模拟器前只 sleep 0.5，不要 sleep 2 —— 下面的 recvfrom 超时是 2 秒，
  等太久探针会先判「无帧」退出，一个样本都采不到。
* 模拟器要传**有限**的 --cycles。`--cycles 0` 是无限模式，永远不打印
  final_pose，而那一行是判断「机器人到底动没动」的唯一依据——只看间隔
  分布的话，一个把 IK 全部解失败、机器人卡死不动的版本反而最「干净」。

跑完记得确认没有残留进程占着端口：
  powershell -Command "Get-Process | Where-Object {$_.ProcessName -match 'krc_simulator'}"
"""
import argparse
import re
import socket
import time

IPOC_RE = re.compile(rb"<IPOC>(\d+)</IPOC>")


def build_sen(dx, ipoc, sen_type="ImFree"):
    return (
        f'<Sen Type="{sen_type}">'
        f'<RKorr X="{dx:.4f}" Y="0.0000" Z="0.0000" '
        f'A="0.0000" B="0.0000" C="0.0000"/>'
        f"<IPOC>{ipoc}</IPOC></Sen>"
    ).encode()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=59152)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--step", type=float, default=0.0,
                    help="每帧回发的 X 增量 mm（0 = 机器人不动）")
    ap.add_argument("--gap-ms", type=float, default=240.0,
                    help="超过该间隔即视为断流（默认与主机看门狗一致）")
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    s.bind((args.bind, args.port))
    s.settimeout(2.0)
    print(f"listening {args.bind}:{args.port}  step={args.step} mm/frame")

    gaps = []
    prev = None
    frames = 0
    bad = 0
    t0 = time.perf_counter()

    while time.perf_counter() - t0 < args.seconds:
        try:
            data, peer = s.recvfrom(4096)
        except socket.timeout:
            print("!! 2s 内无帧，对端已停发")
            break
        now = time.perf_counter()
        if prev is not None:
            gaps.append((now - prev) * 1000.0)
        prev = now
        frames += 1

        m = IPOC_RE.search(data)
        if not m:
            bad += 1
            continue
        s.sendto(build_sen(args.step, int(m.group(1))), peer)

    if not gaps:
        print("没有采到间隔样本")
        return

    gaps.sort()
    n = len(gaps)
    over = [g for g in gaps if g >= args.gap_ms]
    print(f"frames={frames} unparsable={bad}")
    print(f"间隔 ms: min={gaps[0]:.2f} p50={gaps[n // 2]:.2f} "
          f"p99={gaps[min(n - 1, int(n * 0.99))]:.2f} max={gaps[-1]:.2f}")
    print(f"超过 {args.gap_ms}ms 的间隔: {len(over)} 个")
    for g in over[-10:]:
        print(f"   {g:.1f} ms")


if __name__ == "__main__":
    main()

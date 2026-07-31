#!/usr/bin/env python3
"""RSI 帧捕获 —— 判断 KRC 到底有没有把帧发到宿主，以及来自哪个源 IP。

用法：
  1. 在 rsi_host 界面点「停止监听」（不关程序，释放 59152 端口）
  2. 双击 tools/网络抓包.bat（或直接运行本脚本）
  3. 到示教器启动 PoseTrack，等到报错
  4. 回到本窗口看打印；Ctrl+C 退出
  5. 回 rsi_host 重新「开始监听」

只收不发：KRC 会在约 1 秒（Timeout x 12ms）后因无应答超时报错，
这正是我们要观察的窗口 —— 窗口里有没有帧、来自哪个 IP，一眼定音。

结果判读：
  有帧，来自 192.168.44.147  -> 网络配置生效，帧到了宿主，问题在应答侧
  有帧，来自其它地址         -> 配置生效但在另一个网段，帧绕了路
  0 帧                      -> KRC 根本没把帧发到这块网卡，配置未生效
"""
import socket
import sys
import os
import datetime

HOST = sys.argv[2] if len(sys.argv) > 2 else "192.168.44.1"
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 59152


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind((HOST, PORT))
    except OSError as e:
        print("绑定 %s:%d 失败：%s" % (HOST, PORT, e))
        if "in use" in str(e).lower() or "10048" in str(e):
            print(">>> 端口被占用 —— 请先在 rsi_host 里点「停止监听」，再重新运行")
        sys.exit(1)

    print("已监听 %s:%d 。现在去示教器启动 PoseTrack。\n" % (HOST, PORT))
    print("（本脚本只接收不回复；KRC 会因超时报错停止 —— 属正常）\n")
    print("-" * 70)

    logpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rsi_capture.log")
    logf = open(logpath, "wb")
    n = 0
    s.settimeout(1.0)
    try:
        while True:
            try:
                data, addr = s.recvfrom(65535)
            except socket.timeout:
                continue
            n += 1
            t = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            line = "[%s] 第 %d 帧  来自 %s  长度 %d" % (t, n, addr[0], len(data))
            print(line)
            try:
                print("   " + data.decode("ascii").replace("\n", "\n    "))
            except UnicodeDecodeError:
                print("    非 ASCII:", data.hex())
            print()
            logf.write((line + "\n").encode("utf-8", "replace"))
            logf.write(data + b"\n\n")
            logf.flush()
    except KeyboardInterrupt:
        print("\n共收到 %d 帧，原始内容已存 %s" % (n, logpath))
        if n == 0:
            print("（0 帧：KRC 没把帧发到这里 —— 网络配置未生效，或配到了别的网卡）")


if __name__ == "__main__":
    main()

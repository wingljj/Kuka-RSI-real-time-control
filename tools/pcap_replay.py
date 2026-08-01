#!/usr/bin/env python3
"""重放 pcap 中的 <Rob> 帧到指定 IP:端口，对 rsi_host 做真实抓包序列验证。

非闭环：不回包。支持 --speed 加速、--ipoc-shift 改写 IPOC、--drop 按比例
丢帧、--loop 循环回放（每圈 IPOC 整体上移避免重复）。

用法:
  python pcap_replay.py capture.pcap --host 127.0.0.1 --port 59152
"""
import argparse
import socket
import struct
import sys
import time

ETH_P_IP = 0x0800
ETH_P_8021Q = 0x8100
UDP_PROTO = 17


def parse_pcap(path, target_port):
    """返回 [(relative_ts_sec: float, payload: bytes), ...]，payload 为 UDP 载荷。"""
    frames = []
    with open(path, "rb") as f:
        gh = f.read(24)
        if len(gh) < 24:
            raise ValueError("pcap 全局头不足 24 字节")
        raw = gh[:4]
        magic_le = struct.unpack("<I", raw)[0]
        # 小端文件字节为 D4 C3 B2 A1 → <I 读得 0xA1B2C3D4；大端文件字节为
        # A1 B2 C3 D4 → <I 读得 0xD4C3B2A1。据此推断真实字节序。
        if magic_le in (0xA1B2C3D4, 0xA1B23C4D):
            endian = "<"
        else:
            endian = ">"
        magic = struct.unpack(endian + "I", raw)[0]
        if magic not in (0xA1B2C3D4, 0xA1B23C4D):
            raise ValueError("未知 pcap magic %#x" % magic)
        # 0xA1B23C4D = 纳秒时间戳；0xA1B2C3D4 = 微秒
        subsecond_divisor = 1e9 if magic == 0xA1B23C4D else 1e6
        linktype = struct.unpack(endian + "I", gh[20:24])[0]
        if linktype != 1:
            raise ValueError("仅支持 Ethernet(1) 链路层，得到 %d" % linktype)

        t0 = None
        while True:
            rec = f.read(16)
            if not rec:
                break
            ts_sec, ts_usec, incl_len, _ = struct.unpack(endian + "IIII", rec)
            data = f.read(incl_len)
            if len(data) < incl_len:
                break
            t = ts_sec + ts_usec / subsecond_divisor
            if t0 is None:
                t0 = t
            payload = extract_udp_payload(data, target_port)
            if payload is not None:
                frames.append((t - t0, payload))
    return frames


def extract_udp_payload(eth, target_port):
    """从以太网帧提取 UDP 载荷；目标端口不匹配或非 UDP 返回 None。"""
    if len(eth) < 14:
        return None
    ethertype = struct.unpack(">H", eth[12:14])[0]
    off = 14
    if ethertype == ETH_P_8021Q:
        if len(eth) < 18:
            return None
        ethertype = struct.unpack(">H", eth[16:18])[0]
        off = 18
    if ethertype != ETH_P_IP:
        return None
    ip = eth[off:]
    if len(ip) < 20:
        return None
    ihl = (ip[0] & 0x0F) * 4
    if ip[9] != UDP_PROTO:
        return None
    udp = ip[ihl:]
    if len(udp) < 8:
        return None
    _, dport, _, _ = struct.unpack(">HHHH", udp[:8])
    if dport != target_port:
        return None
    return udp[8:]


def rewrite_ipoc(payload, shift):
    """把 <IPOC>n</IPOC> 改写为 <IPOC>n+shift</IPOC>；不改写返回原样。"""
    if not shift:
        return payload
    head, sep, tail = payload.partition(b"<IPOC>")
    if not sep:
        return payload
    num, _, rest = tail.partition(b"</IPOC>")
    try:
        v = int(num) + shift
    except ValueError:
        return payload
    return head + sep + str(v).encode() + b"</IPOC>" + rest


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pcap", help="输入 pcap 文件")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=59152)
    ap.add_argument("--speed", type=float, default=1.0, help="回放加速倍率")
    ap.add_argument("--ipoc-shift", type=int, default=0, help="改写 IPOC：加 shift")
    ap.add_argument("--drop", type=int, default=0, help="每 N 帧丢 1 帧")
    ap.add_argument("--loop", action="store_true", help="循环回放")
    args = ap.parse_args()
    if args.speed <= 0:
        ap.error("--speed must be > 0")

    frames = parse_pcap(args.pcap, args.port)
    if not frames:
        sys.exit("pcap 中没有发往端口 %d 的 UDP 帧: %s" % (args.port, args.pcap))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    sent = 0
    loop_shift = 0
    try:
        while True:
            prev_t = 0.0   # 每圈重置时钟基准，避免圈间 delta 为负
            for k, (t, payload) in enumerate(frames):
                delta = t - prev_t
                prev_t = t
                if delta > 0:
                    time.sleep(delta / args.speed)
                if args.drop and k > 0 and k % args.drop == 0:
                    continue
                payload = rewrite_ipoc(payload, args.ipoc_shift + loop_shift)
                sock.sendto(payload, (args.host, args.port))
                sent += 1
            if not args.loop:
                break
            loop_shift += 1000   # 每圈 IPOC 整体上移，避免圈间重复
    except KeyboardInterrupt:
        pass
    print("sent %d frames from %s" % (sent, args.pcap))


if __name__ == "__main__":
    main()

// 假 KRC：按固定周期发 <Rob>、收 <Sen>，验证上位机的实时行为。
// 把收到的 RKorr 累加到自身位姿，模拟 RELATIVE 修正语义。
// 支持故障注入：--ipoc-dup/--ipoc-gap/--ipoc-back/--drop/--reorder/--late-ms
// /--ignore-replies/--send-delay，用于验证主机的异常处理路径。
#include <QCoreApplication>
#include <QCommandLineParser>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QThread>
#include <QUdpSocket>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include "core/Pose.h"
#include "core/RsiCodec.h"

namespace {

QByteArray buildRob(const Pose &p, quint64 ipoc, quint64 delay)
{
    QByteArray s;
    s.reserve(320);
    s += "<Rob Type=\"KUKA\">\n<RIst";
    const char *k[6] = {" X=\"", " Y=\"", " Z=\"",
                        " A=\"", " B=\"", " C=\""};
    const double v[6] = {p.x, p.y, p.z, p.a, p.b, p.c};
    for (int i = 0; i < 6; ++i) {
        s += k[i];
        s += QByteArray::number(v[i], 'f', 4);
        s += '"';
    }
    s += "/>\n<RSol";
    for (int i = 0; i < 6; ++i) {
        s += k[i];
        s += QByteArray::number(v[i], 'f', 4);
        s += '"';
    }
    s += "/>\n<Delay D=\"";
    s += QByteArray::number(delay);
    s += "\"/>\n<IPOC>";
    s += QByteArray::number(ipoc);
    s += "</IPOC>\n</Rob>";
    return s;
}

// 从 <Sen> 中取出 RKorr 与 IPOC。
// 注意不能用 RsiCodec::parseRob —— <Sen> 与 <Rob> 结构不同，这里定向提取。
bool parseSen(const QByteArray &d, Pose *korr, quint64 *ipoc)
{
    const int rk = d.indexOf("<RKorr");
    const int ip = d.indexOf("<IPOC>");
    if (rk < 0 || ip < 0)
        return false;

    const char *keys[6] = {"X=\"", "Y=\"", "Z=\"",
                           "A=\"", "B=\"", "C=\""};
    double *dst[6] = {&korr->x, &korr->y, &korr->z,
                      &korr->a, &korr->b, &korr->c};
    for (int i = 0; i < 6; ++i) {
        const int at = d.indexOf(keys[i], rk);
        if (at < 0)
            return false;
        const int b = at + int(qstrlen(keys[i]));
        const int e = d.indexOf('"', b);
        if (e < 0)
            return false;
        bool ok = false;
        *dst[i] = d.mid(b, e - b).toDouble(&ok);
        if (!ok)
            return false;
    }

    const int b = ip + 6;
    const int e = d.indexOf("</IPOC>", b);
    if (e < 0)
        return false;
    bool ok = false;
    *ipoc = d.mid(b, e - b).trimmed().toULongLong(&ok);
    return ok;
}

// 注入开关：everyN > 0 且第 i 周期触发（i>0 避开首帧，保证对端锁定正常）。
bool active(int everyN, int i)
{
    return everyN > 0 && i > 0 && (i % everyN == 0);
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QCommandLineParser p;
    p.addHelpOption();
    QCommandLineOption oHost("host", "host IP", "ip", "192.168.44.1");
    QCommandLineOption oPort("port", "host port", "n", "59152");
    QCommandLineOption oCycle("cycle-ms", "cycle", "ms", "12.0");
    QCommandLineOption oCount("cycles", "cycle count", "n", "500");
    QCommandLineOption oDup("ipoc-dup", "every Nth frame resend previous IPOC", "n", "0");
    QCommandLineOption oGap("ipoc-gap", "every Nth frame jump N IPOC", "n", "0");
    QCommandLineOption oBack("ipoc-back", "every Nth frame send IPOC-1", "n", "0");
    QCommandLineOption oDrop("drop", "every Nth frame send nothing", "n", "0");
    QCommandLineOption oReorder("reorder", "every Nth frame swap order with next", "n", "0");
    QCommandLineOption oLate("late-ms", "every Nth frame delay N ms", "n", "0");
    QCommandLineOption oIgnore("ignore-replies", "drop replies and raise Delay", "");
    QCommandLineOption oDelay("send-delay", "fixed Delay value in every frame", "n", "0");
    p.addOptions({oHost, oPort, oCycle, oCount, oDup, oGap, oBack,
                  oDrop, oReorder, oLate, oIgnore, oDelay});
    p.process(app);

    const QHostAddress host(p.value(oHost));
    const quint16 port  = quint16(p.value(oPort).toUShort());
    const double cycleMs = p.value(oCycle).toDouble();
    const int cycles     = p.value(oCount).toInt();

    const int dupN     = p.value(oDup).toInt();
    const int gapN     = p.value(oGap).toInt();
    const int backN    = p.value(oBack).toInt();
    const int dropN    = p.value(oDrop).toInt();
    const int reorderN = p.value(oReorder).toInt();
    const int lateN    = p.value(oLate).toInt();
    const bool ignore  = p.isSet(oIgnore);
    const quint64 delayBase = p.value(oDelay).toULongLong();

    const bool injected = dupN > 0 || gapN > 0 || backN > 0 || dropN > 0
                          || reorderN > 0 || lateN > 0 || ignore;

    QUdpSocket sock;
    if (!sock.bind(QHostAddress::AnyIPv4, 0)) {
        std::fprintf(stderr, "simulator bind failed: %s\n",
                     qPrintable(sock.errorString()));
        return 2;
    }

    Pose pose{1250.0, 0.0, 1000.0, 0.0, 90.0, 0.0};
    quint64 ipoc = 1000;
    quint64 lastSent = 0;
    quint64 delay    = delayBase;

    int replies = 0, ipocMismatch = 0, missed = 0;
    double maxRttUs = 0.0, sumRttUs = 0.0;

    // 真实 KRC 按固定节拍发帧。若不设节拍而是收到回复就立刻发下一帧，
    // 面对快速主机会全速空转——那测的是吞吐，不是"能否在周期内回复"，
    // 而且主机侧实测出来的周期也不再是 cycleMs。
    QElapsedTimer pace;
    pace.start();

    QByteArray heldRob;          // --reorder 缓冲的上周期帧
    quint64    heldIpoc = 0;
    bool       heldValid = false;

    // 发出的帧按序入队，收到回包弹队首比对 IPOC 回显。UDP 保序，乱序/重复
    // 注入下仍能精确判定主机是否原样回显。--ignore-replies 时不消费，故不入队。
    std::deque<quint64> sentIpocs;

    for (int i = 0; i < cycles; ++i) {
        // 等到本周期的标称发送时刻
        const qint64 dueNs = qint64(double(i) * cycleMs * 1.0e6);
        while (pace.nsecsElapsed() < dueNs) {
            const qint64 remainMs = (dueNs - pace.nsecsElapsed()) / 1000000;
            if (remainMs > 1)
                QThread::msleep(1);
        }

        const bool dup     = active(dupN, i);
        const bool gap     = active(gapN, i);
        const bool back    = active(backN, i);
        const bool drop    = active(dropN, i);
        const bool reorder = active(reorderN, i);
        const bool late    = active(lateN, i);

        // 决定本帧 IPOC：dup 重发上一帧；back 回退；gap 前向跳号。
        quint64 sendIpoc = ipoc;
        if (dup)         sendIpoc = lastSent;
        else if (back)   sendIpoc = (lastSent > 0) ? lastSent - 1 : 0;
        else if (gap)    sendIpoc = ipoc + gapN;

        const QByteArray rob = buildRob(pose, sendIpoc, delay);

        auto sendFrame = [&](const QByteArray &rob2, quint64 frameIpoc) {
            if (late)
                QThread::msleep(lateN);
            sock.writeDatagram(rob2, host, port);
            if (!ignore)
                sentIpocs.push_back(frameIpoc);
        };

        if (reorder) {
            // 本帧缓冲，下周期后发（真乱序）
            heldRob   = rob;
            heldIpoc  = sendIpoc;
            heldValid = true;
        } else {
            if (!drop)
                sendFrame(rob, sendIpoc);          // 本帧先发
            if (heldValid) {
                sendFrame(heldRob, heldIpoc);      // 缓冲帧后发（真乱序）
                heldValid = false;
            }
        }

        // 推进序列：dup 不推进（下帧还发同一个）；其余推进。
        if (!dup) {
            lastSent = sendIpoc;
            ipoc     = sendIpoc + 1;
        }

        QElapsedTimer rtt;
        rtt.start();
        // 等待本周期内的回包。注意不能用 waitForReadyRead 的返回值当作
        // "收到回复"：端口关闭时 Windows 的 ICMP port-unreachable 也会让它
        // 返回 true，那样该周期既不计 replies 也不计 timeouts，
        // cycles == replies + missed 就不再成立。只认解析成功的 <Sen>。
        if (!ignore) {
            const int budgetMs = std::max(1, int(cycleMs));
            bool got = false;
            if (sock.waitForReadyRead(budgetMs)) {
                while (sock.hasPendingDatagrams()) {
                    const QByteArray d = sock.receiveDatagram().data();
                    Pose korr;
                    quint64 echoed = 0;
                    if (parseSen(d, &korr, &echoed)) {
                        ++replies;
                        got = true;
                        if (!sentIpocs.empty()) {
                            if (echoed != sentIpocs.front())
                                ++ipocMismatch;
                            sentIpocs.pop_front();
                        }
                        // RELATIVE：增量累加到当前位姿
                        pose.x += korr.x; pose.y += korr.y; pose.z += korr.z;
                        pose.a = wrap180(pose.a + korr.a);
                        pose.b = wrap180(pose.b + korr.b);
                        pose.c = wrap180(pose.c + korr.c);
                    }
                }
            }
            if (got) {
                const double us = rtt.nsecsElapsed() / 1000.0;
                maxRttUs = std::max(maxRttUs, us);
                sumRttUs += us;
            } else {
                ++missed;
            }
        } else {
            // 模拟 SENTYPE 错配：KRC 静默丢弃每一帧回包，主机毫无察觉，
            // 只有 KRC 自己的 Delay 计数在涨。这里递增 delay，触发主机
            // 的"KRC Delay 连续 3 帧递增 → Fault"运行中保护。
            ++missed;
            delay += 1;
        }
    }

    std::printf("cycles=%d replies=%d missed=%d ipoc_mismatch=%d delay=%llu\n",
                cycles, replies, missed, ipocMismatch,
                static_cast<unsigned long long>(delay));
    std::printf("rtt_avg_us=%.1f rtt_max_us=%.1f\n",
                replies ? sumRttUs / replies : 0.0, maxRttUs);
    std::printf("final_pose X=%.3f Y=%.3f Z=%.3f A=%.3f B=%.3f C=%.3f\n",
                pose.x, pose.y, pose.z, pose.a, pose.b, pose.c);

    // 主机必须始终原样回显 IPOC（即使对异常帧回零增量）。无注入时还要求
    // 每帧都回包；有注入时丢包是预期行为，只查回显正确。
    const bool pass = ipocMismatch == 0 && (injected || replies == cycles);
    std::printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

// 假 KRC：按固定周期发 <Rob>、收 <Sen>，验证上位机的实时行为。
// 关节模型：q[6] 状态，收到的 RKorr（笛卡尔增量）经 limitCartDelta 限制 +
// 雅可比伪逆 solveDelta 转关节增量并 clamp 到限位，forward 正解回报 RIst/AIPos。
// 支持故障注入：--ipoc-dup/--ipoc-gap/--ipoc-back/--drop/--reorder/--late-ms
// /--ignore-replies/--send-delay，用于验证主机的异常处理路径。
// 会话/时序增强：--restart-at-ms/--restart-gap-ms（KRL 重启：停发 + IPOC/q
// 复位）、--ipoc-wrap-at（IPOC 回绕）、--jitter-us（节拍抖动）。
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
#include <cstdlib>
#include <deque>
#include <QStringList>
#include "core/Pose.h"
#include "core/RsiCodec.h"
#include "tools/krc_simulator/kinematics.h"

namespace {

QByteArray buildRob(const Pose &p, quint64 ipoc, quint64 delay, const double *q)
{
    QByteArray s;
    s.reserve(480);
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
    s += "</IPOC>";
    // 关节角（rad → 度）。真实 KRC 的 ethernet.xml 会请求 AIPos/ASPos。
    s += "\n<AIPos";
    for (int i = 0; i < 6; ++i) {
        s += " A";
        s += QByteArray::number(i + 1);
        s += "=\"";
        s += QByteArray::number(q[i] * 180.0 / M_PI, 'f', 4);
        s += '"';
    }
    s += "/>\n<ASPos";
    for (int i = 0; i < 6; ++i) {
        s += " A";
        s += QByteArray::number(i + 1);
        s += "=\"";
        s += QByteArray::number(q[i] * 180.0 / M_PI, 'f', 4);
        s += '"';
    }
    s += "/>\n</Rob>";
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
    QCommandLineOption oRestartAt("restart-at-ms", "stop sending N ms after launch (0=off)", "ms", "0");
    QCommandLineOption oRestartGap("restart-gap-ms", "hold the pause N ms before resuming with IPOC/q reset", "ms", "0");
    QCommandLineOption oIpocWrap("ipoc-wrap-at", "wrap IPOC counter back to 0 when it reaches N (0=off)", "n", "0");
    QCommandLineOption oJitter("jitter-us", "add ±N us random jitter to send cadence (0=off)", "us", "0");
    QCommandLineOption oInitJoints("init-joints", "initial joint angles deg \"q1..q6\"", "q", "");
    QCommandLineOption oJointLimits("joint-limits", "override joint limits deg \"min1 max1 ... min6 max6\"", "s", "");
    QCommandLineOption oCartLimits("cart-limits", "cartesian pose ranges \"xmin xmax ... cmin cmax\"", "s", "");
    QCommandLineOption oMaxVelPos("max-vel-pos", "cartesian position velocity limit mm/s (0=off)", "mm/s", "0");
    QCommandLineOption oMaxVelRot("max-vel-rot", "cartesian rotation velocity limit deg/s (0=off)", "deg/s", "0");
    QCommandLineOption oMaxAccelPos("max-accel-pos", "cartesian position acceleration limit mm/s2 (0=off)", "mm/s2", "0");
    QCommandLineOption oMaxAccelRot("max-accel-rot", "cartesian rotation acceleration limit deg/s2 (0=off)", "deg/s2", "0");
    QCommandLineOption oSelfTest("self-test", "run forward-kinematics self-test and exit", "");
    p.addOptions({oHost, oPort, oCycle, oCount, oDup, oGap, oBack,
                  oDrop, oReorder, oLate, oIgnore, oDelay,
                  oRestartAt, oRestartGap, oIpocWrap, oJitter,
                  oInitJoints, oJointLimits, oCartLimits,
                  oMaxVelPos, oMaxVelRot, oMaxAccelPos, oMaxAccelRot,
                  oSelfTest});
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

    const int restartAtMs = p.value(oRestartAt).toInt();   // 0=off
    const int restartGapMs = p.value(oRestartGap).toInt();
    const quint64 wrapAt  = p.value(oIpocWrap).toULongLong(); // 0=off
    const int jitterUs    = p.value(oJitter).toInt();         // 0=off

    // restartAtMs 会在 gap 期间停发帧 → replies < cycles，属于注入路径，
    // 需放宽 replies==cycles 的严格检查。wrap/jitter 不丢帧，不进 injected。
    const bool injected = dupN > 0 || gapN > 0 || backN > 0 || dropN > 0
                          || reorderN > 0 || lateN > 0 || ignore
                          || restartAtMs > 0;

    if (p.isSet(oSelfTest)) {
        // 正解一致性：已知位形的期望位姿。zero-pose z = d1−d4+d6 = 675−1200+240 = −285。
        struct Case { double q[6]; double x, y, z, a, b, c; };
        const Case cases[] = {
            { {0, 0, 0, 0, 0, 0}, 1541.0, 0.0, -285.0, 0, 0, 0 },
            // 更多位形可由 Task 1 单测值补充
        };
        for (const auto &c : cases) {
            const Pose got = kr210::forward(c.q);
            if (std::fabs(got.x - c.x) > 1e-6 || std::fabs(got.y - c.y) > 1e-6
                || std::fabs(got.z - c.z) > 1e-6
                || std::fabs(got.a - c.a) > 1e-6
                || std::fabs(got.b - c.b) > 1e-6
                || std::fabs(got.c - c.c) > 1e-6) {
                std::fprintf(stderr, "self-test FAIL\n");
                return 2;
            }
        }
        std::printf("self-test OK\n");
        return 0;
    }

    QUdpSocket sock;
    if (!sock.bind(QHostAddress::AnyIPv4, 0)) {
        std::fprintf(stderr, "simulator bind failed: %s\n",
                     qPrintable(sock.errorString()));
        return 2;
    }

    // 关节状态：默认工作位形（腕部非奇异 σ_min≈0.87；原 [0,-45,45,0,0,0] 腕部奇异），
    // 或 --init-joints。内部一律 rad。
    double q[6] = {0, -60.0 * M_PI / 180.0, 30.0 * M_PI / 180.0,
                   0, 90.0 * M_PI / 180.0, 0};
    kr210::JointLimits lim = kr210::limits();   // 本地副本，--joint-limits 可覆盖
    Pose pose = kr210::forward(q);              // RIst 初值 = 真实几何正解
    Pose prevDx{};                              // 加速度限制的上一周期增量

    // --init-joints：6 个度值 → rad。
    if (p.isSet(oInitJoints)) {
        const QStringList toks = p.value(oInitJoints).split(' ', Qt::SkipEmptyParts);
        bool ok = toks.size() == 6;
        for (int i = 0; i < 6 && ok; ++i)
            q[i] = toks[i].toDouble(&ok) * M_PI / 180.0;
        if (!ok) {
            std::fprintf(stderr, "bad --init-joints \"%s\" (need 6 deg values)\n",
                         qPrintable(p.value(oInitJoints)));
            return 2;
        }
        pose = kr210::forward(q);
    }

    // --cart-limits：12 个值（xmin xmax ymin ymax zmin zmax amin amax bmin bmax cmin cmax）。
    double cartLim[12];
    bool cartLimitsSet = false;
    if (p.isSet(oCartLimits)) {
        const QStringList toks = p.value(oCartLimits).split(' ', Qt::SkipEmptyParts);
        bool ok = toks.size() == 12;
        for (int i = 0; i < 12 && ok; ++i)
            cartLim[i] = toks[i].toDouble(&ok);
        if (!ok) {
            std::fprintf(stderr, "bad --cart-limits \"%s\" (need 12 values)\n",
                         qPrintable(p.value(oCartLimits)));
            return 2;
        }
        for (int i = 0; i < 6; ++i) {
            if (cartLim[2 * i] > cartLim[2 * i + 1]) {
                std::fprintf(stderr, "bad --cart-limits \"%s\": axis %d min %.3f > max %.3f\n",
                             qPrintable(p.value(oCartLimits)), i + 1,
                             cartLim[2 * i], cartLim[2 * i + 1]);
                return 2;
            }
        }
        cartLimitsSet = true;
    }

    // --joint-limits：12 个度值（min1 max1 ... min6 max6）→ rad。
    if (p.isSet(oJointLimits)) {
        const QStringList toks = p.value(oJointLimits).split(' ', Qt::SkipEmptyParts);
        bool ok = toks.size() == 12;
        for (int i = 0; i < 6 && ok; ++i) {
            lim.min[i] = toks[2 * i].toDouble(&ok) * M_PI / 180.0;
            if (ok)
                lim.max[i] = toks[2 * i + 1].toDouble(&ok) * M_PI / 180.0;
        }
        if (!ok) {
            std::fprintf(stderr, "bad --joint-limits \"%s\" (need 12 deg values)\n",
                         qPrintable(p.value(oJointLimits)));
            return 2;
        }
        // min > max 会让后续 std::clamp 进入未定义行为，直接拒绝。
        for (int i = 0; i < 6; ++i) {
            if (lim.min[i] > lim.max[i]) {
                std::fprintf(stderr, "bad --joint-limits \"%s\": joint %d min %.3f > max %.3f (deg)\n",
                             qPrintable(p.value(oJointLimits)), i + 1,
                             lim.min[i] * 180.0 / M_PI, lim.max[i] * 180.0 / M_PI);
                return 2;
            }
        }
    }

    // --init-joints 越过限位时 clamp 进限位（防护：避免初始位形越界）。
    // 在 --joint-limits 解析之后执行，用的是覆盖后的限位。
    if (p.isSet(oInitJoints)) {
        bool clamped = false;
        for (int i = 0; i < 6; ++i) {
            const double nq = std::clamp(q[i], lim.min[i], lim.max[i]);
            clamped = clamped || nq != q[i];
            q[i] = nq;
        }
        if (clamped) {
            std::fprintf(stderr, "note: --init-joints clamped into joint limits\n");
            pose = kr210::forward(q);
        }
    }

    // 速度/加速度限制（0 = 不限制），传入 limitCartDelta。
    const double maxVelPos   = p.value(oMaxVelPos).toDouble();
    const double maxVelRot   = p.value(oMaxVelRot).toDouble();
    const double maxAccelPos = p.value(oMaxAccelPos).toDouble();
    const double maxAccelRot = p.value(oMaxAccelRot).toDouble();

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

    // 会话重启状态：restartAtMs 处停发 restartGapMs 毫秒（不发帧、不推进 IPOC、
    // 不处理回包，但节拍照走），随后复位会话——IPOC 回到 1000、q 回到初始位形、
    // Delay 回到基值，模拟 KRL 程序重启 / 新 RSI 会话。initQ 在 --init-joints
    // 与 clamp 处理之后捕获，保证复位到最终生效的初始位形。
    // restartTriggered 是"已触发过"的一次性门闩，恢复后不再清回，避免 gap
    // 结束后立刻再触发一次；gapUntilMs < 0 表示当前不在 gap 中 / 无待恢复。
    bool restartTriggered = false;
    qint64 gapUntilMs = -1;
    double initQ[6];
    std::copy(q, q + 6, initQ);

    for (int i = 0; i < cycles; ++i) {
        // 等到本周期的标称发送时刻（--jitter-us 给节拍加 ±N µs 抖动）
        const qint64 dueNs = qint64(double(i) * cycleMs * 1.0e6)
                             + (jitterUs > 0 ? (std::rand() % (2 * jitterUs + 1) - jitterUs) * 1000 : 0);
        while (pace.nsecsElapsed() < dueNs) {
            const qint64 remainMs = (dueNs - pace.nsecsElapsed()) / 1000000;
            if (remainMs > 1)
                QThread::msleep(1);
        }

        // 会话重启：到 restartAtMs 进入静默 gap。gap 期间不发帧、不推进 IPOC、
        // 不处理回包（重排缓冲与回显队列都保持不动），但节拍照走。
        const qint64 nowMs = pace.nsecsElapsed() / 1000000;
        if (restartAtMs > 0 && !restartTriggered && nowMs >= restartAtMs) {
            restartTriggered = true;      // 一次性门闩：只重启一次
            gapUntilMs = nowMs + restartGapMs;
            std::fprintf(stderr, "session restart at %lld ms\n", nowMs);
        }
        const bool inGap = (gapUntilMs >= 0 && nowMs < gapUntilMs);
        if (inGap) {
            continue;
        }
        if (restartTriggered && gapUntilMs >= 0 && nowMs >= gapUntilMs) {
            // gap 结束：复位会话（IPOC 重置 + q 复位 + delay 重置），并清空
            // 重排缓冲与回显队列——新会话从全新账本开始，不留上一会话残留
            // （否则复位后的首帧回显会被错配到旧队列，误报 ipoc_mismatch）。
            ipoc = 1000;
            std::copy(initQ, initQ + 6, q);
            pose = kr210::forward(q);
            delay = delayBase;
            heldValid = false;
            heldRob.clear();
            sentIpocs.clear();
            gapUntilMs = -1;
            std::fprintf(stderr, "session resumed\n");
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

        const QByteArray rob = buildRob(pose, sendIpoc, delay, q);

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

        // 32 位回绕模拟：IPOC 达到 wrapAt 后回到 0（模拟真实 KRC 的溢出回绕）。
        if (wrapAt > 0 && ipoc >= wrapAt)
            ipoc = 0;

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
                        // 关节模型：笛卡尔 RKorr → 限制 → 雅可比伪逆 → q → 正解回报
                        Pose dx = korr;                     // 笛卡尔增量（mm, °）
                        kr210::limitCartDelta(&dx, prevDx,
                                              maxVelPos, maxVelRot,
                                              maxAccelPos, maxAccelRot,
                                              cycleMs / 1000.0);
                        double dq[6];
                        if (kr210::solveDelta(q, dx, dq)) {
                            for (int i = 0; i < 6; ++i) {
                                q[i] += dq[i];
                                // 关节限位 clamp
                                q[i] = std::clamp(q[i], lim.min[i], lim.max[i]);
                            }
                        }
                        pose = kr210::forward(q);           // RIst = 真实几何正解
                        // 笛卡尔额外约束：clamp 回报的 RIst。只 clamp 回报值，
                        // q 继续（模拟「机器人被笛卡尔限位挡住」——主机看到 RIst 停在限位）。
                        if (cartLimitsSet) {
                            pose.x = std::clamp(pose.x, cartLim[0], cartLim[1]);
                            pose.y = std::clamp(pose.y, cartLim[2], cartLim[3]);
                            pose.z = std::clamp(pose.z, cartLim[4], cartLim[5]);
                            pose.a = std::clamp(pose.a, cartLim[6], cartLim[7]);
                            pose.b = std::clamp(pose.b, cartLim[8], cartLim[9]);
                            pose.c = std::clamp(pose.c, cartLim[10], cartLim[11]);
                        }
                        prevDx = dx;
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

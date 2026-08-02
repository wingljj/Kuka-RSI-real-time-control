// 假 KRC：按固定周期发 <Rob>、收 <Sen>，验证上位机的实时行为。
// 关节模型：q[6] 状态，收到的 RKorr（笛卡尔增量）经 limitCartDelta 限制后
// 叠加到当前位姿得目标位姿，RL（rlk）逆解一次得 q（失败回零增量），
// clamp 到限位，forward 正解回报 RIst/AIPos。
// 支持故障注入：--ipoc-dup/--ipoc-gap/--ipoc-back/--drop/--reorder/--late-ms
// /--ignore-replies/--send-delay，用于验证主机的异常处理路径。
// 会话/时序增强：--restart-at-ms/--restart-gap-ms（KRL 重启：停发 + IPOC/q
// 复位）、--ipoc-wrap-at（IPOC 回绕）、--jitter-us（节拍抖动）。
#include <QApplication>
#include <QCommandLineParser>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QThread>
#include <QTimer>
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
#include "tools/krc_simulator/rl_kinematics.h"
#include "tools/krc_simulator/RobotView.h"
#include "tools/krc_simulator/MeshLoader.h"

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

// 模拟器的全部可变状态——抽取出来供 headless 和 viz 双驱动共享。
struct SimContext {
    double q[6] = {0, 0, 0, 0, 0, 0};
    kr210::JointLimits lim{};
    Pose pose{};
    Pose prevDx{};

    double cartLim[12]{};
    bool cartLimitsSet = false;

    double maxVelPos = 0, maxVelRot = 0;
    double maxAccelPos = 0, maxAccelRot = 0;

    int dupN = 0, gapN = 0, backN = 0, dropN = 0, reorderN = 0, lateN = 0;
    bool ignore = false;
    quint64 delayBase = 0;

    int restartAtMs = 0, restartGapMs = 0;
    quint64 wrapAt = 0;
    int jitterUs = 0;
    bool injected = false;

    quint64 ipoc = 1000;
    quint64 lastSent = 0;
    quint64 delay = 0;

    int replies = 0, ipocMismatch = 0, missed = 0;
    quint64 wrapCount = 0;
    double maxRttUs = 0.0, sumRttUs = 0.0;

    QElapsedTimer pace;

    QByteArray heldRob;
    quint64 heldIpoc = 0;
    bool heldValid = false;

    std::deque<quint64> sentIpocs;

    bool restartTriggered = false;
    qint64 gapUntilMs = -1;
    double initQ[6] = {0, 0, 0, 0, 0, 0};
};

} // namespace

// 无窗口驱动：当前阻塞式 for 循环，逐帧节拍 + waitForReadyRead + 收包。
// 行为与旧代码逐字节一致——这是回归合约，viz 模式不影响此路径。
static int runHeadless(SimContext &ctx, double cycleMs, int cycles,
                       QUdpSocket &sock, const QHostAddress &host, quint16 port)
{
    // cycles == 0 → 无限运行（直到 Ctrl+C / 关窗）。双击场景希望一直发帧，
    // 而不想看到"运行 2 分钟自动退出"。int 溢出需 ~2^31 帧 ≈ 9 年，不现实。
    for (int i = 0; cycles == 0 || i < cycles; ++i) {
        // 无限模式下每 500 周期打一次进度，让控制台可见模拟器仍在发帧
        if (cycles == 0 && i % 500 == 0) {
            std::printf("i=%d replies=%d missed=%d ipoc_mismatch=%d\n",
                        i, ctx.replies, ctx.missed, ctx.ipocMismatch);
            std::fflush(stdout);
        }
        // 等到本周期的标称发送时刻（--jitter-us 给节拍加 ±N µs 抖动）
        const qint64 dueNs = qint64(double(i) * cycleMs * 1.0e6)
                             + (ctx.jitterUs > 0 ? (std::rand() % (2 * ctx.jitterUs + 1) - ctx.jitterUs) * 1000 : 0);
        while (ctx.pace.nsecsElapsed() < dueNs) {
            const qint64 remainMs = (dueNs - ctx.pace.nsecsElapsed()) / 1000000;
            if (remainMs > 1)
                QThread::msleep(1);
        }

        // 会话重启：到 restartAtMs 进入静默 gap。gap 期间不发帧、不推进 IPOC、
        // 不处理回包（重排缓冲与回显队列都保持不动），但节拍照走。
        const qint64 nowMs = ctx.pace.nsecsElapsed() / 1000000;
        if (ctx.restartAtMs > 0 && !ctx.restartTriggered && nowMs >= ctx.restartAtMs) {
            ctx.restartTriggered = true;      // 一次性门闩：只重启一次
            ctx.gapUntilMs = nowMs + ctx.restartGapMs;
            std::fprintf(stderr, "session restart at %lld ms\n", nowMs);
        }
        const bool inGap = (ctx.gapUntilMs >= 0 && nowMs < ctx.gapUntilMs);
        if (inGap) {
            continue;
        }
        if (ctx.restartTriggered && ctx.gapUntilMs >= 0 && nowMs >= ctx.gapUntilMs) {
            // gap 结束：复位会话（IPOC 重置 + q 复位 + delay 重置），并清空
            // 重排缓冲与回显队列——新会话从全新账本开始，不留上一会话残留
            // （否则复位后的首帧回显会被错配到旧队列，误报 ipoc_mismatch）。
            ctx.ipoc = 1000;
            std::copy(ctx.initQ, ctx.initQ + 6, ctx.q);
            ctx.pose = rlk::forward(ctx.q);
            ctx.delay = ctx.delayBase;
            ctx.heldValid = false;
            ctx.heldRob.clear();
            ctx.sentIpocs.clear();
            ctx.gapUntilMs = -1;
            std::fprintf(stderr, "session resumed\n");
        }

        const bool dup     = active(ctx.dupN, i);
        const bool gap     = active(ctx.gapN, i);
        const bool back    = active(ctx.backN, i);
        const bool drop    = active(ctx.dropN, i);
        const bool reorder = active(ctx.reorderN, i);
        const bool late    = active(ctx.lateN, i);

        // 决定本帧 IPOC：dup 重发上一帧；back 回退；gap 前向跳号。
        quint64 sendIpoc = ctx.ipoc;
        if (dup)         sendIpoc = ctx.lastSent;
        else if (back)   sendIpoc = (ctx.lastSent > 0) ? ctx.lastSent - 1 : 0;
        else if (gap)    sendIpoc = ctx.ipoc + ctx.gapN;

        const QByteArray rob = buildRob(ctx.pose, sendIpoc, ctx.delay, ctx.q);

        auto sendFrame = [&](const QByteArray &rob2, quint64 frameIpoc) {
            if (late)
                QThread::msleep(ctx.lateN);
            sock.writeDatagram(rob2, host, port);
            if (!ctx.ignore)
                ctx.sentIpocs.push_back(frameIpoc);
        };

        if (reorder) {
            // 本帧缓冲，下周期后发（真乱序）
            ctx.heldRob   = rob;
            ctx.heldIpoc  = sendIpoc;
            ctx.heldValid = true;
        } else {
            if (!drop)
                sendFrame(rob, sendIpoc);          // 本帧先发
            if (ctx.heldValid) {
                sendFrame(ctx.heldRob, ctx.heldIpoc);      // 缓冲帧后发（真乱序）
                ctx.heldValid = false;
            }
        }

        // 推进序列：dup 不推进（下帧还发同一个）；其余推进。
        if (!dup) {
            ctx.lastSent = sendIpoc;
            ctx.ipoc     = sendIpoc + 1;
        }

        // 32 位回绕模拟：IPOC 达到 wrapAt 后回到 0（模拟真实 KRC 的溢出回绕）。
        // 每次回绕使下一帧以 IPOC 0 发出——主机把 0 当解析失败哨兵回显
        // lastGood 而非 0，于是合法地产生一次 ipoc_mismatch（主机滚动溢出的
        // 固有局限）。计入 wrapCount，由 PASS 门按预期量放宽（见文件末尾）。
        if (ctx.wrapAt > 0 && ctx.ipoc >= ctx.wrapAt) {
            ctx.ipoc = 0;
            ++ctx.wrapCount;
        }

        QElapsedTimer rtt;
        rtt.start();
        // 等待本周期内的回包。注意不能用 waitForReadyRead 的返回值当作
        // "收到回复"：端口关闭时 Windows 的 ICMP port-unreachable 也会让它
        // 返回 true，那样该周期既不计 replies 也不计 timeouts，
        // cycles == replies + missed 就不再成立。只认解析成功的 <Sen>。
        if (!ctx.ignore) {
            const int budgetMs = std::max(1, int(cycleMs));
            bool got = false;
            if (sock.waitForReadyRead(budgetMs)) {
                while (sock.hasPendingDatagrams()) {
                    const QByteArray d = sock.receiveDatagram().data();
                    Pose korr;
                    quint64 echoed = 0;
                    if (parseSen(d, &korr, &echoed)) {
                        ++ctx.replies;
                        got = true;
                        if (!ctx.sentIpocs.empty()) {
                            if (echoed != ctx.sentIpocs.front())
                                ++ctx.ipocMismatch;
                            ctx.sentIpocs.pop_front();
                        }
                        // 关节模型：笛卡尔 RKorr → 限制 → 目标位姿 → RL 一次逆解
                        // → q → 正解回报（目标位姿法，替代逐周期雅可比伪逆）。
                        Pose dx = korr;                     // 笛卡尔增量（mm, °）
                        kr210::limitCartDelta(&dx, ctx.prevDx,
                                              ctx.maxVelPos, ctx.maxVelRot,
                                              ctx.maxAccelPos, ctx.maxAccelRot,
                                              cycleMs / 1000.0);
                        // 目标位姿 = 当前位姿（正解）+ 限制后的增量。
                        Pose target = ctx.pose;
                        target.x += dx.x;
                        target.y += dx.y;
                        target.z += dx.z;
                        target.a += dx.a;
                        target.b += dx.b;
                        target.c += dx.c;
                        // 笛卡尔额外约束：clamp 目标位姿再逆解（模拟「机器人被
                        // 笛卡尔限位挡住」——主机看到 RIst 停在限位）。
                        if (ctx.cartLimitsSet) {
                            target.x = std::clamp(target.x, ctx.cartLim[0], ctx.cartLim[1]);
                            target.y = std::clamp(target.y, ctx.cartLim[2], ctx.cartLim[3]);
                            target.z = std::clamp(target.z, ctx.cartLim[4], ctx.cartLim[5]);
                            target.a = std::clamp(target.a, ctx.cartLim[6], ctx.cartLim[7]);
                            target.b = std::clamp(target.b, ctx.cartLim[8], ctx.cartLim[9]);
                            target.c = std::clamp(target.c, ctx.cartLim[10], ctx.cartLim[11]);
                        }
                        double qNew[6];
                        // 以当前关节角为迭代起点：目标只比当前位姿远不到 1mm，
                        // 从这里出发一两次迭代就收敛，逆解耗时不再随机器人走远
                        // 而膨胀（否则发帧节拍被逆解拖崩，主机看门狗误判断流）。
                        if (rlk::inverse(target, qNew, ctx.q)) {
                            for (int i = 0; i < 6; ++i) {
                                // 关节限位 clamp（与 RL 内部限位双重保险；
                                // --joint-limits 覆盖本地副本后在此生效）
                                ctx.q[i] = std::clamp(qNew[i], ctx.lim.min[i], ctx.lim.max[i]);
                            }
                        }
                        // 逆解失败（目标不可达）保持旧 q（安全回退）。
                        ctx.pose = rlk::forward(ctx.q);             // RIst = 真实几何正解
                        ctx.prevDx = dx;
                    }
                }
            }
            if (got) {
                const double us = rtt.nsecsElapsed() / 1000.0;
                ctx.maxRttUs = std::max(ctx.maxRttUs, us);
                ctx.sumRttUs += us;
            } else {
                ++ctx.missed;
            }
        } else {
            // 模拟 SENTYPE 错配：KRC 静默丢弃每一帧回包，主机毫无察觉，
            // 只有 KRC 自己的 Delay 计数在涨。这里递增 delay，触发主机
            // 的"KRC Delay 连续 3 帧递增 → Fault"运行中保护。
            ++ctx.missed;
            ctx.delay += 1;
        }
    }

    std::printf("cycles=%d replies=%d missed=%d ipoc_mismatch=%d delay=%llu\n",
                cycles, ctx.replies, ctx.missed, ctx.ipocMismatch,
                static_cast<unsigned long long>(ctx.delay));
    std::printf("rtt_avg_us=%.1f rtt_max_us=%.1f\n",
                ctx.replies ? ctx.sumRttUs / ctx.replies : 0.0, ctx.maxRttUs);
    std::printf("final_pose X=%.3f Y=%.3f Z=%.3f A=%.3f B=%.3f C=%.3f\n",
                ctx.pose.x, ctx.pose.y, ctx.pose.z, ctx.pose.a, ctx.pose.b, ctx.pose.c);

    // 主机必须始终原样回显 IPOC（即使对异常帧回零增量）。无注入时还要求
    // 每帧都回包；有注入时丢包是预期行为，只查回显正确。
    // --ipoc-wrap-at 模拟真实 KRC 的 32 位溢出回绕：主机把 IPOC 0 当作解析
    // 失败哨兵（RsiWorker 对 ipoc==0 回显 lastGood 而非 0），所以每次回绕的
    // 那帧必然被记一次 ipoc_mismatch——这是主机对滚动溢出的固有局限，不是
    // 链路错误。计数实际回绕次数，允许等量的预期不匹配；超过才算失败。
    const quint64 expectedMismatch = ctx.wrapAt > 0 ? ctx.wrapCount : 0;
    const bool pass = ctx.ipocMismatch <= expectedMismatch
                      && (ctx.injected || ctx.replies == cycles);
    if (ctx.wrapAt > 0) {
        std::printf("ipoc_wrap_at=%llu wraps=%llu (each wrap-to-0 yields one "
                    "expected ipoc_mismatch: host IPOC-0 sentinel)\n",
                    static_cast<unsigned long long>(ctx.wrapAt),
                    static_cast<unsigned long long>(ctx.wrapCount));
    }
    std::printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    QCommandLineParser p;
    p.addHelpOption();
    // 默认 127.0.0.1（本机回环，不依赖 VMnet、不被防火墙挡）：双击即与本机
    // rsi_host（监听 127.0.0.1）匹配。真机场景用 --host 指定宿主 IP。
    QCommandLineOption oHost("host", "host IP", "ip", "127.0.0.1");
    QCommandLineOption oPort("port", "host port", "n", "59152");
    QCommandLineOption oCycle("cycle-ms", "cycle", "ms", "12.0");
    // 默认 10000 帧 ≈ 2 分钟：双击 exe 也能看到效果，不至于 6 秒闪退。
    QCommandLineOption oCount("cycles", "cycle count (0 = run forever)", "n", "0");
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
    QCommandLineOption oIpocWrap("ipoc-wrap-at",
                                 "wrap IPOC counter back to 0 when it reaches N (0=off). "
                                 "Wrapping to 0 hits the host's IPOC-0 sentinel (RsiWorker "
                                 "echoes lastGood instead of 0), so each wrap yields one "
                                 "EXPECTED ipoc_mismatch -- a host rollover limitation, "
                                 "allowed by the PASS gate", "n", "0");
    QCommandLineOption oJitter("jitter-us", "add ±N us random jitter to send cadence (0=off)", "us", "0");
    QCommandLineOption oInitJoints("init-joints", "initial joint angles deg \"q1..q6\"", "q", "");
    QCommandLineOption oJointLimits("joint-limits", "override joint limits deg \"min1 max1 ... min6 max6\"", "s", "");
    QCommandLineOption oCartLimits("cart-limits", "cartesian pose ranges \"xmin xmax ... cmin cmax\"", "s", "");
    QCommandLineOption oMaxVelPos("max-vel-pos", "cartesian position velocity limit mm/s (0=off)", "mm/s", "0");
    QCommandLineOption oMaxVelRot("max-vel-rot", "cartesian rotation velocity limit deg/s (0=off)", "deg/s", "0");
    QCommandLineOption oMaxAccelPos("max-accel-pos", "cartesian position acceleration limit mm/s2 (0=off)", "mm/s2", "0");
    QCommandLineOption oMaxAccelRot("max-accel-rot", "cartesian rotation acceleration limit deg/s2 (0=off)", "deg/s2", "0");
    QCommandLineOption oModel("model", "RL rlmdl model path (Comau Racer 7-1.4)", "path",
                              "D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml");
    QCommandLineOption oViz("viz", "show live 3D robot view (requires OpenGL)", "");
    QCommandLineOption oSelfTest("self-test", "run forward-kinematics self-test and exit", "");
    p.addOptions({oHost, oPort, oCycle, oCount, oDup, oGap, oBack,
                  oDrop, oReorder, oLate, oIgnore, oDelay,
                  oRestartAt, oRestartGap, oIpocWrap, oJitter,
                  oInitJoints, oJointLimits, oCartLimits,
                  oMaxVelPos, oMaxVelRot, oMaxAccelPos, oMaxAccelRot,
                  oModel, oViz, oSelfTest});
    p.process(app);

    const QHostAddress host(p.value(oHost));
    const quint16 port  = quint16(p.value(oPort).toUShort());
    const double cycleMs = p.value(oCycle).toDouble();
    const int cycles     = p.value(oCount).toInt();

    SimContext ctx;
    ctx.dupN     = p.value(oDup).toInt();
    ctx.gapN     = p.value(oGap).toInt();
    ctx.backN    = p.value(oBack).toInt();
    ctx.dropN    = p.value(oDrop).toInt();
    ctx.reorderN = p.value(oReorder).toInt();
    ctx.lateN    = p.value(oLate).toInt();
    ctx.ignore   = p.isSet(oIgnore);
    ctx.delayBase = p.value(oDelay).toULongLong();
    ctx.delay    = ctx.delayBase;

    ctx.restartAtMs = p.value(oRestartAt).toInt();   // 0=off
    ctx.restartGapMs = p.value(oRestartGap).toInt();
    ctx.wrapAt  = p.value(oIpocWrap).toULongLong(); // 0=off
    ctx.jitterUs    = p.value(oJitter).toInt();         // 0=off

    // restartAtMs 会在 gap 期间停发帧 → replies < cycles，属于注入路径，
    // 需放宽 replies==cycles 的严格检查。wrap/jitter 不丢帧，不进 injected。
    ctx.injected = ctx.dupN > 0 || ctx.gapN > 0 || ctx.backN > 0 || ctx.dropN > 0
                   || ctx.reorderN > 0 || ctx.lateN > 0 || ctx.ignore
                   || ctx.restartAtMs > 0;

    // 加载 RL 运动学模型（Comau Racer 7-1.4，--model 可覆盖路径）。
    // 模型是正逆解的前提，加载失败直接退出（后续 forward/inverse 返回零值/false，
    // 静默运行会把模拟器变成死机，必须显式失败）。
    const QString modelPath = p.value(oModel);
    if (!rlk::loadModel(modelPath.toStdString())) {
        std::fprintf(stderr, "failed to load RL model: %s\n",
                     qPrintable(modelPath));
        return 2;
    }

    if (p.isSet(oSelfTest)) {
        // 正解一致性：已知位形的期望位姿（RL-T1 实测，docs/rl-build-notes.md）。
        // Comau Racer 7-1.4 与 KR210 不同：q=0 的 TCP 是 [20, 0, 1804]，
        // 模型的 home 位形 q=(0,0,-90°,0,0,0) 才是 [934, 0, 1150]
        // （姿态四元数 (0.707, 0, 0.707, 0) = Ry(90°) → B=90°）。
        struct Case { double q[6]; double x, y, z, a, b, c; };
        const Case cases[] = {
            { {0, 0, 0, 0, 0, 0},            20.0, 0.0, 1804.0, 0, 0, 0 },
            { {0, 0, -90.0 * M_PI / 180.0,
               0, 0, 0},                     934.0, 0.0, 1150.0, 0, 90, 0 },
            // 更多位形可由 Task 1 单测值补充
        };
        for (const auto &c : cases) {
            const Pose got = rlk::forward(c.q);
            if (std::fabs(got.x - c.x) > 1e-6 || std::fabs(got.y - c.y) > 1e-6
                || std::fabs(got.z - c.z) > 1e-6
                || std::fabs(got.a - c.a) > 1e-6
                || std::fabs(got.b - c.b) > 1e-6
                || std::fabs(got.c - c.c) > 1e-6) {
                std::fprintf(stderr, "self-test FAIL got X=%.6f Y=%.6f Z=%.6f "
                            "A=%.6f B=%.6f C=%.6f\n",
                            got.x, got.y, got.z, got.a, got.b, got.c);
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

    // 关节状态：默认 Comau 零位全 0（q=0 → 基座系 [20, 0, 1804]；KR210 的
    // 默认位形 q3=30° 超出 Comau 的 [-170°, 0°] 量程，不能再用），
    // 或 --init-joints。内部一律 rad。
    ctx.lim = rlk::limits();                    // 本地副本，--joint-limits 可覆盖
    ctx.pose = rlk::forward(ctx.q);             // RIst 初值 = 真实几何正解

    // --init-joints：6 个度值 → rad。
    if (p.isSet(oInitJoints)) {
        const QStringList toks = p.value(oInitJoints).split(' ', Qt::SkipEmptyParts);
        bool ok = toks.size() == 6;
        for (int i = 0; i < 6 && ok; ++i) {
            ctx.q[i] = toks[i].toDouble(&ok) * M_PI / 180.0;
            // toDouble 会接受 "nan"/"inf"，NaN 随后会传播进 forward()/final_pose
            // （主机 toDouble 失败 → 每帧 invalid），必须显式挡掉。
            if (ok && !std::isfinite(ctx.q[i]))
                ok = false;
        }
        if (!ok) {
            std::fprintf(stderr, "bad --init-joints \"%s\" (need 6 finite deg values)\n",
                         qPrintable(p.value(oInitJoints)));
            return 2;
        }
        ctx.pose = rlk::forward(ctx.q);
    }

    // --cart-limits：12 个值（xmin xmax ymin ymax zmin zmax amin amax bmin bmax cmin cmax）。
    if (p.isSet(oCartLimits)) {
        const QStringList toks = p.value(oCartLimits).split(' ', Qt::SkipEmptyParts);
        bool ok = toks.size() == 12;
        for (int i = 0; i < 12 && ok; ++i) {
            ctx.cartLim[i] = toks[i].toDouble(&ok);
            // toDouble 会接受 "nan"/"inf"，NaN 让 min>max 恒假且 clamp 传播 NaN。
            if (ok && !std::isfinite(ctx.cartLim[i]))
                ok = false;
        }
        if (!ok) {
            std::fprintf(stderr, "bad --cart-limits \"%s\" (need 12 finite values)\n",
                         qPrintable(p.value(oCartLimits)));
            return 2;
        }
        for (int i = 0; i < 6; ++i) {
            if (ctx.cartLim[2 * i] > ctx.cartLim[2 * i + 1]) {
                std::fprintf(stderr, "bad --cart-limits \"%s\": axis %d min %.3f > max %.3f\n",
                             qPrintable(p.value(oCartLimits)), i + 1,
                             ctx.cartLim[2 * i], ctx.cartLim[2 * i + 1]);
                return 2;
            }
        }
        ctx.cartLimitsSet = true;
    }

    // --joint-limits：12 个度值（min1 max1 ... min6 max6）→ rad。
    if (p.isSet(oJointLimits)) {
        const QStringList toks = p.value(oJointLimits).split(' ', Qt::SkipEmptyParts);
        bool ok = toks.size() == 12;
        for (int i = 0; i < 6 && ok; ++i) {
            ctx.lim.min[i] = toks[2 * i].toDouble(&ok) * M_PI / 180.0;
            if (ok)
                ctx.lim.max[i] = toks[2 * i + 1].toDouble(&ok) * M_PI / 180.0;
            // toDouble 会接受 "nan"/"inf"，NaN 让 min>max 恒假且 clamp 传播 NaN。
            if (ok && (!std::isfinite(ctx.lim.min[i]) || !std::isfinite(ctx.lim.max[i])))
                ok = false;
        }
        if (!ok) {
            std::fprintf(stderr, "bad --joint-limits \"%s\" (need 12 finite deg values)\n",
                         qPrintable(p.value(oJointLimits)));
            return 2;
        }
        // min > max 会让后续 std::clamp 进入未定义行为，直接拒绝。
        for (int i = 0; i < 6; ++i) {
            if (ctx.lim.min[i] > ctx.lim.max[i]) {
                std::fprintf(stderr, "bad --joint-limits \"%s\": joint %d min %.3f > max %.3f (deg)\n",
                             qPrintable(p.value(oJointLimits)), i + 1,
                             ctx.lim.min[i] * 180.0 / M_PI, ctx.lim.max[i] * 180.0 / M_PI);
                return 2;
            }
        }
    }

    // --init-joints 越过限位时 clamp 进限位（防护：避免初始位形越界）。
    // 在 --joint-limits 解析之后执行，用的是覆盖后的限位。
    if (p.isSet(oInitJoints)) {
        bool clamped = false;
        for (int i = 0; i < 6; ++i) {
            const double nq = std::clamp(ctx.q[i], ctx.lim.min[i], ctx.lim.max[i]);
            clamped = clamped || nq != ctx.q[i];
            ctx.q[i] = nq;
        }
        if (clamped) {
            std::fprintf(stderr, "note: --init-joints clamped into joint limits\n");
            ctx.pose = rlk::forward(ctx.q);
        }
    }

    // 速度/加速度限制（0 = 不限制），传入 limitCartDelta。
    ctx.maxVelPos   = p.value(oMaxVelPos).toDouble();
    ctx.maxVelRot   = p.value(oMaxVelRot).toDouble();
    ctx.maxAccelPos = p.value(oMaxAccelPos).toDouble();
    ctx.maxAccelRot = p.value(oMaxAccelRot).toDouble();

    // 会话重启初始位形——在 --init-joints 与 clamp 处理之后捕获。
    std::copy(ctx.q, ctx.q + 6, ctx.initQ);

    ctx.pace.start();

    const bool viz = p.isSet(oViz);
    if (!viz) {
        return runHeadless(ctx, cycleMs, cycles, sock, host, port);
    }

    // --viz 模式：QTimer 驱动的事件循环 + 3D 机器人视图
    RobotView view;
    view.resize(900, 700);
    view.setWindowTitle("Comau Racer 7-1.4 — KRC Simulator (RL)");

    // 加载 VRML 网格（link0.wrl..link6.wrl）
    {
        const double *homeQ = ctx.q;
        std::string meshDir = "D:/QTproj/rl/rl-master/3dmodel";
        std::string modelDir = modelPath.toStdString();
        std::size_t slash = modelDir.find_last_of("/\\");
        if (slash != std::string::npos)
            meshDir = modelDir.substr(0, slash);
        std::vector<BodyMesh> meshes = loadModelMeshes(meshDir, homeQ);
        if (!meshes.empty())
            view.setMeshes(meshes);
    }

    // 立即显示初始位姿（不等首帧回包——用户开了 --viz 就要看到机器人）
    {
        rlk::forward(ctx.q);
        const rlk::Skeleton skel = rlk::skeleton();
        double qDeg[6];
        for (int i = 0; i < 6; ++i) qDeg[i] = ctx.q[i] * 180.0 / M_PI;
        view.updateRobot(skel, qDeg);
    }
    view.show();

    int vizCycle = 0;

    // timer 间隔选 cycleMs/2（不超过 4ms），纯 poll 不引入显著抖动
    QTimer tick;
    tick.setTimerType(Qt::PreciseTimer);

    QObject::connect(&tick, &QTimer::timeout, [&]() {
        // 发送到期帧——while 循环批量发送所有应发的帧，保持绝对节拍
        while (cycles == 0 || vizCycle < cycles) {
            const qint64 dueNs = qint64(double(vizCycle) * cycleMs * 1.0e6)
                                 + (ctx.jitterUs > 0 ? (std::rand() % (2 * ctx.jitterUs + 1) - ctx.jitterUs) * 1000 : 0);
            if (ctx.pace.nsecsElapsed() < dueNs)
                break;

            // 进度输出
            if (cycles == 0 && vizCycle % 500 == 0) {
                std::printf("i=%d replies=%d missed=%d ipoc_mismatch=%d\n",
                            vizCycle, ctx.replies, ctx.missed, ctx.ipocMismatch);
                std::fflush(stdout);
            }
            // 会话重启逻辑（与 runHeadless 相同）
            const qint64 nowMs = ctx.pace.nsecsElapsed() / 1000000;
            if (ctx.restartAtMs > 0 && !ctx.restartTriggered && nowMs >= ctx.restartAtMs) {
                ctx.restartTriggered = true;
                ctx.gapUntilMs = nowMs + ctx.restartGapMs;
                std::fprintf(stderr, "session restart at %lld ms\n", nowMs);
            }
            const bool inGap = (ctx.gapUntilMs >= 0 && nowMs < ctx.gapUntilMs);
            if (inGap) {
                ++vizCycle;  // gap 期间占位推进（与 headless 的 for-loop ++i 一致）
                continue;
            }
            if (ctx.restartTriggered && ctx.gapUntilMs >= 0 && nowMs >= ctx.gapUntilMs) {
                ctx.ipoc = 1000;
                std::copy(ctx.initQ, ctx.initQ + 6, ctx.q);
                ctx.pose = rlk::forward(ctx.q);
                ctx.delay = ctx.delayBase;
                ctx.heldValid = false;
                ctx.heldRob.clear();
                ctx.sentIpocs.clear();
                ctx.gapUntilMs = -1;
                std::fprintf(stderr, "session resumed\n");
            }

            const bool dup     = active(ctx.dupN, vizCycle);
            const bool gap     = active(ctx.gapN, vizCycle);
            const bool back    = active(ctx.backN, vizCycle);
            const bool drop    = active(ctx.dropN, vizCycle);
            const bool reorder = active(ctx.reorderN, vizCycle);
            const bool late    = active(ctx.lateN, vizCycle);
            quint64 sendIpoc = ctx.ipoc;
            if (dup)         sendIpoc = ctx.lastSent;
            else if (back)   sendIpoc = (ctx.lastSent > 0) ? ctx.lastSent - 1 : 0;
            else if (gap)    sendIpoc = ctx.ipoc + ctx.gapN;
            const QByteArray rob = buildRob(ctx.pose, sendIpoc, ctx.delay, ctx.q);
            auto sendFrame = [&](const QByteArray &r, quint64 ip) {
                if (late) QThread::msleep(ctx.lateN);
                sock.writeDatagram(r, host, port);
                if (!ctx.ignore) ctx.sentIpocs.push_back(ip);
            };
            if (reorder) {
                ctx.heldRob = rob; ctx.heldIpoc = sendIpoc; ctx.heldValid = true;
            } else {
                if (!drop) sendFrame(rob, sendIpoc);
                if (ctx.heldValid) { sendFrame(ctx.heldRob, ctx.heldIpoc); ctx.heldValid = false; }
            }
            if (!dup) { ctx.lastSent = sendIpoc; ctx.ipoc = sendIpoc + 1; }
            if (ctx.wrapAt > 0 && ctx.ipoc >= ctx.wrapAt) { ctx.ipoc = 0; ++ctx.wrapCount; }
            ++vizCycle;
        }

        // 收包（非阻塞 drain——与 headless 的 waitForReadyRead 不同，viz 用事件驱动）
        if (!ctx.ignore) {
            while (sock.hasPendingDatagrams()) {
                const QByteArray d = sock.receiveDatagram().data();
                Pose korr;
                quint64 echoed = 0;
                if (!parseSen(d, &korr, &echoed))
                    continue;
                ++ctx.replies;
                if (!ctx.sentIpocs.empty()) {
                    if (echoed != ctx.sentIpocs.front())
                        ++ctx.ipocMismatch;
                    ctx.sentIpocs.pop_front();
                }
                QElapsedTimer rtt;
                rtt.start();
                Pose dx = korr;
                kr210::limitCartDelta(&dx, ctx.prevDx,
                                      ctx.maxVelPos, ctx.maxVelRot,
                                      ctx.maxAccelPos, ctx.maxAccelRot,
                                      cycleMs / 1000.0);
                Pose target = ctx.pose;
                target.x += dx.x; target.y += dx.y; target.z += dx.z;
                target.a += dx.a; target.b += dx.b; target.c += dx.c;
                if (ctx.cartLimitsSet) {
                    target.x = std::clamp(target.x, ctx.cartLim[0], ctx.cartLim[1]);
                    target.y = std::clamp(target.y, ctx.cartLim[2], ctx.cartLim[3]);
                    target.z = std::clamp(target.z, ctx.cartLim[4], ctx.cartLim[5]);
                    target.a = std::clamp(target.a, ctx.cartLim[6], ctx.cartLim[7]);
                    target.b = std::clamp(target.b, ctx.cartLim[8], ctx.cartLim[9]);
                    target.c = std::clamp(target.c, ctx.cartLim[10], ctx.cartLim[11]);
                }
                double qNew[6];
                // 同 headless 路径：当前关节角作种子，避免逆解拖垮 QTimer 节拍。
                if (rlk::inverse(target, qNew, ctx.q)) {
                    for (int i = 0; i < 6; ++i)
                        ctx.q[i] = std::clamp(qNew[i], ctx.lim.min[i], ctx.lim.max[i]);
                }
                ctx.pose = rlk::forward(ctx.q);
                ctx.prevDx = dx;
                const double us = rtt.nsecsElapsed() / 1000.0;
                ctx.maxRttUs = std::max(ctx.maxRttUs, us);
                ctx.sumRttUs += us;
            }
        } else {
            while (sock.hasPendingDatagrams())
                sock.receiveDatagram();
        }

        // 每 tick 刷新 3D 视图（无条件——不管有没有回包，骨架都要显示）
        {
            const rlk::Skeleton skel = rlk::skeleton();
            double qDeg[6];
            for (int i = 0; i < 6; ++i) qDeg[i] = ctx.q[i] * 180.0 / M_PI;
            view.updateRobot(skel, qDeg);
        }
        view.setCycleInfo(vizCycle, ctx.replies, ctx.missed);

        // 完成条件（cycles > 0 时）
        if (cycles > 0 && vizCycle >= cycles && ctx.sentIpocs.empty()) {
            tick.stop();
            std::printf("cycles=%d replies=%d missed=%d ipoc_mismatch=%d delay=%llu\n",
                        cycles, ctx.replies, ctx.missed, ctx.ipocMismatch,
                        static_cast<unsigned long long>(ctx.delay));
            std::printf("rtt_avg_us=%.1f rtt_max_us=%.1f\n",
                        ctx.replies ? ctx.sumRttUs / ctx.replies : 0.0, ctx.maxRttUs);
            std::printf("final_pose X=%.3f Y=%.3f Z=%.3f A=%.3f B=%.3f C=%.3f\n",
                        ctx.pose.x, ctx.pose.y, ctx.pose.z, ctx.pose.a, ctx.pose.b, ctx.pose.c);
            const quint64 em = ctx.wrapAt > 0 ? ctx.wrapCount : 0;
            const bool pass = ctx.ipocMismatch <= em && (ctx.injected || ctx.replies == cycles);
            std::printf("%s\n", pass ? "PASS" : "FAIL");
            QApplication::quit();
        }
    });

    tick.start(std::max(1, int(std::min(4.0, cycleMs * 0.5))));
    return app.exec();
}

// 假 KRC：按固定周期发 <Rob>、收 <Sen>，验证上位机的实时行为。
// 把收到的 RKorr 累加到自身位姿，模拟 RELATIVE 修正语义。
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
#include "core/Pose.h"
#include "core/RsiCodec.h"

namespace {

QByteArray buildRob(const Pose &p, quint64 ipoc)
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
    s += "/>\n<Delay D=\"0\"/>\n<IPOC>";
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
    p.addOptions({oHost, oPort, oCycle, oCount});
    p.process(app);

    const QHostAddress host(p.value(oHost));
    const quint16 port  = quint16(p.value(oPort).toUShort());
    const double cycleMs = p.value(oCycle).toDouble();
    const int cycles     = p.value(oCount).toInt();

    QUdpSocket sock;
    if (!sock.bind(QHostAddress::AnyIPv4, 0)) {
        std::fprintf(stderr, "simulator bind failed: %s\n",
                     qPrintable(sock.errorString()));
        return 2;
    }

    Pose pose{1250.0, 0.0, 1000.0, 0.0, 90.0, 0.0};
    quint64 ipoc = 1000;

    int replies = 0, ipocMismatch = 0, missed = 0;
    double maxRttUs = 0.0, sumRttUs = 0.0;

    // 真实 KRC 按固定节拍发帧。若不设节拍而是收到回复就立刻发下一帧，
    // 面对快速主机会全速空转——那测的是吞吐，不是"能否在周期内回复"，
    // 而且主机侧实测出来的周期也不再是 cycleMs。
    QElapsedTimer pace;
    pace.start();

    for (int i = 0; i < cycles; ++i) {
        // 等到本周期的标称发送时刻
        const qint64 dueNs = qint64(double(i) * cycleMs * 1.0e6);
        while (pace.nsecsElapsed() < dueNs) {
            const qint64 remainMs = (dueNs - pace.nsecsElapsed()) / 1000000;
            if (remainMs > 1)
                QThread::msleep(1);
        }

        QElapsedTimer rtt;
        rtt.start();
        sock.writeDatagram(buildRob(pose, ipoc), host, port);

        // 等待本周期内的回包。注意不能用 waitForReadyRead 的返回值当作
        // "收到回复"：端口关闭时 Windows 的 ICMP port-unreachable 也会让它
        // 返回 true，那样该周期既不计 replies 也不计 timeouts，
        // cycles == replies + missed 就不再成立。只认解析成功的 <Sen>。
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
                    if (echoed != ipoc)
                        ++ipocMismatch;
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
        ++ipoc;
    }

    std::printf("cycles=%d replies=%d missed=%d ipoc_mismatch=%d\n",
                cycles, replies, missed, ipocMismatch);
    std::printf("rtt_avg_us=%.1f rtt_max_us=%.1f\n",
                replies ? sumRttUs / replies : 0.0, maxRttUs);
    std::printf("final_pose X=%.3f Y=%.3f Z=%.3f A=%.3f B=%.3f C=%.3f\n",
                pose.x, pose.y, pose.z, pose.a, pose.b, pose.c);

    const bool pass = (replies == cycles) && (ipocMismatch == 0);
    std::printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}

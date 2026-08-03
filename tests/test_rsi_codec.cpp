#include <QtTest>
#include "core/RsiCodec.h"

#include <QXmlStreamReader>

#include <cmath>
#include <limits>


class TestRsiCodec : public QObject
{
    Q_OBJECT
private slots:
    void parseRob_readsRistAndIpoc()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1250.5\" Y=\"-10.25\" Z=\"1000.0\" "
                  "A=\"1.5\" B=\"90.0\" C=\"-45.5\"/>"
            "<RSol X=\"1250.0\" Y=\"0.0\" Z=\"1000.0\" "
                  "A=\"0.0\" B=\"90.0\" C=\"0.0\"/>"
            "<Delay D=\"0\"/>"
            "<IPOC>123456789</IPOC>"
            "</Rob>";

        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.rist.x, 1250.5);
        QCOMPARE(f.rist.y, -10.25);
        QCOMPARE(f.rist.z, 1000.0);
        QCOMPARE(f.rist.a, 1.5);
        QCOMPARE(f.rist.b, 90.0);
        QCOMPARE(f.rist.c, -45.5);
        QCOMPARE(f.rsol.x, 1250.0);
        QCOMPARE(f.ipoc, quint64(123456789));
    }

    void parseRob_handlesLargeIpoc()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<IPOC>18446744073709551615</IPOC>"
            "</Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.ipoc, std::numeric_limits<quint64>::max());
    }

    void parseRob_malformedIsInvalidNotCrash()
    {
        QVERIFY(!RsiCodec::parseRob("").valid);
        QVERIFY(!RsiCodec::parseRob("not xml at all").valid);
        QVERIFY(!RsiCodec::parseRob("<Rob><RIst X=\"1\"/></Rob>").valid);
        QVERIFY(!RsiCodec::parseRob("<Rob><IPOC>5</IPOC></Rob>").valid);
        // 截断的 XML
        QVERIFY(!RsiCodec::parseRob("<Rob><RIst X=\"1\" Y=\"2\"").valid);
    }

    void buildSen_echoesIpocVerbatim()
    {
        const Pose k{0.5, -1.25, 2.0, 0.1, -0.2, 0.3};
        const QByteArray s = RsiCodec::buildSen(k, 987654321, "ImFree");
        QVERIFY(s.contains("<Sen Type=\"ImFree\">"));
        QVERIFY(s.contains("<IPOC>987654321</IPOC>"));
        QVERIFY(s.contains("</Sen>"));

        const QByteArray big = RsiCodec::buildSen(
            Pose{}, std::numeric_limits<quint64>::max(), "ImFree");
        QVERIFY(big.contains("<IPOC>18446744073709551615</IPOC>"));
    }

    void buildSen_roundTripsThroughParser()
    {
        // 生成的 RKorr 数值须能被重新读回，验证格式与精度
        const Pose k{0.5, -1.25, 2.0, 0.1, -0.2, 0.3};
        const QByteArray s = RsiCodec::buildSen(k, 42, "ImFree");
        QVERIFY(s.contains("X=\"0.5000\""));
        QVERIFY(s.contains("Y=\"-1.2500\""));
        QVERIFY(s.contains("C=\"0.3000\""));
    }

    void buildSen_zeroKorrIsWellFormed()
    {
        const QByteArray s = RsiCodec::buildSen(Pose{}, 1, "ImFree");
        QVERIFY(s.contains("X=\"0.0000\""));
        QVERIFY(s.contains("<IPOC>1</IPOC>"));
    }

    void buildSen_nonFiniteBecomesZero()
    {
        Pose k{};
        k.x = std::numeric_limits<double>::quiet_NaN();
        k.y = std::numeric_limits<double>::infinity();
        k.z = -std::numeric_limits<double>::infinity();
        k.a = 1.5;
        const QByteArray s = RsiCodec::buildSen(k, 7, "ImFree");
        // 非有限分量必须降级为 0.0000，且绝不出现 nan/inf 字样
        QVERIFY(s.contains("X=\"0.0000\""));
        QVERIFY(s.contains("Y=\"0.0000\""));
        QVERIFY(s.contains("Z=\"0.0000\""));
        QVERIFY(s.contains("A=\"1.5000\""));
        QVERIFY(!s.contains("nan"));
        QVERIFY(!s.contains("inf"));
        QVERIFY(s.contains("<IPOC>7</IPOC>"));
    }

    void parseRob_rejectsNonFiniteRist()
    {
        // 这条用例的前提：Qt 把 "nan"/"inf" 当合法浮点解析（ok 为真）。
        // 先把前提断言出来，否则一旦某天 toDouble 自己开始拒绝它们，
        // 下面的 QVERIFY(!valid) 会因为走了 !ok 分支而空洞通过，
        // isfinite 守卫被删掉都发现不了。
        for (const char *lit : {"nan", "inf", "-inf"}) {
            const QString s = QString::fromLatin1(lit);
            bool ok = false;
            const double parsed = QStringView(s).toDouble(&ok);
            QVERIFY2(!std::isfinite(parsed), lit);
            QVERIFY2(ok, lit);
        }

        // 逐个分量放毒：任何一项非有限都必须拒绝整帧。若只挡 X，
        // 后五项仍是活的缺口。
        static const char *keys[6] = {"X", "Y", "Z", "A", "B", "C"};
        for (const char *poison : {"nan", "inf", "-inf"}) {
            for (int i = 0; i < 6; ++i) {
                QByteArray d = "<Rob Type=\"KUKA\"><RIst";
                for (int k = 0; k < 6; ++k) {
                    d += ' ';
                    d += keys[k];
                    d += "=\"";
                    d += (k == i) ? QByteArray(poison) : QByteArray("1.5");
                    d += '"';
                }
                d += "/><IPOC>77</IPOC></Rob>";
                const RobFrame f = RsiCodec::parseRob(d);
                QVERIFY2(!f.valid, d.constData());
            }
        }

        // 大小写变体也要挡住：Qt 的解析不区分大小写，KRC 侧真出问题时
        // 打印成什么形状不由主机决定。
        const QByteArray up =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"NaN\" Y=\"INF\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<IPOC>77</IPOC></Rob>";
        QVERIFY(!RsiCodec::parseRob(up).valid);
    }

    void parseRob_acceptsFiniteExtremes()
    {
        // 守卫只针对非有限值，不得顺手把合法的极端值也拒掉——
        // 否则真机上一个大坐标就变成"每帧都失败"的全盘故障。
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1e308\" Y=\"-1e308\" Z=\"1e-308\" "
                  "A=\"0\" B=\"-180\" C=\"179.9999\"/>"
            "<IPOC>78</IPOC></Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.rist.x, 1e308);
        QCOMPARE(f.rist.y, -1e308);
        QCOMPARE(f.rist.c, 179.9999);
    }

    void parseRob_nonFiniteRsolDoesNotRejectFrame()
    {
        // RSol 只是诊断字段，解析失败刻意不拒绝整帧（调用点忽略返回值）。
        // 守卫不能把这条语义改掉：RIst 完好时丢一帧只因为额定位姿里有个
        // NaN，等于把一个只影响显示的问题升级成通信降级。
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1250.5\" Y=\"-10.25\" Z=\"1000.0\" "
                  "A=\"1.5\" B=\"90.0\" C=\"-45.5\"/>"
            "<RSol X=\"1250.0\" Y=\"nan\" Z=\"-inf\" "
                  "A=\"0.0\" B=\"90.0\" C=\"0.0\"/>"
            "<IPOC>79</IPOC></Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.rist.x, 1250.5);
        QCOMPARE(f.ipoc, quint64(79));
        // 且非有限值绝不能落进 rsol——解析失败时六项整体保持默认，
        // 不留"前几项新、后几项旧"的拼接位姿。
        const double *v = &f.rsol.x;
        for (int i = 0; i < 6; ++i) {
            QVERIFY(std::isfinite(v[i]));
            QCOMPARE(v[i], 0.0);
        }
    }

    void parseRob_toleratesTrailingPadding()
    {
        QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1250.5\" Y=\"-10.25\" Z=\"1000.0\" "
                  "A=\"1.5\" B=\"90.0\" C=\"-45.5\"/>"
            "<IPOC>555</IPOC>"
            "</Rob>";
        d.append('\0');
        d.append("\0\0", 2);
        const RobFrame f = RsiCodec::parseRob(d);
        // 尾部填充不得导致整帧被拒——那会是每帧都失败的全盘故障
        QVERIFY(f.valid);
        QCOMPARE(f.rist.x, 1250.5);
        QCOMPARE(f.ipoc, quint64(555));

        // 证明这条用例真的走了"reader 报错但仍接受"的路径，而非因为
        // QXmlStreamReader 对尾部 NUL 根本不报错而空洞通过
        QXmlStreamReader probe(d);
        while (!probe.atEnd())
            probe.readNext();
        QVERIFY(probe.hasError());
    }

    void parseRob_rejectsTruncatedIpoc()
    {
        // 纯粹截断在数字末尾：实测 Qt 6.5.3 下 readElementText() 返回空串，
        // 故本形状在加守卫前后都被拒绝——它并不能区分守卫是否存在。
        // 保留它是为了盯住版本漂移：若将来某个 Qt 在纯 EOF 处也吐出部分
        // 数字，本用例会立刻失败。守卫的真正回归锚点是下面的
        // parseRob_rejectsPartialIpocFollowedByMarkup。
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<IPOC>5551";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(!f.valid);
    }

    void parseRob_rejectsPartialIpocFollowedByMarkup()
    {
        // 在 Qt 6.5.3 上，只有当部分数字后面还跟着一个 '<' 时
        // readElementText() 才会真正吐出"已累积的部分字符"（"5551"）并同时
        // 置位 PrematureEndOfDocumentError。这两条输入才是修复前真正能
        // 产生错误 IPOC 回显的形状，故必须单独覆盖，否则回归测试是空洞的。
        const QByteArray d1 =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<IPOC>5551<";
        const QByteArray d2 =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<IPOC>5551</IPOC";
        QVERIFY(!RsiCodec::parseRob(d1).valid);
        QVERIFY(!RsiCodec::parseRob(d2).valid);
    }

    void parseRob_rejectsDamageBeforeIpoc()
    {
        // RIst 完整但 IPOC 开标签本身被截断
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<IPO";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(!f.valid);
    }

    void parseRob_readsDelay()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<Delay D=\"42\"/>"
            "<IPOC>5</IPOC>"
            "</Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.delay, quint64(42));
    }

    void parseRob_missingDelay_isValidDefaultZero()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<IPOC>5</IPOC>"
            "</Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.delay, quint64(0));
    }

    void parseRob_badDelayKeepsDefault()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<Delay D=\"abc\"/>"
            "<IPOC>5</IPOC>"
            "</Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.delay, quint64(0));
    }
};

QTEST_MAIN(TestRsiCodec)
#include "test_rsi_codec.moc"

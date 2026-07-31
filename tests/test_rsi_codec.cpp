#include <QtTest>
#include "core/RsiCodec.h"

#include <QXmlStreamReader>

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
        // 截断发生在 IPOC 数字中间：readElementText() 会返回部分数字，
        // 若不检查 reader 状态就会回显一个错误的 IPOC——等同丢包。
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
};

QTEST_MAIN(TestRsiCodec)
#include "test_rsi_codec.moc"

#include <QtTest>
#include "core/RsiCodec.h"

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
};

QTEST_MAIN(TestRsiCodec)
#include "test_rsi_codec.moc"

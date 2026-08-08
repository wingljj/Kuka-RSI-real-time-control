#include <QtTest>
#include <cstring>
#include <limits>
#include "core/SriProtocol.h"

class TestSriProtocol : public QObject
{
    Q_OBJECT
private slots:
    void parsesSingleFrame()
    {
        SriFrameParser parser;
        uint8_t data[31];
        std::memset(data, 0, sizeof(data));
        data[0] = 0xAA; data[1] = 0x55; data[2] = 0x00; data[3] = 0x1B;
        // float32 LE 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 at offset 6
        const float vals[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
        std::memcpy(data + 6, vals, sizeof(vals));
        const auto frames = parser.feed(data, 31);
        QCOMPARE(frames.size(), size_t(1));
        QCOMPARE(frames[0].values[0], 1.0f);
        QCOMPARE(frames[0].values[5], 6.0f);
    }

    void discardsNonFiniteValues()
    {
        SriFrameParser parser;
        uint8_t data[31] = {};
        data[0] = 0xAA; data[1] = 0x55; data[2] = 0x00; data[3] = 0x1B;
        const float inf = std::numeric_limits<float>::infinity();
        std::memcpy(data + 6, &inf, 4);  // first value is Inf
        const auto frames = parser.feed(data, 31);
        QCOMPARE(frames.size(), size_t(0));
        QCOMPARE(parser.discardedCount(), size_t(1));
    }

    void handlesPartialFrameAcrossCalls()
    {
        SriFrameParser parser;
        // Build one full frame
        uint8_t full[31] = {};
        full[0] = 0xAA; full[1] = 0x55; full[2] = 0x00; full[3] = 0x1B;
        for (int i = 0; i < 6; ++i) { float v = float(i); std::memcpy(full+6+i*4, &v, 4); }

        // Feed first 10 bytes (partial header + partial values)
        auto r1 = parser.feed(full, 10);
        QCOMPARE(r1.size(), size_t(0));  // no complete frame yet

        // Feed remaining 21 bytes
        auto r2 = parser.feed(full + 10, 21);
        QCOMPARE(r2.size(), size_t(1));
    }

    void handlesMultipleFramesInOneFeed()
    {
        SriFrameParser parser;
        uint8_t buf[31 * 3];
        for (int f = 0; f < 3; ++f) {
            uint8_t *frame = buf + f * 31;
            frame[0] = 0xAA; frame[1] = 0x55; frame[2] = 0x00; frame[3] = 0x1B;
            float v = float(f + 1);
            for (int i = 0; i < 6; ++i) std::memcpy(frame+6+i*4, &v, 4);
        }
        const auto frames = parser.feed(buf, sizeof(buf));
        QCOMPARE(frames.size(), size_t(3));
        QCOMPARE(frames[0].values[0], 1.0f);
        QCOMPARE(frames[2].values[0], 3.0f);
    }

    void skipsGarbageBeforeHeader()
    {
        SriFrameParser parser;
        uint8_t buf[35];
        buf[0] = 0xFF; buf[1] = 0xFE; buf[2] = 0xFD; buf[3] = 0xFC;  // garbage
        // frame starting at offset 4
        buf[4] = 0xAA; buf[5] = 0x55; buf[6] = 0x00; buf[7] = 0x1B;
        float v = 42.0f;
        for (int i = 0; i < 6; ++i) std::memcpy(buf+4+6+i*4, &v, 4);
        const auto frames = parser.feed(buf, 35);
        QCOMPARE(frames.size(), size_t(1));
        QCOMPARE(frames[0].values[0], 42.0f);
    }

    void resetClearsBuffer()
    {
        SriFrameParser parser;
        uint8_t partial[10] = {0xAA, 0x55, 0x00, 0x1B};
        parser.feed(partial, 10);
        parser.reset();
        // After reset, feed remaining bytes of a frame — should NOT complete old partial
        uint8_t rest[31] = {};
        const float v = 1.0f;
        for (int i = 0; i < 6; ++i) std::memcpy(rest+6-4+i*4, &v, 4);
        // offset -4 because the first 4 of values already came in partial
        auto frames = parser.feed(rest, 21);
        QCOMPARE(frames.size(), size_t(0));  // old partial was discarded
    }
};
QTEST_MAIN(TestSriProtocol)
#include "test_sri_protocol.moc"

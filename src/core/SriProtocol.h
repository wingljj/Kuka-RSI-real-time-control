#pragma once
#include <array>
#include <cstdint>
#include <vector>

struct SriFrame
{
    float values[6];   // Fx,Fy,Fz,Mx,My,Mz in engineering units
};

// Incrementally extract SRI binary frames from an arbitrary byte stream.
// Thread-safe: call feed() from one thread; no internal shared state beyond buffer.
class SriFrameParser
{
public:
    explicit SriFrameParser(size_t maxBuffer = 16384);

    // Feed raw bytes. Returns all complete frames extracted so far.
    // Non-finite frames (NaN/Inf) are silently discarded.
    std::vector<SriFrame> feed(const uint8_t *data, size_t len);

    // Reset internal buffer (e.g. after reconnection).
    void reset();

    // Number of frames discarded due to NaN/Inf since construction or last reset().
    size_t discardedCount() const { return m_discarded; }

    // Number of bytes dropped due to buffer overflow since construction or last reset().
    size_t overflowCount() const { return m_overflow; }

private:
    static constexpr uint8_t kHeader[4] = {0xAA, 0x55, 0x00, 0x1B};
    static constexpr size_t  kHeaderLen = 4;
    static constexpr size_t  kFrameSize = 31;
    static constexpr size_t  kValueOffset = 6;

    std::vector<uint8_t> m_buf;
    size_t m_maxBuffer;
    size_t m_discarded = 0;
    size_t m_overflow = 0;

    size_t headerPrefixLenAtEnd() const;
};

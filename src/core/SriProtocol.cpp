#include "core/SriProtocol.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>

SriFrameParser::SriFrameParser(size_t maxBuffer)
    : m_maxBuffer(std::max(maxBuffer, kFrameSize))
{
    m_buf.reserve(m_maxBuffer);
}

size_t SriFrameParser::headerPrefixLenAtEnd() const
{
    const size_t maxLen = std::min(m_buf.size(), kHeaderLen - 1);
    for (size_t len = maxLen; len > 0; --len) {
        if (std::memcmp(m_buf.data() + m_buf.size() - len, kHeader, len) == 0)
            return len;
    }
    return 0;
}

void SriFrameParser::reset()
{
    m_buf.clear();
    m_discarded = 0;
    m_overflow = 0;
}

std::vector<SriFrame> SriFrameParser::feed(const uint8_t *data, size_t len)
{
    m_buf.insert(m_buf.end(), data, data + len);

    // Buffer overflow protection: after the parse loop below the buffer is
    // always <= kFrameSize - 1 bytes, so the only unbounded growth is a
    // single oversized feed(). Trim the oldest bytes here, keeping the tail
    // so a frame whose header survives in the retained window still parses.
    if (m_buf.size() > m_maxBuffer) {
        m_buf.erase(m_buf.begin(), m_buf.end() - m_maxBuffer);
        ++m_overflow;
    }

    std::vector<SriFrame> frames;

    while (true) {
        // Find header
        auto it = std::search(m_buf.begin(), m_buf.end(),
                              std::begin(kHeader), std::end(kHeader));
        if (it == m_buf.end()) {
            // Keep only suffix that could be a partial header
            size_t keep = headerPrefixLenAtEnd();
            if (keep) {
                std::memmove(m_buf.data(), m_buf.data() + m_buf.size() - keep, keep);
                m_buf.resize(keep);
            } else {
                m_buf.clear();
            }
            break;
        }

        // Discard data before header
        if (it != m_buf.begin()) {
            m_buf.erase(m_buf.begin(), it);
        }

        if (m_buf.size() < kFrameSize)
            break;

        // Parse frame
        SriFrame f;
        std::memcpy(f.values, m_buf.data() + kValueOffset, sizeof(f.values));

        // Check all values finite
        bool finite = true;
        for (int i = 0; i < 6; ++i) {
            if (!std::isfinite(f.values[i])) {
                finite = false;
                break;
            }
        }

        if (finite) {
            frames.push_back(f);
        } else {
            ++m_discarded;
        }

        m_buf.erase(m_buf.begin(), m_buf.begin() + kFrameSize);
    }

    return frames;
}

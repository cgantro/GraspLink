#pragma once

#include "poselink/streaming/PoseBuffer.h"
#include "poselink/transport/UdpSocket.h"

#include <cstdint>
#include <string>

namespace poselink {
class PoseReceiver {
public:
    explicit PoseReceiver(uint16_t port, size_t bufferCapacity = 256);
    bool Start(std::string& error);
    void Poll();
    bool Sample(uint64_t renderTimeUs, PoseSample& pose, bool& interpolated) const;
    const NetworkMetrics& Metrics() const { return m_metrics; }
    size_t BufferSize() const { return m_buffer.Size(); }
private:
    UdpSocket m_socket;
    uint16_t m_port;
    PoseBuffer m_buffer;
    SequenceAnalyzer m_sequenceAnalyzer;
    NetworkMetrics m_metrics;
};
}

#include "poselink/streaming/PoseReceiver.h"
#include "poselink/transport/Protocol.h"
#include "poselink/pose/Time.h"
#include <vector>
namespace poselink {
PoseReceiver::PoseReceiver(uint16_t port, size_t cap) : m_port(port), m_buffer(cap) {}
bool PoseReceiver::Start(std::string& error) { return m_socket.Bind(m_port, error); }
void PoseReceiver::Poll() { std::string error; std::vector<uint8_t> bytes; while (m_socket.Receive(bytes, 0, error) > 0) { DecodedPacket packet; if (!DecodePose(bytes.data(), bytes.size(), packet, error)) { ++m_metrics.invalid; continue; } m_sequenceAnalyzer.Observe(packet.sequence, SteadyNowUs(), m_metrics); m_buffer.Push(packet.pose); } }
bool PoseReceiver::Sample(uint64_t time, PoseSample& pose, bool& interpolated) const { return m_buffer.Sample(time, pose, interpolated); }
}

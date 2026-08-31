#include "AppModes.h"
#include "poselink/vision/PoseSource.h"
#include "poselink/transport/Protocol.h"
#include "poselink/pose/Time.h"
#include "poselink/transport/UdpSocket.h"
#include <chrono>
#include <iostream>
#include <thread>
int RunVisionMode(int argc, char** argv) {
    std::string host = "127.0.0.1"; uint16_t port = 5000; int rate = 30;
    for (int i = 1; i + 1 < argc; ++i) { std::string a = argv[i]; if (a == "--host") host = argv[++i]; else if (a == "--port") port = uint16_t(std::stoi(argv[++i])); else if (a == "--rate") rate = std::stoi(argv[++i]); }
    std::string error; poselink::UdpSocket socket;
    if (!socket.Connect(host, port, error)) { std::cerr << error << '\n'; return 1; }
    auto source = poselink::MakeSyntheticPoseSource(1); uint32_t sequence = 0;
    const auto period = std::chrono::microseconds(1'000'000 / rate);
    std::cout << "PoseLink vision -> " << host << ':' << port << " at " << rate << " Hz\n";
    for (;;) { const auto packet = poselink::EncodePose(sequence++, source->Next(poselink::SteadyNowUs())); if (!socket.Send(packet.data(), packet.size(), error)) std::cerr << error << '\n'; std::this_thread::sleep_for(period); }
}

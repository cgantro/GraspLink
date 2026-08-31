#include "AppModes.h"
#include "poselink/pose/Time.h"
#include "poselink/transport/UdpSocket.h"
#include <algorithm>
#include <iostream>
#include <random>
#include <vector>
namespace { struct Pending { uint64_t due; std::vector<uint8_t> bytes; }; }
int RunProxyMode(int argc, char** argv) {
    uint16_t listen=5001,target=5000; int delay=0,jitter=0; double loss=0,reorder=0; uint32_t seed=1;
    for (int i=1;i+1<argc;++i) { std::string a=argv[i]; if(a=="--listen")listen=uint16_t(std::stoi(argv[++i]));else if(a=="--target-port")target=uint16_t(std::stoi(argv[++i]));else if(a=="--delay-ms")delay=std::stoi(argv[++i]);else if(a=="--jitter-ms")jitter=std::stoi(argv[++i]);else if(a=="--loss")loss=std::stod(argv[++i]);else if(a=="--reorder")reorder=std::stod(argv[++i]);else if(a=="--seed")seed=uint32_t(std::stoul(argv[++i])); }
    std::string error; poselink::UdpSocket input, output;
    if (!input.Bind(listen,error) || !output.Connect("127.0.0.1",target,error)) { std::cerr<<error<<'\n'; return 1; }
    std::mt19937 rng(seed); std::uniform_real_distribution<double> chance(0,100); std::uniform_int_distribution<int> delta(-jitter,jitter); std::vector<Pending> pending;
    for (;;) { std::vector<uint8_t> bytes; if (input.Receive(bytes,2,error)>0 && chance(rng)>=loss) pending.push_back({poselink::SteadyNowUs()+uint64_t(std::max(0,delay+delta(rng)))*1000,std::move(bytes)}); if(pending.size()>1&&chance(rng)<reorder)std::swap(pending[pending.size()-1],pending[pending.size()-2]); for(auto it=pending.begin();it!=pending.end();) { if(it->due<=poselink::SteadyNowUs()){output.Send(it->bytes.data(),it->bytes.size(),error);it=pending.erase(it);}else ++it; } }
}

#include "poselink/streaming/PoseBuffer.h"
#include "poselink/transport/Protocol.h"
#include <cassert>
#include <iostream>
int main(){poselink::PoseSample p{7,100,{1,2,3},{0,0,0,1},true};auto raw=poselink::EncodePose(12,p);poselink::DecodedPacket decoded;std::string e;assert(poselink::DecodePose(raw.data(),raw.size(),decoded,e));assert(decoded.sequence==12&&decoded.pose.objectId==7);raw[0]=0;assert(!poselink::DecodePose(raw.data(),raw.size(),decoded,e));poselink::PoseBuffer b;b.Push(p);p.timestampUs=200;p.position.x=3;b.Push(p);bool used=false;assert(b.Sample(150,p,used)&&used&&p.position.x==2);poselink::NetworkMetrics m;poselink::SequenceAnalyzer a;a.Observe(1,100,m);a.Observe(3,200,m);a.Observe(2,300,m);assert(m.lost==1&&m.reordered==1);std::cout<<"poselink tests passed\n";}

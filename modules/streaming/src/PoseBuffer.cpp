#include "poselink/streaming/PoseBuffer.h"
#include <algorithm>
#include <cmath>
namespace poselink { 
void SequenceAnalyzer::Observe(uint32_t s,uint64_t now,NetworkMetrics& m){++m.received;if(m.hasSequence){int32_t d=static_cast<int32_t>(s-m.lastSequence);if(d==0)++m.duplicates;else if(d<0)++m.reordered;else if(d>1)m.lost+=uint32_t(d-1);double interval=double(now-m.lastArrivalUs);m.jitterUs+= (std::abs(interval-33333.0)-m.jitterUs)/16.0;}m.lastSequence=s;m.lastArrivalUs=now;m.hasSequence=true;}
PoseBuffer::PoseBuffer(size_t max,uint64_t retention):m_maxSamples(max),m_retentionUs(retention){}
void PoseBuffer::Push(const PoseSample& p){auto i=std::upper_bound(m_samples.begin(),m_samples.end(),p.timestampUs,[](uint64_t t,const PoseSample& v){return t<v.timestampUs;});m_samples.insert(i,p);while(m_samples.size()>m_maxSamples)m_samples.pop_front();while(m_samples.size()>1&&m_samples.back().timestampUs-m_samples.front().timestampUs>m_retentionUs)m_samples.pop_front();}
Vec3 Lerp(const Vec3&a,const Vec3&b,double t){return {a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t};}
Quaternion Slerp(const Quaternion&a,const Quaternion&b,double t){Quaternion q=b;double dot=a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;if(dot<0){dot=-dot;q={-b.x,-b.y,-b.z,-b.w};}if(dot>.9995){Quaternion r{a.x+t*(q.x-a.x),a.y+t*(q.y-a.y),a.z+t*(q.z-a.z),a.w+t*(q.w-a.w)};double n=std::sqrt(r.x*r.x+r.y*r.y+r.z*r.z+r.w*r.w);return {r.x/n,r.y/n,r.z/n,r.w/n};}double theta=std::acos(dot),s=std::sin(theta);double x=std::sin((1-t)*theta)/s,y=std::sin(t*theta)/s;return {a.x*x+q.x*y,a.y*x+q.y*y,a.z*x+q.z*y,a.w*x+q.w*y};}
bool PoseBuffer::Sample(uint64_t t,PoseSample& out,bool& inter) const{inter=false;if(m_samples.empty())return false;auto b=std::upper_bound(m_samples.begin(),m_samples.end(),t,[](uint64_t v,const PoseSample&p){return v<p.timestampUs;});if(b==m_samples.begin()){out=*b;return true;}if(b==m_samples.end()){out=m_samples.back();return true;}auto a=std::prev(b);double f=double(t-a->timestampUs)/double(b->timestampUs-a->timestampUs);out=*a;out.position=Lerp(a->position,b->position,f);out.orientation=Slerp(a->orientation,b->orientation,f);inter=true;return true;}
}

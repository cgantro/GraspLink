#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace poselink { class UdpSocket { public: UdpSocket(); ~UdpSocket(); UdpSocket(const UdpSocket&)=delete; UdpSocket& operator=(const UdpSocket&)=delete; bool Bind(uint16_t port,std::string& error); bool Connect(const std::string& host,uint16_t port,std::string& error); bool Send(const uint8_t* data,size_t size,std::string& error) const; int Receive(std::vector<uint8_t>& output,int timeoutMs,std::string& error) const; bool Valid() const; private: intptr_t m_handle{-1}; }; }

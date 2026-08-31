#include "poselink/transport/UdpSocket.h"
#include <cstring>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
namespace poselink { namespace { 
#ifdef _WIN32
using Socket=SOCKET; constexpr Socket Invalid=INVALID_SOCKET; void Close(Socket s){closesocket(s);} struct Wsa{Wsa(){WSADATA d;WSAStartup(MAKEWORD(2,2),&d);}~Wsa(){WSACleanup();}}; Wsa g_wsa;
#else
using Socket=int; constexpr Socket Invalid=-1; void Close(Socket s){close(s);}
#endif
std::string Error(){ return "socket operation failed"; } }
UdpSocket::UdpSocket()=default; UdpSocket::~UdpSocket(){if(Valid())Close(Socket(m_handle));}
bool UdpSocket::Valid() const{return m_handle!=intptr_t(Invalid);} 
bool UdpSocket::Bind(uint16_t port,std::string& e){Socket s=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);if(s==Invalid){e=Error();return false;} sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_ANY);a.sin_port=htons(port);if(bind(s,reinterpret_cast<sockaddr*>(&a),sizeof(a))!=0){e=Error();Close(s);return false;}m_handle=intptr_t(s);return true;}
bool UdpSocket::Connect(const std::string& host,uint16_t port,std::string& e){Socket s=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);if(s==Invalid){e=Error();return false;} sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);if(inet_pton(AF_INET,host.c_str(),&a.sin_addr)!=1){e="host must be a numeric IPv4 address";Close(s);return false;}if(connect(s,reinterpret_cast<sockaddr*>(&a),sizeof(a))!=0){e=Error();Close(s);return false;}m_handle=intptr_t(s);return true;}
bool UdpSocket::Send(const uint8_t* d,size_t n,std::string& e) const{if(!Valid()){e="socket is closed";return false;}int sent=send(Socket(m_handle),reinterpret_cast<const char*>(d),int(n),0);if(sent!=int(n)){e=Error();return false;}return true;}
int UdpSocket::Receive(std::vector<uint8_t>& out,int ms,std::string& e) const{if(!Valid()){e="socket is closed";return -1;}fd_set f;FD_ZERO(&f);FD_SET(Socket(m_handle),&f);timeval tv{ms/1000,(ms%1000)*1000};int ready=select(int(Socket(m_handle))+1,&f,nullptr,nullptr,&tv);if(ready==0)return 0;if(ready<0){e=Error();return -1;}out.resize(2048);int got=recv(Socket(m_handle),reinterpret_cast<char*>(out.data()),int(out.size()),0);if(got<0){e=Error();return -1;}out.resize(size_t(got));return got;}
}

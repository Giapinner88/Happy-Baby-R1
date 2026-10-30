#pragma once
// Project-owned overlay. Explicit environment opt-in; loopback only.
// P4Y1 uint32 magic, uint32 seq, float yaw rad/s, uint32 enable (little endian).
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>

class SearchYawReceiver {
public:
    struct Packet {uint32_t magic,seq;float yaw;uint32_t enable;};
    static_assert(sizeof(Packet)==16,"packet size");
    ~SearchYawReceiver(){if(fd_>=0)::close(fd_);}
    bool Apply(float& vx,float& vy,float& yaw,bool manual,bool active,bool emergency) {
        using Clock=std::chrono::steady_clock;
        const auto now=Clock::now();
        if(!initialized_){
            initialized_=true;const char* value=std::getenv("P4_YAW_PORT");
            if(!value)return false;
            char* end=nullptr;long port=std::strtol(value,&end,10);
            if(*end || port<1024 || port>65535)throw std::runtime_error("invalid P4_YAW_PORT");
            fd_=::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
            sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);addr.sin_port=htons(port);
            if(fd_<0 || ::bind(fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0)
                throw std::runtime_error("P4 yaw bind failed");
        }
        if(fd_<0)return false;
        if(std::chrono::duration<double>(now-last_).count()>.25)have_=false;
        Packet p{};sockaddr_in src{};socklen_t len=sizeof(src);
        for(int i=0;i<64;++i){
            const auto n=::recvfrom(fd_,&p,sizeof(p),MSG_TRUNC,reinterpret_cast<sockaddr*>(&src),&len);
            if(n<0)break;
            if(n!=sizeof(p) || ntohl(src.sin_addr.s_addr)!=INADDR_LOOPBACK || p.magic!=0x31593450 ||
               p.enable>1 || !std::isfinite(p.yaw) || std::abs(p.yaw)>0.7853982f)continue;
            if(have_ && static_cast<int32_t>(p.seq-seq_)<=0)continue;
            seq_=p.seq;have_=true;last_=now;rate_=p.enable?p.yaw:0.f;
        }
        if(!have_ || manual || !active || emergency)return false;
        vx=vy=0.f;yaw=rate_;return true;
    }
private:
    bool initialized_=false,have_=false;int fd_=-1;uint32_t seq_=0;float rate_=0;
    std::chrono::steady_clock::time_point last_{};
};

#pragma once
// APV1 robot-local protocol. No DDS publisher, no arming/mode changes.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

namespace approach_owner {
constexpr uint32_t kMagic=0x31565041;
// Receiver ceiling, not a validated walking speed or stride bound.
constexpr float kMaxForwardMps=.20f;
enum Kind:uint16_t { Hello=1, Command=2, Stop=3, Ping=4 };
enum Status:uint16_t { Ready=1, Stored=2, Stopped=3, Pong=4, Invalid=10, Gate=11, Session=12, Sequence=13, Expired=14, ZeroOnly=15, Slew=16 };
struct Packet { uint32_t magic; uint16_t version,kind; uint64_t session,sequence,deadline_us; float vx,wz; };
struct Ack { uint32_t magic; uint16_t version,status; uint64_t session,sequence,owner_us; float requested_vx,requested_wz; };
static_assert(sizeof(Packet)==40 && sizeof(Ack)==40,"APV1 wire layout");
inline uint64_t Now(){return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}

class Lease {
public:
    Ack Receive(const Packet& p,uint64_t now,bool eligible,bool manual,bool emergency,bool allow_nonzero) {
        Tick(now,eligible,manual,emergency);
        Ack a{kMagic,1,Invalid,p.session,p.sequence,now,0,0};
        if(p.magic!=kMagic || p.version!=1 || !p.session || !std::isfinite(p.vx) || !std::isfinite(p.wz) ||
           p.vx<0 || p.vx>kMaxForwardMps || std::abs(p.wz)>.5f)return a;
        if(p.deadline_us<=now || p.deadline_us-now>150000){a.status=Expired;return a;}
        if(p.kind==Ping){if(p.vx==0 && p.wz==0)a.status=Pong;return a;}
        if(p.kind==Stop){
            if(p.vx || p.wz)return a;
            // A zero STOP may always suppress the adapter; it cannot arm it.
            armed_=false;vx_=wz_=0;owned_=true;a.status=Stopped;return a;
        }
        if(!eligible || manual || emergency){a.status=Gate;return a;}
        if(p.kind==Hello){
            if(p.vx || p.wz)return a;
            if(armed_ || p.session==session_){a.status=Session;return a;}
            session_=p.session;sequence_=p.sequence;deadline_=p.deadline_us;
            velocity_time_=now;
            armed_=owned_=true;vx_=wz_=0;a.status=Ready;return a;
        }
        if(p.kind!=Command)return a;
        if(!armed_ || p.session!=session_){a.status=Session;return a;}
        if(p.sequence<=sequence_){a.status=Sequence;return a;}
        if(!allow_nonzero && (p.vx!=0 || p.wz!=0)){a.status=ZeroOnly;return a;}
        const float dt=static_cast<float>(now-velocity_time_)*1e-6f;
        if((p.vx!=0 || p.wz!=0) && (p.vx>vx_+.2f*dt+1e-6f || std::abs(p.wz-wz_)>dt+1e-6f)){
            a.status=Slew;return a;
        }
        sequence_=p.sequence;deadline_=p.deadline_us;vx_=p.vx;wz_=p.wz;
        velocity_time_=now;
        a.status=Stored;a.requested_vx=vx_;a.requested_wz=wz_;return a;
    }
    bool Tick(uint64_t now,bool eligible,bool manual,bool emergency){
        if(armed_ && (now>=deadline_ || !eligible || manual || emergency)){
            armed_=false;vx_=wz_=0; // latched: another nonce/Hello is required
        }
        return owned_ && eligible && !manual && !emergency;
    }
    float vx()const{return armed_?vx_:0.f;}
    float wz()const{return armed_?wz_:0.f;}
private:
    bool armed_=false,owned_=false;
    uint64_t session_=0,sequence_=0,deadline_=0,velocity_time_=0;
    float vx_=0,wz_=0;
};

class Receiver {
public:
    ~Receiver(){if(fd_>=0)::close(fd_);}
    bool Apply(float& vx,float& vy,float& wz,bool manual,bool eligible,bool emergency){
        if(!initialized_){
            initialized_=true;const char* value=std::getenv("R1_APPROACH_PORT");
            std::string file_port;bool file_zero_only=false;
            if(!value){
                // HB preflight forbids systemd drop-ins. Dedicated root-owned
                // opt-in file avoids changing service or the secret stack.env.
                std::ifstream config("/etc/hb/approach_owner.port");
                if(!config.good())return false;
                std::string extra;
                if(!(config>>file_port) || (config>>extra))throw std::runtime_error("invalid approach port file");
                value=file_port.c_str();file_zero_only=true;
            }
            char* end=nullptr;long port=std::strtol(value,&end,10);
            if(*end || port<1024 || port>65535)throw std::runtime_error("invalid R1_APPROACH_PORT");
            const char* enable=std::getenv("R1_APPROACH_ALLOW_NONZERO");allow_nonzero_=!file_zero_only && enable && std::string(enable)=="1";
            fd_=::socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
            sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);address.sin_port=htons(port);
            if(fd_<0 || ::bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0)throw std::runtime_error("approach bind failed");
        }
        if(fd_<0)return false;
        const auto now=Now();
        for(int i=0;i<16;++i){
            Packet p{};sockaddr_in peer{};socklen_t size=sizeof(peer);
            auto n=::recvfrom(fd_,&p,sizeof(p),MSG_TRUNC,reinterpret_cast<sockaddr*>(&peer),&size);
            if(n<0)break;
            if(n!=sizeof(p) || ntohl(peer.sin_addr.s_addr)!=INADDR_LOOPBACK)continue;
            // Pin command peer as well as nonce; Ping/Stop do not acquire ownership.
            if(peer_set_ && p.kind!=Ping && p.kind!=Stop && p.kind!=Hello &&
               (peer.sin_port!=peer_.sin_port || peer.sin_addr.s_addr!=peer_.sin_addr.s_addr))continue;
            auto ack=lease_.Receive(p,now,eligible,manual,emergency,allow_nonzero_);
            if(ack.status==Ready){peer_=peer;peer_set_=true;}
            ::sendto(fd_,&ack,sizeof(ack),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&peer),size);
        }
        if(!lease_.Tick(now,eligible,manual,emergency))return false;
        vx=lease_.vx();vy=0;wz=lease_.wz();return true;
    }
private:
    int fd_=-1;bool initialized_=false,allow_nonzero_=false,peer_set_=false;
    sockaddr_in peer_{};Lease lease_;
};
} // namespace approach_owner

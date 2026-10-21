#include "../src/safety/StateMotionGuard.hpp"
#include <iostream>
#include <array>
#include <vector>
#include "../src/safety/DanceUprightWindow.hpp"
#include <limits>
using namespace motion_safety;
static int checks=0;
static void Check(bool v) {++checks;if(!v)throw std::runtime_error("Motion safety check "+std::to_string(checks));}
static Eigen::Quaternionf Rx(float deg){return Eigen::Quaternionf(Eigen::AngleAxisf(deg/kRadToDeg,Eigen::Vector3f::UnitX()));}
static Eigen::Quaternionf Rz(float deg){return Eigen::Quaternionf(Eigen::AngleAxisf(deg/kRadToDeg,Eigen::Vector3f::UnitZ()));}
const auto I=Eigen::Quaternionf::Identity();
const auto Z=Eigen::Vector3f::Zero().eval();
struct Driver {
    Guard guard;uint32_t tick=100;double wall=0;
    bool Step(Mode mode,float actual,float reference=0,float speed=0) {
        tick+=20;wall+=.02;
        if(guard.Sensor(tick,wall,Rx(actual),Z,true,speed))return true;
        return guard.Pose(mode,Rx(actual),Z,Rx(actual),Rx(reference));
    }
};
struct MotionProbe {
    std::vector<Eigen::Quaternionf> quat = std::vector<Eigen::Quaternionf>(100, I);
    std::vector<std::array<float,24>> dq{100};
    int num_frames() const { return quat.size(); }
    float fps() const { return 50; }
    const auto& torso_quat(int f) const { return quat[f]; }
    const auto& joint_vel(int f) const { return dq[f]; }
};
int main(){
    for(float a:{0.f,45.f,90.f,135.f,180.f,-170.f}) {
        Check(std::abs(TiltErrorDeg(Rx(a),Rx(a)))<.04f);
        Check(std::abs(TiltErrorDeg(Rz(120)*Rx(a),Rx(a)))<.04f);
        auto q=Rx(a);q.coeffs()*=-1;Check(std::abs(TiltErrorDeg(q,Rx(a)))<.04f);
    }
    Check(std::abs(TiltErrorDeg(Rx(45),Rx(-45))-90)<.04f);
    Check(std::abs(TiltErrorDeg(Rx(110),Rx(100))-10)<.04f);
    Check(std::isnan(TiltErrorDeg(Eigen::Quaternionf(0,0,0,0),I)));
    {Driver d;for(int i=0;i<200;++i)Check(!d.Step(Mode::Dance,170,170));}
    {Driver d;for(int i=0;i<5;++i)Check(!d.Step(Mode::Dance,46));Check(d.Step(Mode::Dance,46));Check(std::string(d.guard.Reason())=="dance_tracking");Check(d.Step(Mode::Dance,0));}
    {Driver d;for(int i=0;i<100;++i)Check(!d.Step(Mode::Dance,44.9f));}
    {Driver d;for(int i=0;i<5;++i)Check(!d.Step(Mode::Dance,50));Check(!d.Step(Mode::Dance,0));for(int i=0;i<5;++i)Check(!d.Step(Mode::Dance,50));Check(d.Step(Mode::Dance,50));}
    {Driver d;Check(!d.Step(Mode::Upright,60));Check(!d.Step(Mode::Upright,60));Check(d.Step(Mode::Upright,60));}
    {Driver d;for(int i=0;i<5;++i)Check(!d.Step(Mode::Dance,46));Check(!d.Step(Mode::Upright,0));Check(!d.Step(Mode::Dance,46));}
    {Driver d;Check(!d.Step(Mode::Dance,170,170,26));Check(!d.Step(Mode::Dance,170,170,26));Check(d.Step(Mode::Dance,170,170,26));Check(std::string(d.guard.Reason())=="joint_speed");}
    {Guard g;Check(!g.Sensor(0,0,I,Z,true,0));Check(g.Sensor(0,.101,I,Z,true,0));Check(std::string(g.Reason())=="stale_state");Check(g.Sensor(200,.102,I,Z,true,0));}
    {Guard g;Check(!g.Sensor(0,0,I,Z,true,0));Check(g.Sensor(20,.101,I,Z,true,0));}
    {Guard g;Check(!g.Sensor(100,0,I,Z,true,0));Check(g.Sensor(0,.02,I,Z,true,0));}
    {Guard g;Check(!g.Sensor(UINT32_MAX-5,0,I,Z,true,0));Check(!g.Sensor(14,.02,I,Z,true,0));}
    {Guard g;Check(!g.Sensor(0,0,I,Z,true,0));Check(g.Sensor(20,-1,I,Z,true,0));}
    {Guard g;Check(g.Sensor(0,0,I,Z,false,0));}
    {Guard g;Check(g.Sensor(0,0,Eigen::Quaternionf(0,0,0,0),Z,true,0));}
    {Guard g;Check(g.Sensor(0,0,I,Eigen::Vector3f(NAN,0,0),true,0));}
    {Guard g;Check(!g.Sensor(0,0,I,Z,true,0));Check(g.Pose(Mode::Dance,I,Z,I,Eigen::Quaternionf(0,0,0,0)));}
    // Duplicate and high-frequency DDS samples must not extend pose debounce.
    {Guard g;Check(!g.Sensor(0,0,I,Z,true,0));Check(!g.Pose(Mode::Dance,I,Z,Rx(60),I));
      for(int i=0;i<20;++i){Check(!g.Sensor(0,.001*i,I,Z,true,0));Check(!g.Pose(Mode::Dance,I,Z,Rx(60),I));}}
    {Driver d;for(int i=0;i<5;++i)Check(!d.Step(Mode::Dance,46));Check(!d.Step(Mode::Posture,0));Check(!d.Step(Mode::Dance,46));}
    {Guard g;auto gyro=Eigen::Vector3f(5,0,0);Check(!g.Sensor(0,0,Rx(35),gyro,true,0));Check(!g.Pose(Mode::Upright,Rx(35),gyro,I,I));Check(!g.Sensor(20,.02,Rx(35),gyro,true,0));Check(!g.Pose(Mode::Upright,Rx(35),gyro,I,I));Check(!g.Sensor(40,.04,Rx(35),gyro,true,0));Check(g.Pose(Mode::Upright,Rx(35),gyro,I,I));Check(std::string(g.Reason())=="upright_fast_flip");}
    {for(float value:{NAN,0.f,200.f}){Config c;c.dance_deg=value;bool threw=false;try{Guard g(c);}catch(...){threw=true;}Check(threw);}}
    // Reference-only terminal detection must exclude standing pauses before a flip.
    {MotionProbe m;Check(UprightWindow::StartFrame(m)==0);
     m.quat[70]=Rx(170);Check(UprightWindow::StartFrame(m)==71);
     m.quat[99]=Rx(20);Check(UprightWindow::StartFrame(m)==100);}
    {MotionProbe m;m.dq[90][3]=1;Check(UprightWindow::StartFrame(m)==100);}
    {MotionProbe m;m.dq[20][18]=5;Check(UprightWindow::StartFrame(m)==0);
     m.dq[20][12]=1;Check(UprightWindow::StartFrame(m)==21);}
    {MotionProbe m;m.quat[20].coeffs()[0]=NAN;Check(UprightWindow::StartFrame(m)==21);}
    // Identical intentional inverted motion remains legal with the new checks absent.
    {Guard g;for(int k=0;k<100;++k){auto q=Rx(170);auto gyro=Eigen::Vector3f(5,0,0);
      Check(!g.Sensor(100+2*k,.002*k,q,gyro,true,0));Check(!g.Pose(Mode::Dance,q,gyro,q,q,false));}}
    // A terminal upright flip is cut after exactly 30 ms, even below 45 degrees.
    {Guard g;auto gyro=Eigen::Vector3f(5,0,0);for(int k=0;k<=15;++k){
      Check(!g.Sensor(100+2*k,.002*k,Rx(35),gyro,true,0));
      Check(g.Pose(Mode::Dance,Rx(35),gyro,Rx(35),I,true)==(k==15));}
      Check(std::string(g.Reason())=="dance_upright_fast_flip");}
    // Quiet but severely tilted terminal posture has its own 30 ms limit.
    {Guard g;for(int k=0;k<=15;++k){Check(!g.Sensor(100+2*k,.002*k,Rx(55),Z,true,0));
      Check(g.Pose(Mode::Dance,Rx(55),Z,Rx(55),I,true)==(k==15));}
      Check(std::string(g.Reason())=="dance_upright_tilt");}
    // No repeated-sample dwell credit; no root-vs-torso mismatch on a waist bend.
    {Guard g;auto gyro=Eigen::Vector3f(5,0,0);Check(!g.Sensor(100,0,Rx(35),gyro,true,0));
     for(int k=0;k<50;++k)Check(!g.Pose(Mode::Dance,Rx(35),gyro,Rx(35),I,true));}
    {Guard g;auto gyro=Eigen::Vector3f(5,0,0);for(int k=0;k<100;++k){
      Check(!g.Sensor(100+2*k,.002*k,Rx(35),gyro,true,0));Check(!g.Pose(Mode::Dance,Rx(35),gyro,I,I,true));}}
    // Leaving the terminal context resets the fast-check dwell; boundary resets it too.
    {Guard g;auto gyro=Eigen::Vector3f(5,0,0);for(int k=0;k<15;++k){
      Check(!g.Sensor(100+2*k,.002*k,Rx(35),gyro,true,0));Check(!g.Pose(Mode::Dance,Rx(35),gyro,Rx(35),I,k!=7));}
     g.PolicyBoundary();Check(!g.Sensor(132,.032,Rx(35),gyro,true,0));Check(!g.Pose(Mode::Dance,Rx(35),gyro,Rx(35),I,true));}
    std::cout<<"STATE_MOTION_GUARD_OK checks="<<checks<<"\n";
}

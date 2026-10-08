#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <vector>
#include <unistd.h>
#include <cnpy.h>
#include "motion/JointTrajectory.hpp"
#include "motion/MotionData.hpp"

void Require(bool ok) { if (!ok) std::abort(); }
template<class Fn> void Reject(Fn fn) {
    bool rejected = false;
    try { fn(); } catch (const std::exception&) { rejected = true; }
    Require(rejected);
}

int main() {
    char tmp[] = "/tmp/hb-motion-validation.XXXXXX";
    Require(mkdtemp(tmp) != nullptr);
    const std::string path = std::string(tmp) + "/fixture.npz";
    std::vector<double> jp(48, 0.125), jv(48, 0.0), torso(8, 0.0);
    const size_t bodies = spec::kMotionTorsoIdx + 1;
    std::vector<double> bq(2 * bodies * 4, 0.0);
    for (size_t i=0; i<bq.size(); i+=4) bq[i]=1.0;
    torso[0]=torso[4]=1.0;
    float fps = 60.0f;
    auto write = [&]() {
        cnpy::npz_save(path, "joint_pos", jp.data(), {2,24}, "w");
        cnpy::npz_save(path, "joint_vel", jv.data(), {2,24}, "a");
        cnpy::npz_save(path, "torso_quat", torso.data(), {2,4}, "a");
        cnpy::npz_save(path, "body_quat_w", bq.data(), {2,bodies,4}, "a");
        cnpy::npz_save(path, "fps", &fps, {1}, "a");
    };
    JointTrajectory trajectory;
    MotionData motion;
    write(); trajectory.Load(path); motion.Load(path);
    Require(trajectory.fps()==60.0f && motion.fps()==60.0f);
    Require(trajectory.frame(1)[23]==0.125f && motion.joint_pos(1)[23]==0.125f);
    Require(std::abs(trajectory.RefGravityAt(0).z()+1.0f)<1e-6f);
    for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(), 1e300}) {
        jp[10]=bad; write();
        Reject([&]{trajectory.Load(path);}); Require(trajectory.num_frames()==0);
        Reject([&]{motion.Load(path);}); Require(motion.num_frames()==0);
    }
    jp[10]=0.125;
    for (float bad : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()}) {
        fps=bad; write();
        Reject([&]{trajectory.Load(path);}); Reject([&]{motion.Load(path);});
    }
    fps=60.0f;
    torso[0]=0.0; write(); Reject([&]{trajectory.Load(path);}); torso[0]=1.0;
    bq[spec::kMotionTorsoIdx*4]=0.0; write(); Reject([&]{motion.Load(path);});
    bq[spec::kMotionTorsoIdx*4]=1.0;
    // i4 and f4 have identical width: test actual NPY dtype, not just word_size.
    write(); std::vector<int32_t> ints(48, 1);
    cnpy::npz_save(path, "joint_pos", ints.data(), {2,24}, "a");
    Reject([&]{trajectory.Load(path);}); Reject([&]{motion.Load(path);});
    write(); double head[4]={0,0,0,std::numeric_limits<double>::quiet_NaN()};
    cnpy::npz_save(path, "head_pos", head, {2,2}, "a");
    Reject([&]{trajectory.Load(path);});
    write(); double empty=0;
    cnpy::npz_save(path, "fps", &empty, {0}, "a");
    Reject([&]{trajectory.Load(path);}); Reject([&]{motion.Load(path);});
    std::filesystem::remove(path);
    std::filesystem::remove(tmp);
    std::cout << "MOTION_VALIDATION_OK\n";
}

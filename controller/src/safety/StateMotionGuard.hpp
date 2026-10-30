#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <Eigen/Geometry>

namespace motion_safety {
constexpr float kRadToDeg = 57.29577951308232f;
enum class Mode { Upright, Dance, Posture };
inline const char* Name(Mode m) { return m == Mode::Dance ? "dance" : m == Mode::Upright ? "upright" : "posture"; }
struct Config {
    float dance_deg = 45, dance_dwell_s = .10f;
    float upright_deg = 50, upright_dwell_s = .03f;
    float flip_deg = 30, flip_gyro = 4;
    float joint_speed = 25, joint_dwell_s = .03f;
    float timeout_s = .10f;
    void Validate() const {
        auto range=[](float x,float lo,float hi){return std::isfinite(x)&&x>=lo&&x<=hi;};
        if (!range(dance_deg,1,90)||!range(dance_dwell_s,.02f,.5f)||
            !range(upright_deg,1,80)||!range(upright_dwell_s,.02f,.5f)||
            !range(flip_deg,1,upright_deg)||!range(flip_gyro,.1f,20)||
            !range(joint_speed,1,40)||!range(joint_dwell_s,.02f,.5f)||!range(timeout_s,.04f,.5f))
            throw std::invalid_argument("Invalid state motion safety configuration");
    }
};
inline bool ValidQuat(const Eigen::Quaternionf& q) {
    const float n=q.norm();
    return q.coeffs().allFinite()&&std::isfinite(n)&&n>=.5f&&n<=2.f;
}
// Both quaternions map the SAME body (torso) to world. Gravity in body axes
// intentionally ignores global yaw; do not subtract Euler tilt magnitudes.
inline float TiltErrorDeg(const Eigen::Quaternionf& actual, const Eigen::Quaternionf& reference) {
    if (!ValidQuat(actual)||!ValidQuat(reference)) return NAN;
    const auto down=Eigen::Vector3f(0,0,-1);
    const Eigen::Vector3f a=actual.normalized().conjugate()*down;
    const Eigen::Vector3f b=reference.normalized().conjugate()*down;
    return std::acos(std::clamp(a.dot(b),-1.f,1.f))*kRadToDeg;
}
class Guard {
public:
    explicit Guard(Config c = {}) : c_(c) { c_.Validate(); }
    bool Trip(const char* why) { if (!reason_) reason_=why; return true; }
    void PolicyBoundary() { orientation_bad_=flip_bad_=upright_bad_=0;orientation_active_=flip_active_=upright_active_=false;have_mode_=false; }
    bool Latched() const { return reason_ != nullptr; }
    const char* Reason() const { return reason_ ? reason_ : "none"; }
    float SampleDt() const { return sample_dt_; }
    float PoseDt() const { return pose_dt_; }
    float ErrorDeg() const { return error_deg_; }
    float BadSeconds() const { return std::max({orientation_bad_, flip_bad_, upright_bad_}); }
    double ProgressAgeSeconds() const { return progress_age_s_; }
    double ObservationGapSeconds() const { return observation_gap_s_; }
    uint32_t TickAdvance() const { return tick_advance_; }
    // Runtime supplies the receive timestamp of the last changed physical tick,
    // captured with this sample. Without it, retain the conservative sync guard.
    bool Sensor(uint32_t tick, double wall_s, const Eigen::Quaternionf& root,
                const Eigen::Vector3f& gyro, bool joints_finite, float max_dq, double received_progress_s = -1) {
        sample_dt_=0;
        if (Latched()) return true;
        if (!std::isfinite(wall_s)||!ValidQuat(root)||!gyro.allFinite()||
            !joints_finite||!std::isfinite(max_dq)) return Trip("invalid_state");
        observation_gap_s_ = initialized_ ? wall_s-wall_ : 0;
        tick_advance_ = initialized_ ? tick-tick_ : 0;
        const bool receive_clock = received_progress_s != -1;
        if (receive_clock && (!std::isfinite(received_progress_s) || received_progress_s < 0 ||
                              received_progress_s > wall_s)) return Trip("invalid_state");
        progress_age_s_ = receive_clock ? wall_s-received_progress_s :
                          initialized_ ? wall_s-progress_wall_ : 0;
        if (receive_clock && progress_age_s_ > c_.timeout_s+1e-7) return Trip("stale_state");
        if (!initialized_) {
            initialized_=true; tick_=tick; wall_=progress_wall_=wall_s; sample_dt_=.02f;
        } else {
            if (wall_s<wall_) return Trip("clock_reversed");
            wall_=wall_s;
            const uint32_t advance=tick-tick_;
            if (advance>=0x80000000u) return Trip("state_clock_reversed");
            // A delayed consumer may resume with feedback that remained fresh
            // in the DDS callback. Do not confuse its scheduling gap with age.
            if (!receive_clock && progress_age_s_>c_.timeout_s+1e-7) return Trip("stale_state");
            if (advance>0) {
                sample_dt_=std::min(.02f,advance*.001f);
                tick_=tick; progress_wall_=wall_s;
            }
        }
        Advance(max_dq>c_.joint_speed,sample_dt_,speed_bad_,speed_active_);
        if (speed_bad_+1e-6f>=c_.joint_dwell_s) return Trip("joint_speed");
        return false;
    }
    // Called once per new evaluated physical sample, using the last reference
    // actually commanded. Warmup/cooldown use their interpolated references.
    bool Pose(Mode mode, const Eigen::Quaternionf& root, const Eigen::Vector3f& gyro,
              const Eigen::Quaternionf& actual, const Eigen::Quaternionf& reference,
              bool dance_upright = false) {
        if (Latched()) return true;
        if (!have_mode_||mode!=mode_) {
            have_mode_=true;mode_=mode;orientation_bad_=flip_bad_=upright_bad_=0;orientation_active_=flip_active_=upright_active_=false;
        }
        if (!ValidQuat(root)||!gyro.allFinite()) return Trip("invalid_state");
        if (mode==Mode::Posture) { error_deg_=0; return false; }
        error_deg_=TiltErrorDeg(mode==Mode::Dance?actual:root,
                               mode==Mode::Dance?reference:Eigen::Quaternionf::Identity());
        if (!std::isfinite(error_deg_)) return Trip("invalid_reference");
        const float pose_dt = pose_dt_ = have_pose_tick_ ? std::min(.02f, (tick_-pose_tick_)*.001f) : .02f;
        have_pose_tick_=true; pose_tick_=tick_;
        const float limit=mode==Mode::Dance?c_.dance_deg:c_.upright_deg;
        Advance(error_deg_>limit,pose_dt,orientation_bad_,orientation_active_);
        const float dwell=mode==Mode::Dance?c_.dance_dwell_s:c_.upright_dwell_s;
        // A terminal upright dance retains reference tracking AND gains the fast
        // upright checks. Evaluate the same torso as the reference, not pelvis
        // tilt, so an intentional waist bend is not mistaken for a fall.
        const bool upright_dance = mode == Mode::Dance && dance_upright;
        const float upright_error = upright_dance
            ? TiltErrorDeg(actual, Eigen::Quaternionf::Identity()) : error_deg_;
        Advance(upright_dance && upright_error > c_.upright_deg,
                pose_dt, upright_bad_, upright_active_);
        Advance((mode==Mode::Upright || upright_dance)&&upright_error>c_.flip_deg&&gyro.norm()>c_.flip_gyro,
                pose_dt,flip_bad_,flip_active_);
        if (orientation_bad_+1e-6f>=dwell) return Trip(mode==Mode::Dance?"dance_tracking":"upright_tilt");
        if (upright_bad_+1e-6f>=c_.upright_dwell_s) return Trip("dance_upright_tilt");
        if (flip_bad_+1e-6f>=c_.upright_dwell_s)
            return Trip(upright_dance ? "dance_upright_fast_flip" : "upright_fast_flip");
        return false;
    }
private:
    static void Advance(bool bad,float dt,float& elapsed,bool& active) {
        // No credit for time preceding the first observed bad sample.
        if (!bad) {elapsed=0;active=false;}
        else {elapsed=active?elapsed+dt:0;active=true;}
    }
    bool orientation_active_=false,flip_active_=false,upright_active_=false,speed_active_=false;
    Config c_;
    const char* reason_=nullptr;
    bool initialized_=false,have_mode_=false;
    bool have_pose_tick_=false;
    uint32_t tick_=0, pose_tick_=0;
    double wall_=0,progress_wall_=0;
    double progress_age_s_=0,observation_gap_s_=0;
    uint32_t tick_advance_=0;
    Mode mode_=Mode::Upright;
    float pose_dt_=0,sample_dt_=0,speed_bad_=0,orientation_bad_=0,flip_bad_=0,upright_bad_=0,error_deg_=0;
};
}

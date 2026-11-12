// Twin of simulate_cpp/src/controllers/locomotion/MultiGaitV257Runtime.hpp (sim_multigait_v257).
// Keep in sync; parity: tests/multigait_v2573_runtime_parity_test.cpp.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "LegFK.hpp"

// Scalar port of the V2.5.7 training Machine/Clock/BalanceState (command.py compute).
// Observation is sampled BEFORE this tick's timing action; AdvanceAction runs after inference.
// Recovery latch (V2.5.4) and constant lateral cap 0.6 (V2.5.5) are always on; stop thresholds per StopGate.
struct MultiGaitV257Runtime {
    enum Mode { STAND, WALK, RUN, S2W, W2S, W2R, R2W, TURN };
    enum Substate { BRAKE, GATHER, SETTLE };
    static constexpr float dt=.02f;
    using V3=std::array<float,3>;
    static float clamp(float x,float lo,float hi){return std::clamp(x,lo,hi);}
    static float ramp(float x,float lo,float hi){return clamp((x-lo)/(hi-lo),0,1);}
    static float norm2(float x,float y){return std::hypot(x,y);}
    static float norm3(const V3& v){return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
    static float dot(const V3& a,const V3& b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
    static V3 cross(const V3& a,const V3& b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
    static V3 horizontal(const V3& v,const V3& g){float d=dot(v,g);return {v[0]-d*g[0],v[1]-d*g[1],v[2]-d*g[2]};}

    // Stop-path thresholds: V2.5.7 original vs stopfix (balance.py/command.py/machine.py of each source).
    struct StopGate {
        float hold_speed=.10f,move_speed=.14f,slow_move_speed=.10f,urgency_lo=.10f,urgency_hi=.35f,latch_expire_s=0;
        bool recovery_landing=false;
        static StopGate V257(){return {};}
        static StopGate StopFix(){return {.30f,.30f,.22f,.35f,.70f,4.f,true};}
    };
    // v2573/v2574 deltas (machine.py, command.py, gather.py, timing.py, conditioning.py).
    struct Variant {
        bool v2573=false;       // run .95/.75, accel .5/.8/.6/1.0, cap 1.2, grounded duty, tight gather, turn init, gather step, W2S_GATHER quiet code
        bool anisotropic=false; // v2573_push/v2574: anisotropic tilt/predicted/motion urgency
    };
    // IMU/FK stop proxy (balance.py BalanceState, FSM outputs only).
    struct Balance {
        StopGate gate;
        std::array<V3,2> previous{};std::array<float,2> support_age{};V3 velocity{};long samples=0;
        float ready_dwell=0,hold_dwell=0,motion_dwell=0,slow_motion_dwell=0,support_dwell=0,pose_dwell=0,speed=0,uncertainty=0;
        bool confident=false,recovery_balanced=false;
        struct Out{bool stop_ready,stand_ready,motion_step,pose_invalid;};
        Out Update(const std::array<V3,2>& feet,const V3& g,const V3& gyro,const std::array<float,2>& support,const V3& raw,bool hold,
                   bool base_stable,float torso_tilt,float waist_rate,bool lift,const std::array<V3,2>& foot_velocity){
            bool valid=true;for(auto& f:feet)for(float x:f)valid&=std::isfinite(x);for(int i=0;i<3;++i)valid&=std::isfinite(g[i])&&std::isfinite(gyro[i]);
            if(valid)previous=feet;
            samples=valid?samples+1:0;
            // Body-frame foot motion -> pelvis velocity estimate per foot.
            std::array<V3,2> vel{};std::array<float,2> height{};
            for(int f=0;f<2;++f){auto c=cross(gyro,feet[f]);for(int i=0;i<3;++i)vel[f][i]=-foot_velocity[f][i]-c[i];height[f]=-dot(feet[f],g);}
            float lowest=std::min(height[0],height[1]);std::array<bool,2> low{height[0]<=lowest+.02f,height[1]<=lowest+.02f};
            std::array<bool,2> sel{support[0]>.5f&&low[0],support[1]>.5f&&low[1]};if(!sel[0]&&!sel[1])sel=low;
            float count=std::max(1.f,float(sel[0])+float(sel[1]));V3 est{};
            for(int f=0;f<2;++f)if(sel[f])for(int i=0;i<3;++i)est[i]+=vel[f][i];
            for(float& x:est)x/=count;
            est=horizontal(est,g);
            bool finite_est=std::isfinite(est[0])&&std::isfinite(est[1])&&std::isfinite(est[2])&&valid;
            // Fast rise, slow decay.
            float alpha=norm3(est)>norm3(velocity)+.06f?.65f:.35f;
            if(finite_est)for(int i=0;i<3;++i)velocity[i]+=alpha*(est[i]-velocity[i]);
            float u=0;for(int f=0;f<2;++f)if(sel[f]){V3 r{vel[f][0]-est[0],vel[f][1]-est[1],vel[f][2]-est[2]};u+=norm3(horizontal(r,g));}
            uncertainty=u/count;
            bool finite=finite_est&&std::isfinite(velocity[0])&&std::isfinite(velocity[1])&&std::isfinite(velocity[2]);
            bool persistent=false;
            for(int f=0;f<2;++f){support_age[f]=support[f]>.5f&&low[f]&&finite?support_age[f]+dt:0;persistent|=support_age[f]>=.06f-.5f*dt&&sel[f];}
            bool candidate=persistent&&finite&&uncertainty<.20f&&norm3(est)<3.f;
            support_dwell=candidate?support_dwell+dt:0;
            confident=samples>=3&&candidate&&support_dwell>=.02f-.5f*dt;
            speed=norm3(velocity);
            bool stop=norm2(raw[0],raw[1])<.1f&&std::abs(raw[2])<.1f;
            bool calm=base_stable&&torso_tilt<.08727f&&waist_rate<.6f&&confident&&speed<.10f&&!lift;
            ready_dwell=stop&&calm&&hold?ready_dwell+dt:0;
            float root_tilt=std::acos(clamp(-g[2],-1,1)),gyro_n=norm3(gyro);
            bool landing=root_tilt<.12f&&gyro_n<.6f&&torso_tilt<.14f&&waist_rate<1.f&&confident&&speed<gate.hold_speed&&!lift;
            hold_dwell=stop&&landing?hold_dwell+dt:0;
            motion_dwell=stop&&confident&&speed>gate.move_speed?motion_dwell+dt:0;
            slow_motion_dwell=stop&&confident&&speed>gate.slow_move_speed?slow_motion_dwell+dt:0;
            recovery_balanced=gate.recovery_landing?stop&&landing:stop&&confident&&speed<.10f&&root_tilt<.08727f&&gyro_n<.35f&&torso_tilt<.08727f&&waist_rate<.8f&&!lift;
            pose_dwell=stop&&(torso_tilt>.10472f||waist_rate>.8f)?pose_dwell+dt:0;
            return {hold_dwell>=.04f-.5f*dt,ready_dwell>=.15f-.5f*dt,motion_dwell>=.06f-.5f*dt||slow_motion_dwell>=.20f-.5f*dt,pose_dwell>=.15f-.5f*dt};
        }
    };

    int mode=STAND,substate=BRAKE,next_side=0;
    float elapsed=0,progress=0,settle=0,run_blend=0,turn_weight=0;
    float start_cycle=0,urgency=0,budget=0,calm=0,error_age=0,corrective_age=0,cooldown=0;
    float phase=0,cycles=0,period=1,duty=.56f;
    float gather_error_age=0,corrective_cycle=0;
    bool recovery_latched=false;float recovery_age=0,recovery_exit_dwell=0,recovery_onset_dwell=0;
    bool correction_lift=false,saw_lift=false,armed=true,hold=true;
    V3 scheduled{},raw{},imu{};
    Balance balance;
    StopGate gate;
    Variant variant;

    bool Recovery() const {return recovery_latched||(budget>0&&urgency>.1f);}
    float RecoveryContext() const {return recovery_latched?.5f+.5f*clamp(budget/1.5f,0,1):budget/1.5f;}
    void Reset(){auto g=gate;auto v=variant;*this=MultiGaitV257Runtime{};gate=g;variant=v;}
    int Desired(int request=-1) const {
        float v=norm2(raw[0],raw[1]),w=std::abs(raw[2]);
        bool turning=(v<.1f&&w>.2f)||(mode==TURN&&v<.15f&&w>=.1f);
        bool running=variant.v2573?(raw[0]>=.95f||((mode==RUN||mode==W2R)&&raw[0]>.75f)):(raw[0]>=1.5f||((mode==RUN||mode==W2R)&&raw[0]>1.2f));
        int d=turning?TURN:running?RUN:WALK;
        if(request==STAND||request==WALK||request==TURN||request==RUN)d=request;
        if(d==RUN&&(raw[0]<=0||std::abs(raw[1])>.15f||w>.3f))d=WALK;
        return v<.1f&&w<.1f?STAND:d;
    }
    // tilt_urgency/predicted_urgency >= 0 replace the isotropic tilt ramps (v2573_push anisotropic risk).
    void Recover(float tilt,const V3& gyro,float predicted_tilt,float motion_urgency,float tilt_urgency=-1,float predicted_urgency=-1){
        for(int i=0;i<3;++i)imu[i]+=.2f*(gyro[i]-imu[i]);
        float tilt_term=tilt_urgency>=0?tilt_urgency:ramp(tilt,.12f,.35f);
        float base=std::max({tilt_term,ramp(norm2(imu[0],imu[1]),.65f,1.5f),ramp(std::abs(imu[2]-scheduled[2]),.8f,1.8f)});
        bool eligible=norm2(raw[0],raw[1])<.1f&&std::abs(raw[2])<.1f&&norm3(scheduled)<.2f&&(mode==STAND||mode==W2S);
        float predicted=predicted_urgency>=0?predicted_urgency:ramp(predicted_tilt,.12f,.35f);
        urgency=eligible?std::max({base,predicted,clamp(motion_urgency,0,1)}):base;
        budget=std::max(0.f,budget-dt);calm=urgency<.1f?calm+dt:0;
        armed=armed||calm>=.5f;
        bool legacy_onset=base>.6f&&armed;
        recovery_onset_dwell=eligible&&urgency>.45f?recovery_onset_dwell+dt:0;
        bool early_onset=eligible&&armed&&(recovery_onset_dwell>=.04f-.5f*dt||urgency>.8f);
        bool onset=legacy_onset||early_onset;
        if(onset)budget=1.5f;
        armed=armed&&!onset;
        recovery_latched=recovery_latched||(eligible&&(onset||(budget>0&&urgency>.1f)));
        recovery_age=recovery_latched?recovery_age+dt:0;
    }
    void Schedule(int desired){
        auto target=raw;
        target[0]=clamp(target[0],-1,variant.v2573?2.f:2.5f);target[1]=clamp(target[1],-.6f,.6f);target[2]=clamp(target[2],-1.5f,1.5f);
        if(desired==TURN)target[0]=target[1]=0;
        if(desired==STAND)target={0,0,0};
        float limit=variant.v2573?1.2f:((mode==RUN||mode==W2R)?2.5f:1.6f);
        if((mode==RUN||mode==R2W)&&desired==TURN)target[2]=0;
        target[0]=std::min(target[0],limit);
        for(int i=0;i<3;++i){
            float accel=variant.v2573?(i==2?1.f:i==1?.6f:std::abs(target[i])>std::abs(scheduled[i])?.5f:.8f)
                                     :(i==2?2.f:std::abs(target[i])>std::abs(scheduled[i])?.8f:1.2f);
            scheduled[i]+=clamp(target[i]-scheduled[i],-accel*dt,accel*dt);
        }
    }
    struct Gates{bool stop_ready=true,motion_step=false,stand_ready=true,pose_invalid=false,recovery_balanced=false;};
    void Update(int desired,bool stable,bool stance_ok,bool lift,bool start_stable,bool exit_ok,int correction_side,const Gates& gt){
        int old=mode,m=old;bool old_hold=hold;elapsed+=dt;
        float speed=norm2(scheduled[0],scheduled[1]),yaw=std::abs(scheduled[2]);bool stop=desired==STAND;
        if(recovery_latched&&!stop){recovery_latched=false;recovery_age=0;recovery_exit_dwell=0;}
        error_age=!exit_ok?error_age+dt:0;
        bool needs_step=error_age>=.15f||Recovery()||(stop&&gt.motion_step);cooldown=std::max(0.f,cooldown-dt);
        if(old==STAND&&!stop)m=S2W;
        if(old==STAND&&stop&&(needs_step||gt.pose_invalid))m=W2S;
        saw_lift=saw_lift||(old==S2W&&lift);
        bool first_done=cycles-start_cycle>=.5f&&saw_lift&&!lift&&start_stable;
        if(old==S2W&&first_done)m=desired==TURN?TURN:WALK;
        if(old==S2W&&stop)m=W2S;
        if(old==WALK&&stop)m=W2S;
        if(old==WALK&&desired==TURN&&speed<.15f)m=TURN;
        if(old==WALK&&desired==RUN)m=W2R;
        if(old==TURN&&stop)m=W2S;
        if(old==TURN&&(desired==WALK||desired==RUN))m=WALK;
        if(old==RUN&&desired!=RUN)m=R2W;
        if(old==W2R&&desired!=RUN)m=R2W;
        if(old==W2R&&desired==RUN&&run_blend>=.999f)m=RUN;
        if(old==R2W&&desired==RUN)m=W2R;
        if(old==R2W&&desired!=RUN&&run_blend<=.001f&&speed<=1.3f)m=stop?W2S:WALK;
        if(old==W2S&&!stop)m=S2W;
        if(m==W2S&&old!=W2S)substate=BRAKE;
        bool braking=m==W2S&&substate==BRAKE;
        bool entering_gather=braking&&speed<.15f&&yaw<.15f;
        if(entering_gather)substate=GATHER;
        bool gathering=m==W2S&&substate==GATHER;
        gather_error_age=gathering&&!stance_ok?gather_error_age+dt:0;
        bool gather_need=needs_step||gather_error_age>=.30f-.5f*dt;
        bool recovery_calm=gt.recovery_balanced&&stance_ok&&!lift&&urgency<.1f;
        if(recovery_latched&&recovery_calm)gather_need=false;
        bool correction=gathering&&old_hold&&gather_need&&cooldown<=0&&(!lift||Recovery());
        correction_lift=correction_lift||(gathering&&!old_hold&&lift);
        bool completed=cycles-corrective_cycle>=.5f&&correction_lift&&!lift;
        bool timed_out=gathering&&!old_hold&&corrective_age>=std::min(period+.15f,1.2f)&&!Recovery();
        bool can_hold=gathering&&!lift&&(!gather_need||completed||timed_out);
        if(variant.v2573)can_hold=can_hold&&(recovery_latched||stance_ok||completed||timed_out);
        bool recovery_landed=!correction_lift||completed;
        if(recovery_latched)can_hold=can_hold&&recovery_calm&&recovery_landed;
        can_hold=can_hold&&gt.stop_ready;
        hold=m==W2S?old_hold:false;
        if(braking)hold=false;
        if(gathering&&can_hold)hold=true;
        if(correction)hold=false;
        corrective_age=gathering&&!hold?corrective_age+dt:0;
        if(gathering&&!old_hold&&hold)cooldown=.3f;
        recovery_exit_dwell=recovery_latched&&recovery_calm&&hold&&!lift?recovery_exit_dwell+dt:0;
        bool expired=gate.latch_expire_s>0&&recovery_age>=gate.latch_expire_s&&urgency<.45f;
        if(recovery_latched&&(recovery_exit_dwell>=.4f-.5f*dt||expired)){recovery_latched=false;budget=0;}
        bool settling=m==W2S&&substate==SETTLE;
        bool valid_shape=stance_ok&&stable&&!Recovery()&&gt.stand_ready;
        if(gathering&&hold&&valid_shape)substate=SETTLE;
        if(settling&&!valid_shape)substate=GATHER;
        if(settling)hold=true;
        bool valid=m==W2S&&substate==SETTLE&&valid_shape&&stop;
        settle=valid?settle+dt:0;
        if(valid&&settle>=.75f-.5f*dt)m=STAND;
        hold=hold||m==STAND;
        bool changed=m!=old;
        if(changed)elapsed=0;
        if(m==S2W&&changed){
            start_cycle=cycles;saw_lift=false;
            // v2573 turn_init_phase: the swing leg follows the turn direction.
            if(variant.v2573&&desired==TURN){phase=scheduled[2]>0?.5f:0.f;next_side=scheduled[2]>0?0:1;}
        }
        // v2573: a GATHER entry with a bad footprint triggers a corrective step immediately.
        bool trigger_step=(old_hold&&!hold)||(variant.v2573&&entering_gather&&!stance_ok&&m==W2S);
        if(trigger_step){
            phase=(m==W2S&&!Recovery()&&correction_side>=0)?.5f*(1-correction_side):next_side*.5f;
            next_side=1-int(phase*2);
            if(m==W2S){corrective_cycle=cycles;correction_lift=false;}
        }
        mode=m;
        run_blend+=clamp(float(m==RUN||m==W2R)-run_blend,-dt/.6f,dt/.6f);
        turn_weight+=clamp(float(m==TURN||(m==S2W&&desired==TURN))-turn_weight,-dt/.4f,dt/.4f);
        progress=clamp(elapsed/.6f,0,1);
        if(m==W2R)progress=run_blend;
        if(m==R2W)progress=1-run_blend;
        if(m==S2W)progress=clamp((cycles-start_cycle)/.5f,0,1);
        if(m==W2S&&substate==SETTLE)progress=clamp(settle/.75f,0,1);
        if(recovery_latched&&hold)progress=clamp(recovery_exit_dwell/.4f,0,1);
    }
    std::array<float,2> Support() const {
        bool frozen=hold||mode==STAND||(mode==W2S&&substate==SETTLE);
        return {float(frozen||phase<duty),float(frozen||std::fmod(phase+.5f,1.f)<duty)};
    }
    std::array<float,23> Conditioning() const {
        std::array<float,23> o{};o[mode]=1;if(mode==W2S)o[8+substate]=1;
        o[11]=period;o[12]=duty;auto s=Support();o[13]=s[0];o[14]=s[1];o[15]=progress;
        o[16]=urgency;o[17]=RecoveryContext();
        for(int i=0;i<3;++i)o[18+i]=raw[i];
        o[21]=turn_weight;o[22]=1;
        bool quiet=(mode==STAND||(mode==W2S&&hold&&norm3(scheduled)<.2f))&&!Recovery();
        if(quiet){
            for(int i=0;i<11;++i)o[i]=0;
            // v2573 actor_quiet_revision: STAND and W2S hold share the W2S_GATHER code.
            if(variant.v2573){o[W2S]=1;o[8+GATHER]=1;}else o[0]=1;
            o[15]=1;
        }
        return o;
    }
    void AdvanceAction(float raw_timing){
        float z=(std::tanh(raw_timing)+1)*.5f;
        float walking=std::exp(std::log(.65f)+z*std::log(1.f/.65f));
        float turning=std::exp(std::log(.55f)+z*std::log(1.f/.55f));
        float walk=std::exp((1-turn_weight)*std::log(walking)+turn_weight*std::log(turning));
        float run=std::exp(std::log(.5f)+z*std::log(.7f/.5f));
        float target=std::exp((1-run_blend)*std::log(walk)+run_blend*std::log(run));
        if(mode==W2S)target=.8f;
        if(mode==STAND)target=1;
        period=std::exp(std::log(period)+clamp(std::log(target)-std::log(period),-1.5f*dt,1.5f*dt));
        float swing=clamp(.44f*period,.24f,.44f);
        float walk_duty=clamp(1-swing/period,.54f,.88f);
        duty=variant.v2573?walk_duty:(1-run_blend)*walk_duty+run_blend*.42f;
        bool frozen=hold||mode==STAND||(mode==W2S&&substate==SETTLE);
        float inc=frozen?0:dt/period;cycles+=inc;phase=std::fmod(phase+inc,1.f);
    }

    // One control tick of command.py compute(), from raw sensors (policy joint order).
    struct Inputs{V3 raw{},gyro{},gravity{};std::array<float,24> q{},dq{};int request=-1;};
    void Step(const Inputs& in,const LegFK& fk){
        raw=in.raw;const V3& g=in.gravity;
        float tilt=std::acos(clamp(-g[2],-1,1));
        V3 filtered{},predicted=g;
        for(int i=0;i<3;++i)filtered[i]=imu[i]+.2f*(in.gyro[i]-imu[i]);
        auto c=cross(filtered,g);for(int i=0;i<3;++i)predicted[i]-=.12f*c[i];
        float pn=std::max(1.e-6f,norm3(predicted));float predicted_tilt=std::acos(clamp(-predicted[2]/pn,-1,1));
        // Motion urgency uses the previous tick's balance estimate (Python order).
        balance.gate=gate;
        if(variant.anisotropic){
            // command.py v2573_push: elliptic risk, backward/forward/lateral axes differ.
            auto risk=[](float x,float y,float back,float fwd,float side){float a=std::max(-x,0.f)/back,b=std::max(x,0.f)/fwd,c=y/side;return a*a+b*b+c*c;};
            float tilt_u=ramp(risk(g[0],g[1],.06f,.12f,.09f),1,4);
            float pred_u=ramp(risk(predicted[0]/pn,predicted[1]/pn,.06f,.12f,.09f),1,4);
            float motion_u=balance.confident?ramp(risk(balance.velocity[0],balance.velocity[1],.15f,.35f,.20f),1,4):0.f;
            Recover(tilt,in.gyro,predicted_tilt,motion_u,tilt_u,pred_u);
        }else{
            Recover(tilt,in.gyro,predicted_tilt,balance.confident?ramp(balance.speed,gate.urgency_lo,gate.urgency_hi):0.f);
        }
        std::array<float,12> leg_q{},plus{},minus{};float v2=0;
        for(int i=0;i<12;++i){leg_q[i]=in.q[i];plus[i]=in.q[i]+.001f*in.dq[i];minus[i]=in.q[i]-.001f*in.dq[i];v2+=in.dq[i]*in.dq[i];}
        auto metrics=fk.Compute(leg_q,g);
        bool stable=tilt<.08727f&&norm3(imu)<.35f&&std::sqrt(v2/12)<.6f;
        bool stance,exit_ok;
        if(variant.v2573){
            stance=std::abs(metrics.width_err)<.008f&&std::abs(metrics.dx_err)<.025f&&std::abs(metrics.dyaw_err)<.08f;
            exit_ok=std::abs(metrics.width_err)<.030f&&std::abs(metrics.dx_err)<.045f&&std::abs(metrics.dyaw_err)<.14f;
        }else{
            stance=std::abs(metrics.width_err)<.025f&&std::abs(metrics.dx_err)<.035f&&std::abs(metrics.dyaw_err)<.1f;
            exit_ok=std::abs(metrics.width_err)<.04f&&std::abs(metrics.dx_err)<.05f&&std::abs(metrics.dyaw_err)<.16f;
        }
        bool lift=metrics.height_difference>.025f;
        int desired=Desired(in.request);Schedule(desired);
        const auto& left=metrics.feet_pos[0];const auto& right=metrics.feet_pos[1];
        float cx0=right[0]+.5f*fk.home_dx(),cy0=right[1]+.5f*fk.home_width(),cx1=left[0]-.5f*fk.home_dx(),cy1=left[1]-.5f*fk.home_width();
        float score0=cx0*cx0+cy0*cy0,score1=cx1*cx1+cy1*cy1;
        int side=std::abs(score0-score1)<1.e-5f?1-next_side:int(score1<score0);
        // Torso gravity = Rz(yaw)·Rx(roll)·g; yaw keeps z, so tilt depends on roll only.
        float torso_tilt=std::acos(clamp(-(-std::sin(in.q[12])*g[1]+std::cos(in.q[12])*g[2]),-1,1));
        float waist_rate=std::sqrt(.5f*(in.dq[12]*in.dq[12]+in.dq[13]*in.dq[13]));
        auto feet=fk.BodyFeet(leg_q),fp=fk.BodyFeet(plus),fm=fk.BodyFeet(minus);
        std::array<V3,2> fv{};for(int f=0;f<2;++f)for(int i=0;i<3;++i)fv[f][i]=(fp[f][i]-fm[f][i])/.002f;
        auto out=balance.Update(feet,g,in.gyro,Support(),raw,hold,stable,torso_tilt,waist_rate,lift,fv);
        stable=stable&&torso_tilt<.08727f&&waist_rate<.6f;
        Update(desired,stable,stance,lift,tilt<.18f&&norm2(imu[0],imu[1])<.8f,exit_ok,side,{out.stop_ready,out.motion_step,out.stand_ready,out.pose_invalid,balance.recovery_balanced});
    }
};

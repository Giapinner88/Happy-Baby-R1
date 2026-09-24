// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// !!! hack code: make glfw_adapter.window_ public
#define private public
#include "glfw_adapter.h"
#undef private

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include "simulate.h"
#include "array_safety.h"
#include "unitree_sdk2_bridge.h"
#include "param.h"
#include "sim_startup_gate.h"
#include "scheduled_push.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"
#define NUM_MOTOR_IDL_GO 20

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#endif
}

class ElasticBand
{
public:
  ElasticBand(){};
  void Advance(std::vector<double> x, std::vector<double> dx)
  {
    std::vector<double> delta_x = {0.0, 0.0, 0.0};
    delta_x[0] = point_[0] - x[0];
    delta_x[1] = point_[1] - x[1];
    delta_x[2] = point_[2] - x[2];
    double distance = sqrt(delta_x[0] * delta_x[0] + delta_x[1] * delta_x[1] + delta_x[2] * delta_x[2]);

    std::vector<double> direction = {0.0, 0.0, 0.0};
    direction[0] = delta_x[0] / distance;
    direction[1] = delta_x[1] / distance;
    direction[2] = delta_x[2] / distance;

    double v = dx[0] * direction[0] + dx[1] * direction[1] + dx[2] * direction[2];

    f_[0] = (stiffness_ * (distance - length_) - damping_ * v) * direction[0];
    f_[1] = (stiffness_ * (distance - length_) - damping_ * v) * direction[1];
    f_[2] = (stiffness_ * (distance - length_) - damping_ * v) * direction[2];
  }


  double stiffness_ = 200;
  double damping_ = 100;
  std::vector<double> point_ = {0, 0, 3};
  double length_ = 0.0;
  bool enable_ = true;
  std::vector<double> f_ = {0, 0, 0};
};
inline ElasticBand elastic_band;


namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;
  ScheduledPush scheduled_push;

  // Optional gait-quality telemetry.  The policy process only sees DDS state;
  // this recorder lives beside MuJoCo so foot contacts, peaks and impact force
  // are measured from the actual physics state.  It is disabled unless the
  // caller sets MUJOCO_GAIT_TELEMETRY to a CSV path.
  class GaitTelemetry
  {
  public:
    GaitTelemetry()
    {
      const char *path = std::getenv("MUJOCO_GAIT_TELEMETRY");
      if (path && *path) path_ = path;
    }

    bool Enabled() const { return !path_.empty(); }

    void Bind(mjModel *model, mjData *data)
    {
      if (!Enabled() || model == bound_model_) return;
      if (out_.is_open()) out_.close();
      bound_model_ = model;
      out_.open(path_, std::ios::out | std::ios::trunc);
      if (!out_) {
        std::cerr << "[GAIT TELEMETRY] Cannot open " << path_ << "\n";
        path_.clear();
        return;
      }
      for (int side = 0; side < 2; ++side) {
        site_id_[side] = mj_name2id(model, mjOBJ_SITE, side == 0 ? "left_foot" : "right_foot");
        joint_id_[side] = mj_name2id(model, mjOBJ_JOINT, side == 0 ? "left_knee_joint" : "right_knee_joint");
      }
      for (int geom = 0; geom < model->ngeom; ++geom) {
        const char *name = mj_id2name(model, mjOBJ_GEOM, geom);
        if (!name) continue;
        const std::string geom_name(name);
        for (int side = 0; side < 2; ++side) {
          const std::string prefix = side == 0 ? "left_foot" : "right_foot";
          if (geom_name.find(prefix) != std::string::npos &&
              geom_name.find("collision") != std::string::npos) {
            foot_geom_ids_[side].push_back(geom);
          }
        }
      }
      for (int side = 0; side < 2; ++side) {
        if (site_id_[side] < 0 || joint_id_[side] < 0) {
          std::cerr << "[GAIT TELEMETRY] Missing " << (side == 0 ? "left" : "right")
                    << " foot site or knee joint; columns will be NaN.\n";
        }
      }
      out_ << "time,base_x,base_y,base_z,base_vx,base_vy,"
              "left_foot_x,left_foot_y,left_foot_z,left_foot_vz,left_contact,left_force_n,"
              "left_knee_q,left_knee_dq,left_swing_peak_z,left_touchdown_vz,left_step_length,"
              "right_foot_x,right_foot_y,right_foot_z,right_foot_vz,right_contact,right_force_n,"
              "right_knee_q,right_knee_dq,right_swing_peak_z,right_touchdown_vz,right_step_length\n";
      out_.flush();
      (void)data;
      Reset();
      std::cout << "[GAIT TELEMETRY] Writing MuJoCo gait metrics to " << path_
                << " (left/right foot geoms=" << foot_geom_ids_[0].size() << "/"
                << foot_geom_ids_[1].size() << ")\n";
    }

    void Record(const mjModel *model, const mjData *data)
    {
      if (!out_.is_open() || model != bound_model_) return;
      bool contact[2] = {false, false};
      double force_n[2] = {0.0, 0.0};
      for (int i = 0; i < data->ncon; ++i) {
        const mjContact &contact_info = data->contact[i];
        for (int side = 0; side < 2; ++side) {
          if (!IsFootGeom(side, contact_info.geom[0]) &&
              !IsFootGeom(side, contact_info.geom[1])) continue;
          double wrench[6] = {};
          mj_contactForce(model, data, i, wrench);
          contact[side] = true;
          force_n[side] += std::max(0.0, wrench[0]);
        }
      }

      double touchdown_peak[2] = {NAN, NAN};
      double touchdown_vz[2] = {NAN, NAN};
      double step_length[2] = {NAN, NAN};
      for (int side = 0; side < 2; ++side) {
        const int site = site_id_[side];
        const double z = site >= 0 ? data->site_xpos[3 * site + 2] : NAN;
        const double x = site >= 0 ? data->site_xpos[3 * site + 0] : NAN;
        double site_velocity[6] = {};
        if (site >= 0) mj_objectVelocity(model, data, mjOBJ_SITE, site, site_velocity, 0);
        const double vz = site >= 0 ? site_velocity[5] : NAN;
        if (!contact[side] && std::isfinite(z)) {
          swing_peak_z_[side] = std::max(swing_peak_z_[side], z);
        }
        if (!prev_contact_[side] && contact[side]) {
          touchdown_peak[side] = swing_peak_z_[side];
          touchdown_vz[side] = vz;
          if (has_touchdown_[side] && std::isfinite(x)) {
            step_length[side] = x - last_touchdown_x_[side];
          }
          if (std::isfinite(x)) {
            last_touchdown_x_[side] = x;
            has_touchdown_[side] = true;
          }
          swing_peak_z_[side] = z;
        }
        if (contact[side] && std::isfinite(z) && !std::isfinite(swing_peak_z_[side])) {
          swing_peak_z_[side] = z;
        }
        prev_contact_[side] = contact[side];
      }

      out_ << data->time << ',' << data->qpos[0] << ',' << data->qpos[1] << ',' << data->qpos[2]
           << ',' << data->qvel[0] << ',' << data->qvel[1];
      for (int side = 0; side < 2; ++side) {
        const int site = site_id_[side];
        const int joint = joint_id_[side];
        const bool valid_site = site >= 0;
        const bool valid_joint = joint >= 0;
        double site_velocity[6] = {};
        if (valid_site) mj_objectVelocity(model, data, mjOBJ_SITE, site, site_velocity, 0);
        out_ << ',' << (valid_site ? data->site_xpos[3 * site + 0] : NAN)
             << ',' << (valid_site ? data->site_xpos[3 * site + 1] : NAN)
             << ',' << (valid_site ? data->site_xpos[3 * site + 2] : NAN)
             << ',' << (valid_site ? site_velocity[5] : NAN)
             << ',' << (contact[side] ? 1 : 0) << ',' << force_n[side]
             << ',' << (valid_joint ? data->qpos[model->jnt_qposadr[joint]] : NAN)
             << ',' << (valid_joint ? data->qvel[model->jnt_dofadr[joint]] : NAN)
             << ',' << touchdown_peak[side] << ',' << touchdown_vz[side]
             << ',' << step_length[side];
      }
      out_ << '\n';
      if (++rows_ % 100 == 0) out_.flush();
    }

    void Close()
    {
      if (out_.is_open()) {
        out_.flush();
        out_.close();
      }
      bound_model_ = nullptr;
    }

  private:
    bool IsFootGeom(int side, int geom) const
    {
      return std::find(foot_geom_ids_[side].begin(), foot_geom_ids_[side].end(), geom) !=
             foot_geom_ids_[side].end();
    }

    void Reset()
    {
      prev_contact_[0] = prev_contact_[1] = false;
      has_touchdown_[0] = has_touchdown_[1] = false;
      swing_peak_z_[0] = swing_peak_z_[1] = -INFINITY;
      last_touchdown_x_[0] = last_touchdown_x_[1] = NAN;
      rows_ = 0;
    }

    std::string path_;
    std::ofstream out_;
    mjModel *bound_model_ = nullptr;
    std::array<int, 2> site_id_ = {-1, -1};
    std::array<int, 2> joint_id_ = {-1, -1};
    std::array<std::vector<int>, 2> foot_geom_ids_;
    bool prev_contact_[2] = {false, false};
    bool has_touchdown_[2] = {false, false};
    double swing_peak_z_[2] = {-INFINITY, -INFINITY};
    double last_touchdown_x_[2] = {NAN, NAN};
    std::size_t rows_ = 0;
  } gait_telemetry;

  // control noise variables
  mjtNum *ctrlnoise = nullptr;

  volatile std::sig_atomic_t camera_record_stop = 0;
  std::atomic<int> process_exit_status{0};

  void CameraRecordSignalHandler(int)
  {
    camera_record_stop = 1;
  }

  class FfmpegPipe
  {
  public:
    bool Start(const std::filesystem::path &output, int width, int height, int fps)
    {
#if defined(_WIN32) || defined(__CYGWIN__)
      (void)output; (void)width; (void)height; (void)fps;
      std::cerr << "Direct camera recording is only implemented on POSIX systems.\n";
      return false;
#else
      int pipefd[2];
      if (pipe(pipefd) != 0)
      {
        std::perror("pipe");
        return false;
      }
      const std::string size = std::to_string(width) + "x" + std::to_string(height);
      const std::string rate = std::to_string(fps);
      pid_ = fork();
      if (pid_ < 0)
      {
        std::perror("fork");
        close(pipefd[0]);
        close(pipefd[1]);
        return false;
      }
      if (pid_ == 0)
      {
        dup2(pipefd[0], STDIN_FILENO);
        close(pipefd[0]);
        close(pipefd[1]);
        execlp("ffmpeg", "ffmpeg", "-hide_banner", "-loglevel", "warning", "-y",
               "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", size.c_str(),
               "-framerate", rate.c_str(), "-i", "pipe:0", "-vf", "vflip",
               "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "20",
               "-pix_fmt", "yuv420p", output.c_str(), static_cast<char *>(nullptr));
        std::perror("execlp ffmpeg");
        _exit(127);
      }
      close(pipefd[0]);
      fd_ = pipefd[1];
      return true;
#endif
    }

    bool Write(const unsigned char *data, std::size_t size)
    {
#if defined(_WIN32) || defined(__CYGWIN__)
      (void)data; (void)size;
      return false;
#else
      std::size_t offset = 0;
      while (offset < size)
      {
        const ssize_t n = write(fd_, data + offset, size - offset);
        if (n > 0)
        {
          offset += static_cast<std::size_t>(n);
        }
        else if (n < 0 && errno == EINTR)
        {
          continue;
        }
        else
        {
          return false;
        }
      }
      return true;
#endif
    }

    bool Finish()
    {
#if defined(_WIN32) || defined(__CYGWIN__)
      return false;
#else
      if (fd_ >= 0)
      {
        close(fd_);
        fd_ = -1;
      }
      if (pid_ <= 0) return true;
      int status = 0;
      while (waitpid(pid_, &status, 0) < 0)
      {
        if (errno != EINTR) return false;
      }
      pid_ = -1;
      return WIFEXITED(status) && WEXITSTATUS(status) == 0;
#endif
    }

    ~FfmpegPipe()
    {
      if (fd_ >= 0) close(fd_);
    }

  private:
    int fd_ = -1;
    pid_t pid_ = -1;
  };

  using Seconds = std::chrono::duration<double>;

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {
#if defined(_WIN32) || defined(__CYGWIN__)
    constexpr char kPathSep = '\\';
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      DWORD buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
        if (written < buf_size)
        {
          success = true;
        }
        else if (written == buf_size)
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
        else
        {
          std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
          return "";
        }
      }
      return realpath.get();
    }();
#else
    constexpr char kPathSep = '/';
#if defined(__APPLE__)
    std::unique_ptr<char[]> buf(nullptr);
    {
      std::uint32_t buf_size = 0;
      _NSGetExecutablePath(nullptr, &buf_size);
      buf.reset(new char[buf_size]);
      if (!buf)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }
      if (_NSGetExecutablePath(buf.get(), &buf_size))
      {
        std::cerr << "unexpected error from _NSGetExecutablePath\n";
      }
    }
    const char *path = buf.get();
#else
    const char *path = "/proc/self/exe";
#endif
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();
#endif

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }

    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }

    // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
#else
    const std::string sep = "/";
#endif

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // ChannelFactory::Instance()->Init(0);
    // UnitreeDds ud(d);

    // run until asked to exit
    while (!sim.exitrequest.load())
    {
      if (sim.droploadrequest.load())
      {
        sim.LoadMessage(sim.dropfilename);
        mjModel *mnew = LoadModel(sim.dropfilename, sim);
        sim.droploadrequest.store(false);

        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
          if (dnew && mnew->nkey > 0) mj_resetDataKeyframe(mnew, dnew, 0);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      if (sim.uiloadrequest.load())
      {
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
          if (dnew && mnew->nkey > 0) mj_resetDataKeyframe(mnew, dnew, 0);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          gait_telemetry.Bind(m, d);
          // running
          const bool r1_control_ready =
              param::config.robot != "r1" ||
              sim_startup::bridge_control_ready.load(std::memory_order_acquire);
          if (sim.run && r1_control_ready)
          {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std)
            {
              // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
              mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
              mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

              for (int i = 0; i < m->nu; i++)
              {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed)
            {
              // re-sync
              syncCPU = startCPU;
              syncSim = d->time;
              sim.speed_changed = false;

              // run single step, let next iteration deal with timing
              scheduled_push.Step(m, d);
              gait_telemetry.Record(m, d);
              stepped = true;
            }

            // in-sync: step until ahead of cpu
            else
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // elastic band on base link
                if (param::config.enable_elastic_band == 1)
                {
                  if (elastic_band.enable_)
                  {
                    std::vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]};
                    std::vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};

                    elastic_band.Advance(x, dx);

                    d->xfrc_applied[param::config.band_attached_link] = elastic_band.f_[0];
                    d->xfrc_applied[param::config.band_attached_link + 1] = elastic_band.f_[1];
                    d->xfrc_applied[param::config.band_attached_link + 2] = elastic_band.f_[2];
                  }
                }

                // call mj_step
                scheduled_push.Step(m, d);
                gait_telemetry.Record(m, d);
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped && param::config.record_path.empty())
            {
              sim.AddToHistory();
            }
          }

          // paused
          else
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
        }
      } // release std::lock_guard<std::mutex>
    }
  }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  sim_startup::model_state_ready.store(false, std::memory_order_release);
  sim_startup::bridge_control_ready.store(false, std::memory_order_release);
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    if (param::config.record_path.empty()) sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      if (param::config.record_path.empty())
      {
        sim->Load(m, d, filename);
      }
      else
      {
        const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
        sim->m_ = m;
        sim->d_ = d;
      }
      // Spawn ở keyframe "stand" nếu model có: robot đứng sẵn thay vì bị thả rơi.
      if (m->nkey > 0) mj_resetDataKeyframe(m, d, 0);
      mj_forward(m, d);
      sim_startup::model_state_ready.store(true, std::memory_order_release);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  gait_telemetry.Close();
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);

  std::exit(process_exit_status.load(std::memory_order_acquire));
}

void *UnitreeSdk2BridgeThread(void *arg)
{
  // Wait for mujoco data
  while (true)
  {
    if (d && sim_startup::model_state_ready.load(std::memory_order_acquire))
    {
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(1000);
  }

  unitree::robot::ChannelFactory::Instance()->Init(param::config.domain_id, param::config.interface);


  int body_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
  if (body_id < 0) {
    body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
  }
  param::config.band_attached_link = 6 * body_id;
  
  std::unique_ptr<UnitreeSDK2BridgeBase> interface = nullptr;
  if (m->nu > NUM_MOTOR_IDL_GO) {
    interface = std::make_unique<G1Bridge>(m, d);
  } else {
    interface = std::make_unique<Go2Bridge>(m, d);
  }
  interface->start();
  
  while (true)
  {
    sleep(1);
  }
}

int CameraRecordLoop(mj::Simulate *sim)
{
  const int width = param::config.record_width;
  const int height = param::config.record_height;
  const int fps = param::config.record_fps;
  if (width <= 0 || height <= 0 || fps <= 0)
  {
    std::cerr << "[RECORDER] width, height and fps must be positive.\n";
    return 2;
  }

  std::signal(SIGINT, CameraRecordSignalHandler);
  std::signal(SIGTERM, CameraRecordSignalHandler);
  std::signal(SIGPIPE, SIG_IGN);

  std::cout << "[RECORDER] Waiting for the MuJoCo model...\n";
  while (!camera_record_stop &&
         !sim_startup::model_state_ready.load(std::memory_order_acquire))
  {
    sim->platform_ui->PollEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (camera_record_stop || !m || !d) return 3;

  mjvScene scene;
  mjv_defaultScene(&scene);
  mjv_makeScene(m, &scene, mj::Simulate::kMaxGeom);
  // Recorder ưu tiên tốc độ và hình ảnh ổn định trên Xvfb/software GL.
  scene.flags[mjRND_SHADOW] = 0;
  scene.flags[mjRND_REFLECTION] = 0;

  mjrContext context;
  mjr_defaultContext(&context);
  m->vis.global.offwidth = width;
  m->vis.global.offheight = height;
  mjr_makeContext(m, &context, mjFONTSCALE_100);
  mjr_setBuffer(mjFB_OFFSCREEN, &context);
  if (context.currentBuffer != mjFB_OFFSCREEN)
  {
    std::cerr << "[RECORDER] MuJoCo offscreen framebuffer is unavailable.\n";
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    return 4;
  }

  mjvCamera record_cam;
  mjv_defaultCamera(&record_cam);
  int track_body = mj_name2id(m, mjOBJ_BODY, "torso_link");
  if (track_body < 0) track_body = mj_name2id(m, mjOBJ_BODY, "base_link");
  if (track_body < 0)
  {
    std::cerr << "[RECORDER] Cannot find torso_link or base_link for the tracking camera.\n";
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    return 5;
  }
  record_cam.type = mjCAMERA_TRACKING;
  record_cam.trackbodyid = track_body;
  record_cam.distance = param::config.record_camera_distance;
  record_cam.azimuth = param::config.record_camera_azimuth;
  record_cam.elevation = param::config.record_camera_elevation;
  record_cam.lookat[0] = d->xpos[3 * track_body + 0];
  record_cam.lookat[1] = d->xpos[3 * track_body + 1];
  record_cam.lookat[2] = d->xpos[3 * track_body + 2];

  FfmpegPipe ffmpeg;
  if (!ffmpeg.Start(param::config.record_path, width, height, fps))
  {
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    return 6;
  }

  std::vector<unsigned char> rgb(static_cast<std::size_t>(3) * width * height);
  const mjrRect viewport{0, 0, width, height};
  std::size_t frames = 0;
  std::size_t rendered_frames = 0;
  double first_sim_time = -1.0;
  bool write_ok = true;

  std::cout << "[RECORDER] Direct MuJoCo camera -> " << param::config.record_path
            << " (" << width << "x" << height << " @ " << fps << " fps, body="
            << mj_id2name(m, mjOBJ_BODY, track_body) << ")\n";

  while (!camera_record_stop && !sim->platform_ui->ShouldCloseWindow())
  {
    sim->platform_ui->PollEvents();
    double sim_time = 0.0;
    {
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
      if (!m || !d) break;
      sim_time = d->time;
      if (sim_time <= 0.0)
      {
        continue;
      }
      if (first_sim_time < 0.0) first_sim_time = sim_time;
      mjv_updateScene(m, d, &sim->opt, &sim->pert, &record_cam, mjCAT_ALL, &scene);
    }
    const std::size_t target_frames = static_cast<std::size_t>(
        std::floor((sim_time - first_sim_time) * fps)) + 1;
    if (target_frames <= frames)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    mjr_render(viewport, &scene, &context);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
                "R1 FLAT LOCOMOTION", "Direct MuJoCo camera | policy_goc.onnx", &context);
    mjr_readPixels(rgb.data(), nullptr, viewport, &context);
    ++rendered_frames;
    while (frames < target_frames)
    {
      if (!ffmpeg.Write(rgb.data(), rgb.size()))
      {
        std::cerr << "[RECORDER] Failed to write a frame to FFmpeg.\n";
        write_ok = false;
        break;
      }
      ++frames;
    }
    if (!write_ok) break;
  }

  const bool encode_ok = ffmpeg.Finish();
  mjr_freeContext(&context);
  mjv_freeScene(&scene);
  std::cout << "[RECORDER] Finalized " << frames << " encoded frames from "
            << rendered_frames << " direct camera renders: "
            << param::config.record_path << "\n";
  return write_ok && encode_ok ? 0 : 7;
}
//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char *msg)
{
  rosetta_error_msg = msg;
}
#endif

// user keyboard callback
void user_key_cb(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act==GLFW_PRESS)
  {
    if(param::config.enable_elastic_band == 1) {
      if (key==GLFW_KEY_9) {
        elastic_band.enable_ = !elastic_band.enable_;
      } else if (key==GLFW_KEY_7 || key==GLFW_KEY_UP) {
        elastic_band.length_ -= 0.1;
      } else if (key==GLFW_KEY_8 || key==GLFW_KEY_DOWN) {
        elastic_band.length_ += 0.1;
      }
    }
    if(key==GLFW_KEY_BACKSPACE) {
      if (m->nkey > 0) mj_resetDataKeyframe(m, d, 0);
      else             mj_resetData(m, d);
      mj_forward(m, d);
    }
  }
}

// run event loop
int main(int argc, char **argv)
{

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg)
  {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // Load simulation configuration
  std::filesystem::path proj_dir = std::filesystem::path(getExecutableDir()).parent_path();
  param::config.load_from_yaml(proj_dir / "config.yaml");
  param::helper(argc, argv);
  if (!param::config.record_path.empty() && param::config.record_path.is_relative()) {
    param::config.record_path = std::filesystem::absolute(param::config.record_path);
  }
  if(param::config.robot_scene.is_relative()) {
    param::config.robot_scene = proj_dir.parent_path() / "unitree_robots" / param::config.robot / param::config.robot_scene;
  }

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ false);

  std::thread unitree_thread(UnitreeSdk2BridgeThread, nullptr);

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), param::config.robot_scene.c_str());
  // Normal GUI, or direct offscreen MuJoCo-camera recorder.
  glfwSetKeyCallback(static_cast<mj::GlfwAdapter*>(sim->platform_ui.get())->window_,user_key_cb);
  int render_status = 0;
  if (param::config.record_path.empty()) sim->RenderLoop();
  else render_status = CameraRecordLoop(sim.get());
  process_exit_status.store(render_status, std::memory_order_release);
  sim->exitrequest.store(1);
  physicsthreadhandle.join();

  pthread_exit(NULL);
  return render_status;
}

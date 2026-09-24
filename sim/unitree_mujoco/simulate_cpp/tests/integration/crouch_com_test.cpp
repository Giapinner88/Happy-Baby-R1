// Headless MuJoCo acceptance test for the no-chair crouch trajectory.
// It uses the same R1 scene and motor torque limits as unitree_mujoco/simulate.
#include <mujoco/mujoco.h>
#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int kJoints = 24;
constexpr int L_HIP_P = 0, L_HIP_R = 1, L_KNEE = 3, L_ANK_P = 4, L_ANK_R = 5;
constexpr int R_HIP_P = 6, R_HIP_R = 7, R_KNEE = 9, R_ANK_P = 10, R_ANK_R = 11;
constexpr int L_SHO_P = 14, L_ELBOW = 17, R_SHO_P = 19, R_ELBOW = 22;
constexpr double kPi = 3.14159265358979323846;

void MuJoCoError(const char* message) {
  std::fprintf(stderr, "MuJoCo fatal error: %s\n", message);
  std::fflush(stderr);
  std::exit(70);
}

void MuJoCoWarning(const char* message) {
  std::fprintf(stderr, "MuJoCo warning: %s\n", message);
}

const std::array<double, kJoints> kStand = {
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
    0.0, 0.0,
    0.35, 0.18, 0.0, 0.87, 0.0,
    0.35, -0.18, 0.0, 0.87, 0.0,
};

const std::array<double, kJoints> kActionScale = {
    .22, .22, .22, .3475, .3125, .3125,
    .22, .22, .22, .3475, .3125, .3125,
    .125, .22,
    .15625, .15625, .15625, .15625, .15625,
    .15625, .15625, .15625, .15625, .15625,
};

struct Point { double x, y; };
struct Result {
  bool pass = true;
  std::string reason;
  double min_margin = std::numeric_limits<double>::infinity();
  double max_pitch_deg = 0.0;
  double min_base_z = std::numeric_limits<double>::infinity();
  double min_active_margin = std::numeric_limits<double>::infinity();
  int min_left_contacts = std::numeric_limits<int>::max();
  int min_right_contacts = std::numeric_limits<int>::max();
  double left_missing_s = 0.0;
  double right_missing_s = 0.0;
};

// Mirrors FlatController::ComputeObservation()/ComputeTargetQ() while using
// state directly from this headless MuJoCo instance instead of DDS.
class FlatPolicy {
 public:
  explicit FlatPolicy(const std::string& model_path)
      : env_(ORT_LOGGING_LEVEL_WARNING, "r1_crouch_com_test") {
    options_.SetIntraOpNumThreads(1);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), options_);
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
    last_action_.fill(0.0f);
  }

  std::array<double, kJoints> Step(const mjModel* m, const mjData* d) {
    std::array<float, 83> obs{};
    const int gyro_id = mj_name2id(m, mjOBJ_SENSOR, "imu_gyro");
    const int quat_id = mj_name2id(m, mjOBJ_SENSOR, "imu_quat");
    if (gyro_id < 0 || quat_id < 0) throw std::runtime_error("R1 IMU sensors missing");
    const mjtNum* gyro = d->sensordata + m->sensor_adr[gyro_id];
    const mjtNum* quat = d->sensordata + m->sensor_adr[quat_id];
    obs[0] = gyro[0]; obs[1] = gyro[1]; obs[2] = gyro[2];
    const float qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
    obs[3] = 2.0f * (qw * qy - qx * qz);
    obs[4] = -2.0f * (qy * qz + qw * qx);
    obs[5] = 2.0f * (qx * qx + qy * qy) - 1.0f;
    // commands and stand gait phase remain zero in this acceptance test.
    for (int a = 0; a < kJoints; ++a) {
      const int joint = m->actuator_trnid[2 * a];
      obs[11 + a] = static_cast<float>(d->qpos[m->jnt_qposadr[joint]] - kStand[a]);
      obs[35 + a] = static_cast<float>(d->qvel[m->jnt_dofadr[joint]]);
      obs[59 + a] = last_action_[a];
    }
    const std::array<int64_t, 2> shape{1, 83};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input = Ort::Value::CreateTensor<float>(memory, obs.data(), obs.size(), shape.data(), shape.size());
    const char* in = input_name_.c_str();
    const char* out = output_name_.c_str();
    auto output = session_->Run(Ort::RunOptions{nullptr}, &in, &input, 1, &out, 1);
    const float* action = output.front().GetTensorData<float>();
    std::array<double, kJoints> target{};
    for (int a = 0; a < kJoints; ++a) {
      last_action_[a] = action[a];
      target[a] = kStand[a] + action[a] * kActionScale[a];
    }
    return target;
  }

 private:
  Ort::Env env_;
  Ort::SessionOptions options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_, output_name_;
  std::array<float, kJoints> last_action_{};
};

double Clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
double Smooth5(double s) {
  s = Clamp(s, 0.0, 1.0);
  return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

std::vector<Point> Hull(std::vector<Point> p) {
  std::sort(p.begin(), p.end(), [](const Point& a, const Point& b) {
    return a.x == b.x ? a.y < b.y : a.x < b.x;
  });
  p.erase(std::unique(p.begin(), p.end(), [](const Point& a, const Point& b) {
    return std::hypot(a.x - b.x, a.y - b.y) < 1e-5;
  }), p.end());
  if (p.size() < 3) return {};
  auto cross = [](const Point& a, const Point& b, const Point& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  };
  std::vector<Point> h;
  for (const auto& q : p) {
    while (h.size() >= 2 && cross(h[h.size()-2], h.back(), q) <= 0.0) h.pop_back();
    h.push_back(q);
  }
  const size_t lower = h.size();
  for (auto it = p.rbegin(); it != p.rend(); ++it) {
    while (h.size() > lower && cross(h[h.size()-2], h.back(), *it) <= 0.0) h.pop_back();
    h.push_back(*it);
  }
  h.pop_back();
  return h;
}

double SignedMargin(const std::vector<Point>& hull, Point p) {
  if (hull.size() < 3) return -std::numeric_limits<double>::infinity();
  double margin = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < hull.size(); ++i) {
    const Point& a = hull[i];
    const Point& b = hull[(i + 1) % hull.size()];
    const double edge = std::hypot(b.x - a.x, b.y - a.y);
    margin = std::min(margin, ((b.x - a.x) * (p.y - a.y) -
                               (b.y - a.y) * (p.x - a.x)) / edge);
  }
  return margin;
}

bool IsFootGeom(const mjModel* m, int geom) {
  const char* name = mj_id2name(m, mjOBJ_GEOM, geom);
  return name && std::strstr(name, "_foot") != nullptr;
}

void AddFootSupport(const mjModel* m, const mjData* d, int geom, std::vector<Point>& points) {
  if (m->geom_type[geom] != mjGEOM_CAPSULE) return;
  const mjtNum* p = d->geom_xpos + 3 * geom;
  const mjtNum* r = d->geom_xmat + 9 * geom;
  // In MuJoCo a capsule's local Z axis joins its two endpoints.
  const double half = m->geom_size[3 * geom + 1];
  const double radius = m->geom_size[3 * geom];
  const Point endpoints[2] = {
      {p[0] - half * r[2], p[1] - half * r[5]},
      {p[0] + half * r[2], p[1] + half * r[5]},
  };
  // Small circles at each end make the projected sole patch conservative but
  // independent of which individual capsule point is currently carrying load.
  for (const Point& e : endpoints) {
    for (int i = 0; i < 8; ++i) {
      const double a = 2.0 * kPi * i / 8.0;
      points.push_back({e.x + radius * std::cos(a), e.y + radius * std::sin(a)});
    }
  }
}

double PelvisPitch(const mjData* d, int pelvis_id) {
  const mjtNum* q = d->xquat + 4 * pelvis_id;
  const double gx = 2.0 * (q[0] * q[2] - q[1] * q[3]);
  const double gz = 2.0 * (q[1] * q[1] + q[2] * q[2]) - 1.0;
  return std::atan2(gx, -gz);
}

std::array<double, kJoints> CrouchTarget(double phase, double pitch_meas,
                                          const std::array<double, kJoints>& start) {
  // A two-sided squat: no chair-transfer/rest pose is ever used.
  // Candidate taken from the former no-chair descent.  It is deliberately
  // retained here as a rejection test until a replacement passes all gates.
  constexpr double hip = -132.0 * kPi / 180.0;
  constexpr double knee = 90.0 * kPi / 180.0;
  constexpr double lean = 52.0 * kPi / 180.0;
  constexpr double spread = 0.08;
  constexpr double arm = -1.30;
  constexpr double elbow = 0.30;
  constexpr double kg = 0.40;
  const double e = Smooth5(phase);
  std::array<double, kJoints> q = start;
  q[L_HIP_P] = q[R_HIP_P] = start[L_HIP_P] + e * (hip - start[L_HIP_P]);
  q[L_KNEE] = q[R_KNEE] = start[L_KNEE] + e * (knee - start[L_KNEE]);
  q[L_HIP_R] = start[L_HIP_R] + e * (spread - start[L_HIP_R]);
  q[R_HIP_R] = start[R_HIP_R] + e * (-spread - start[R_HIP_R]);
  q[L_SHO_P] = q[R_SHO_P] = start[L_SHO_P] + e * (arm - start[L_SHO_P]);
  q[L_ELBOW] = q[R_ELBOW] = start[L_ELBOW] + e * (elbow - start[L_ELBOW]);
  const double lean_ref = e * lean;
  const double ankle_corr = Clamp(kg * (pitch_meas - lean_ref), -0.17, 0.17);
  const double ankle_pitch = Clamp(-(q[L_HIP_P] + q[L_KNEE]) - lean_ref - ankle_corr,
                                   -0.873, 0.576);
  q[L_ANK_P] = q[R_ANK_P] = start[L_ANK_P] + e * (ankle_pitch - start[L_ANK_P]);
  q[L_ANK_R] = start[L_ANK_R] + e * (-q[L_HIP_R] - start[L_ANK_R]);
  q[R_ANK_R] = start[R_ANK_R] + e * (-q[R_HIP_R] - start[R_ANK_R]);
  return q;
}

void SetPD(const mjModel* m, mjData* d, const std::array<double, kJoints>& target,
           double gain_scale, std::deque<std::array<double, kJoints>>& delay) {
  delay.push_back(target);
  const auto& delayed = delay.front();
  for (int a = 0; a < kJoints; ++a) {
    const int joint = m->actuator_trnid[2 * a];
    const int qadr = m->jnt_qposadr[joint];
    const int dadr = m->jnt_dofadr[joint];
    const bool ankle = a == L_ANK_P || a == L_ANK_R || a == R_ANK_P || a == R_ANK_R;
    const bool arm = a >= L_SHO_P;
    // Match the deployed Flat leg gains at hand-over; the slow quintic profile
    // stays within the same 100 Nm/rad position-control envelope.
    const double kp = (arm ? 40.0 : (ankle ? 40.0 : 100.0)) * gain_scale;
    const double kd = (arm ? 2.0 : (ankle ? 2.0 : 3.0)) * gain_scale;
    d->ctrl[a] = kp * (delayed[a] - d->qpos[qadr]) - kd * d->qvel[dadr];
  }
}

bool CheckFrame(const mjModel* m, const mjData* d, int floor_id, int pelvis_id,
                Result& result, double t, bool enforce_margin) {
  std::vector<Point> contacts;
  std::vector<Point> support, active_support;
  int left = 0, right = 0;
  bool nonfoot_floor = false;
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact& c = d->contact[i];
    const int other = c.geom1 == floor_id ? c.geom2 : (c.geom2 == floor_id ? c.geom1 : -1);
    if (other < 0) continue;
    if (!IsFootGeom(m, other)) {
      nonfoot_floor = true;
      continue;
    }
    const char* name = mj_id2name(m, mjOBJ_GEOM, other);
    contacts.push_back({c.pos[0], c.pos[1]});
    if (name && std::strstr(name, "left_")) ++left;
    if (name && std::strstr(name, "right_")) ++right;
  }
  for (int geom = 0; geom < m->ngeom; ++geom) {
    const char* name = mj_id2name(m, mjOBJ_GEOM, geom);
    if (!IsFootGeom(m, geom)) continue;
    AddFootSupport(m, d, geom, support);
    if ((name && std::strstr(name, "left_") && left > 0) ||
        (name && std::strstr(name, "right_") && right > 0))
      AddFootSupport(m, d, geom, active_support);
  }
  result.min_left_contacts = std::min(result.min_left_contacts, left);
  result.min_right_contacts = std::min(result.min_right_contacts, right);
  result.left_missing_s = left == 0 ? result.left_missing_s + m->opt.timestep : 0.0;
  result.right_missing_s = right == 0 ? result.right_missing_s + m->opt.timestep : 0.0;
  result.min_base_z = std::min(result.min_base_z, static_cast<double>(d->qpos[2]));
  result.max_pitch_deg = std::max(result.max_pitch_deg, std::fabs(PelvisPitch(d, pelvis_id)) * 180.0 / kPi);

  const auto hull = Hull(support);
  const auto active_hull = Hull(active_support);
  const Point com{d->subtree_com[3 * pelvis_id], d->subtree_com[3 * pelvis_id + 1]};
  const double margin = SignedMargin(hull, com);
  const double active_margin = SignedMargin(active_hull, com);
  result.min_margin = std::min(result.min_margin, margin);
  result.min_active_margin = std::min(result.min_active_margin, active_margin);
  if (!enforce_margin) return true;
  if (nonfoot_floor) result.reason = "non-foot contact with floor";
  else if ((result.left_missing_s > 0.10 || result.right_missing_s > 0.10) && active_margin <= 0.0)
    result.reason = "supporting-foot CoM margin is outside after a foot unload";
  else if (!std::isfinite(margin) || margin < 0.015) result.reason = "CoM margin below 15 mm";
  else if (result.max_pitch_deg > 65.0) result.reason = "pelvis pitch exceeded 65 deg";
  else if (d->qpos[2] < 0.36) result.reason = "base height dropped below fall threshold";
  if (!result.reason.empty()) {
    result.pass = false;
    double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
    double ymin = std::numeric_limits<double>::infinity(), ymax = -ymin;
    for (const auto& v : hull) {
      xmin = std::min(xmin, v.x); xmax = std::max(xmax, v.x);
      ymin = std::min(ymin, v.y); ymax = std::max(ymax, v.y);
    }
    std::cerr << "[FAIL] t=" << t << "s: " << result.reason
              << " (full/active margin=" << margin * 1000.0 << "/" << active_margin * 1000.0
              << "mm, CoM xy=" << com.x << "," << com.y
              << ", support x=" << xmin << ".." << xmax
              << ", y=" << ymin << ".." << ymax
              << ", contacts L/R=" << left << "/" << right << ")\n";
    return false;
  }
  return true;
}

bool RunScenario(mjModel* m, const std::string& label, int delay_steps,
                 double gain_scale, double mass_scale, double friction,
                 const std::string& policy_path, bool flat_only) {
  mjData* d = mj_makeData(m);
  for (int i = 0; i < m->nbody; ++i) {
    m->body_mass[i] *= mass_scale;
    for (int k = 0; k < 3; ++k) m->body_inertia[3 * i + k] *= mass_scale;
  }
  for (int i = 0; i < m->ngeom; ++i) m->geom_friction[3 * i] = friction;
  mj_setConst(m, d);
  const int floor = mj_name2id(m, mjOBJ_GEOM, "floor");
  const int pelvis = mj_name2id(m, mjOBJ_BODY, "pelvis");
  const int left_foot = mj_name2id(m, mjOBJ_SITE, "left_foot");
  const int right_foot = mj_name2id(m, mjOBJ_SITE, "right_foot");
  if (floor < 0 || pelvis < 0 || left_foot < 0 || right_foot < 0 || m->nu != kJoints) {
    std::cerr << "Model is missing R1 floor/pelvis/feet or has unexpected actuator count.\n";
    mj_deleteData(d);
    return false;
  }

  for (int a = 0; a < kJoints; ++a) {
    const int j = m->actuator_trnid[2 * a];
    d->qpos[m->jnt_qposadr[j]] = kStand[a];
  }
  mj_forward(m, d);
  // The foot sites are at the lowest point of each sole.  Put them on the floor.
  d->qpos[2] -= std::min(d->site_xpos[3 * left_foot + 2], d->site_xpos[3 * right_foot + 2]);
  mj_forward(m, d);

  std::deque<std::array<double, kJoints>> delay;
  for (int i = 0; i <= delay_steps; ++i) delay.push_back(kStand);
  FlatPolicy policy(policy_path);
  Result result;
  constexpr double warmup = 4.0, prepare = 1.2, descend = 8.0, hold = 2.0, rise = 8.0, finish = 1.2;
  const double total = warmup + prepare + descend + hold + rise + finish;
  const int steps = static_cast<int>(std::ceil(total / m->opt.timestep));
  std::array<double, kJoints> policy_target = kStand;
  std::array<double, kJoints> policy_base = kStand;
  bool crouch_started = false;
  for (int step = 0; step < steps; ++step) {
    const double t = step * m->opt.timestep;
    if (step % 4 == 0) policy_target = policy.Step(m, d);
    std::array<double, kJoints> target;
    if (t < warmup || flat_only) {
      target = policy_target;
    } else {
      const double crouch_t = t - warmup;
      if (!crouch_started) {
        policy_base = policy_target;
        crouch_started = true;
      }
      double phase = 0.0;
      if (crouch_t < prepare) {
        target = policy_target;
      } else {
        if (crouch_t < prepare + descend) phase = (crouch_t - prepare) / descend;
        else if (crouch_t < prepare + descend + hold) phase = 1.0;
        else if (crouch_t < prepare + descend + hold + rise)
          phase = 1.0 - (crouch_t - prepare - descend - hold) / rise;
        const auto crouch = CrouchTarget(phase, PelvisPitch(d, pelvis), policy_base);
        // Keep Flat's live corrections and add only the symmetric squat delta.
        target = policy_target;
        for (const int a : {L_HIP_P, L_HIP_R, L_KNEE, L_ANK_P, L_ANK_R,
                            R_HIP_P, R_HIP_R, R_KNEE, R_ANK_P, R_ANK_R,
                            L_SHO_P, L_ELBOW, R_SHO_P, R_ELBOW}) {
          target[a] += crouch[a] - policy_base[a];
        }
      }
    }
    SetPD(m, d, target, gain_scale, delay);
    if (delay.size() > static_cast<size_t>(delay_steps + 1)) delay.pop_front();
    mj_step(m, d);
    const bool enforce = t >= warmup;
    if (!CheckFrame(m, d, floor, pelvis, result, t, enforce)) break;
  }
  std::cout << "[" << label << "] " << (result.pass ? "PASS" : "FAIL")
            << " | min CoM margin=" << result.min_margin * 1000.0 << " mm"
            << " | min active-support margin=" << result.min_active_margin * 1000.0 << " mm"
            << " | max |pitch|=" << result.max_pitch_deg << " deg"
            << " | min base z=" << result.min_base_z << " m"
            << " | min foot contacts L/R=" << result.min_left_contacts << "/" << result.min_right_contacts
            << "\n";
  mj_deleteData(d);
  return result.pass;
}
}  // namespace

int main(int argc, char** argv) {
  mju_user_error = MuJoCoError;
  mju_user_warning = MuJoCoWarning;
  const char* scene = argc > 1 ? argv[1] : "../../unitree_robots/r1/scene_crouch_test.xml";
  const char* policy_path = argc > 2 ? argv[2] : "../../../HB/high_level_2/policies/flat/policy_5.onnx";
  const bool robust = argc > 3 && std::string(argv[3]) == "--robust";
  const bool flat_only = argc > 3 && std::string(argv[3]) == "--flat-only";
  char error[1024] = "";
  mjModel* base = mj_loadXML(scene, nullptr, error, sizeof(error));
  if (!base) {
    std::cerr << "Cannot load " << scene << ": " << error << "\n";
    return 2;
  }
  std::cout << "Testing no-chair squat after 4.0s Flat-policy warmup: prep 1.2s, descend 8.0s, hold 2.0s, reverse 8.0s.\n";
  bool ok = true;
  std::vector<std::array<double, 4>> scenarios{{0.0, 1.0, 1.0, 0.90}};
  if (robust) scenarios.push_back({3.0, 0.85, 1.10, 0.80});
  for (const auto& scenario : scenarios) {
    mjModel* copy = mj_copyModel(nullptr, base);
    const std::string label = scenario[0] == 0.0 ? "nominal" : "robust";
    ok = RunScenario(copy, label, static_cast<int>(scenario[0]), scenario[1], scenario[2], scenario[3], policy_path, flat_only) && ok;
    mj_deleteModel(copy);
  }
  mj_deleteModel(base);
  return ok ? 0 : 1;
}

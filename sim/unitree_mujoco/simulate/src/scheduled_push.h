#pragma once

#include <mujoco/mujoco.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Test-only world-frame force at a body's COM. Schedule rows:
// start_time_s duration_s body_name fx_N fy_N fz_N
// Add/remove around mj_step so interactive forces are preserved.
class ScheduledPush {
 public:
  ScheduledPush() {
    const char* path = std::getenv("MUJOCO_PUSH_SCHEDULE");
    if (!path || !*path) return;
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open MUJOCO_PUSH_SCHEDULE");
    std::string line;
    while (std::getline(input, line)) {
      if (line.empty() || line[0] == '#') continue;
      Event e;
      std::istringstream row(line);
      std::string extra;
      if (!(row >> e.start >> e.duration >> e.body >> e.force[0] >> e.force[1] >> e.force[2]) ||
          (row >> extra) || !std::isfinite(e.start) || e.start < 0 ||
          !std::isfinite(e.duration) || e.duration <= 0)
        throw std::runtime_error("Invalid push schedule row: " + line);
      for (double f : e.force)
        if (!std::isfinite(f)) throw std::runtime_error("Nonfinite push force");
      events_.push_back(e);
    }
  }
  void Step(const mjModel* m, mjData* d) {
    if (bound_ != m) {
      for (auto& e : events_) {
        e.id = mj_name2id(m, mjOBJ_BODY, e.body.c_str());
        if (e.id <= 0) throw std::runtime_error("Unknown/nonmoving push body: " + e.body);
      }
      bound_ = m;
    }
    for (auto& e : events_) {
      e.active = d->time >= e.start && d->time < e.start + e.duration;
      if (e.active) {
        if (!e.logged) {
          std::cout << "[PUSH] time=" << d->time << " body=" << e.body
                    << " force=" << e.force[0] << ',' << e.force[1] << ',' << e.force[2]
                    << " duration=" << e.duration << std::endl;
          e.logged = true;
        }
        for (int k=0; k<3; ++k) d->xfrc_applied[6*e.id+k] += e.force[k];
      }
    }
    mj_step(m, d);
    for (const auto& e : events_)
      if (e.active)
        for (int k=0; k<3; ++k) d->xfrc_applied[6*e.id+k] -= e.force[k];
  }
 private:
  struct Event {
    double start=0, duration=0, force[3]={};
    std::string body;
    int id=-1;
    bool active=false, logged=false;
  };
  std::vector<Event> events_;
  const mjModel* bound_=nullptr;
};

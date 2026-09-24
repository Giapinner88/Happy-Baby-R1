#include "scheduled_push.h"
#include <cassert>
#include <cstring>

int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string path=std::string(argv[1])+"/force_unit_schedule.txt";
  std::ofstream(path) << "0.01 0.2 pelvis 40 0 0\n";
  setenv("MUJOCO_PUSH_SCHEDULE", path.c_str(), 1);
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  const char* xml="<mujoco><option timestep='0.002' gravity='0 0 0'/><worldbody><body name='pelvis'><freejoint/><geom type='sphere' size='.1' mass='2'/></body></worldbody></mujoco>";
  mj_addBufferVFS(&vfs,"test.xml",xml,std::strlen(xml));
  char error[1024];
  auto* m=mj_loadXML("test.xml",&vfs,error,sizeof(error));
  assert(m);
  auto* d=mj_makeData(m);
  ScheduledPush push;
  d->xfrc_applied[6]=3; // preexisting external force must survive
  for(int i=0;i<200;++i) {
    push.Step(m,d);
    assert(std::abs(d->xfrc_applied[6]-3)<1e-10);
  }
  // 40 N * .2 s / 2 kg + 3 N * .4 s / 2 kg = 4.6 m/s.
  assert(std::abs(d->qvel[0]-4.6)<0.041);
  assert(std::abs(d->qvel[1])<1e-12);
  mj_deleteData(d); mj_deleteModel(m); mj_deleteVFS(&vfs);
  std::cout << "PUSH_CONTRACT_PASS: impulse, body indexing, duration, force preservation\n";
}

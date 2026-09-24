#include <mujoco/mujoco.h>
#include <iostream>
int main(){ std::cout << "Version: " << mj_versionString() << std::endl; return 0; }

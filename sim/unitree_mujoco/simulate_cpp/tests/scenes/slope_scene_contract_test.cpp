#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void Check(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
}

bool Near(double actual, double expected, double tolerance = 1e-6) {
    return std::abs(actual - expected) <= tolerance;
}

void WorldPoint(const mjModel* model, int geom_id, const mjtNum local[3], mjtNum world[3]) {
    mju_rotVecQuat(world, local, model->geom_quat + 4 * geom_id);
    for (int axis = 0; axis < 3; ++axis) {
        world[axis] += model->geom_pos[3 * geom_id + axis];
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: slope_scene_contract_test SCENE_XML ANGLE_DEG {direct|platform}\n";
        return 2;
    }

    const std::string scene = argv[1];
    const int angle_deg = std::atoi(argv[2]);
    const std::string mode = argv[3];
    if (mode != "direct" && mode != "platform") {
        std::cerr << "FAIL: mode must be direct or platform\n";
        return 2;
    }
    const std::string suffix = std::to_string(angle_deg) + "deg";

    char error[1024] = {};
    mjModel* model = mj_loadXML(scene.c_str(), nullptr, error, sizeof(error));
    if (!model) {
        std::cerr << "FAIL: cannot load " << scene << ": " << error << '\n';
        return 1;
    }

    Check(model->nkey >= 1, "scene has a standing keyframe");
    Check(model->nq == 31, "scene preserves the R1 nq=31 contract");
    Check(model->nu == 24, "scene preserves the R1 nu=24 contract");
    if (model->nkey >= 1 && model->nq >= 7) {
        const mjtNum* q = model->key_qpos;
        Check(Near(q[0], 0.0) && Near(q[1], 0.0) && Near(q[2], 0.76),
              "keyframe pelvis position is training reset (0,0,0.76)");
        Check(Near(q[3], 1.0) && Near(q[4], 0.0) && Near(q[5], 0.0) && Near(q[6], 0.0),
              "keyframe pelvis quaternion is upright identity");
    }

    const double expected_angle = angle_deg * mjPI / 180.0;
    std::vector<int> terrain_geoms;
    if (mode == "platform") {
        const int platform = mj_name2id(
            model, mjOBJ_GEOM, ("spawn_platform_" + suffix).c_str());
        const int uphill = mj_name2id(
            model, mjOBJ_GEOM, ("slope_" + suffix + "_uphill").c_str());
        const int downhill = mj_name2id(
            model, mjOBJ_GEOM, ("slope_" + suffix + "_downhill").c_str());
        Check(platform >= 0, "spawn platform geom exists");
        Check(uphill >= 0, "uphill ramp geom exists");
        Check(downhill >= 0, "downhill ramp geom exists");
        for (int geom_id : {platform, uphill, downhill}) {
            if (geom_id >= 0) terrain_geoms.push_back(geom_id);
        }

        if (platform >= 0) {
            Check(model->geom_type[platform] == mjGEOM_BOX, "spawn platform is a box");
            Check(Near(model->geom_size[3 * platform], 1.0),
                  "spawn platform is exactly 2 m wide with no ramp overlap or gap");
            Check(Near(model->geom_pos[3 * platform + 2] +
                       model->geom_size[3 * platform + 2], 0.0),
                  "spawn platform top is z=0");
        }

        if (uphill >= 0 && downhill >= 0) {
            const mjtNum uphill_start_local[3] = {
                -model->geom_size[3 * uphill], 0.0, model->geom_size[3 * uphill + 2]};
            const mjtNum uphill_end_local[3] = {
                model->geom_size[3 * uphill], 0.0, model->geom_size[3 * uphill + 2]};
            const mjtNum downhill_join_local[3] = {
                model->geom_size[3 * downhill], 0.0, model->geom_size[3 * downhill + 2]};
            mjtNum uphill_start[3], uphill_end[3], downhill_join[3];
            WorldPoint(model, uphill, uphill_start_local, uphill_start);
            WorldPoint(model, uphill, uphill_end_local, uphill_end);
            WorldPoint(model, downhill, downhill_join_local, downhill_join);

            Check(Near(uphill_start[0], 1.0, 2e-6) && Near(uphill_start[2], 0.0, 2e-6),
                  "uphill ramp joins platform continuously at x=+1");
            Check(Near(downhill_join[0], -1.0, 2e-6) && Near(downhill_join[2], 0.0, 2e-6),
                  "downhill ramp joins platform continuously at x=-1");

            const double actual_angle = std::atan2(
                uphill_end[2] - uphill_start[2], uphill_end[0] - uphill_start[0]);
            Check(Near(actual_angle, expected_angle, 2e-7),
                  "compiled uphill surface angle is exact");

            for (int geom_id : {uphill, downhill}) {
                Check(model->geom_priority[geom_id] == 1, "ramp contact priority is 1");
                Check(Near(model->geom_friction[3 * geom_id], 1.0),
                      "compiled ramp sliding friction is 1.0");
                Check(model->geom_group[geom_id] <= 2,
                      "ramp is visible to Rough height scan");
            }

            std::cout << "platform slope " << angle_deg << "deg: joins=("
                      << uphill_start[0] << ',' << uphill_start[2] << "),("
                      << downhill_join[0] << ',' << downhill_join[2]
                      << "), friction=" << model->geom_friction[3 * uphill] << '\n';
        }
    } else {
        const int direct = mj_name2id(
            model, mjOBJ_GEOM, ("slope_" + suffix + "_direct").c_str());
        Check(direct >= 0, "direct slope geom exists");
        if (direct >= 0) {
            terrain_geoms.push_back(direct);
            Check(model->geom_type[direct] == mjGEOM_PLANE,
                  "direct slope is one infinite plane with no flat patch");

            const mjtNum local_normal[3] = {0.0, 0.0, 1.0};
            mjtNum world_normal[3];
            mju_rotVecQuat(world_normal, local_normal, model->geom_quat + 4 * direct);
            Check(Near(world_normal[0], -std::sin(expected_angle), 2e-7) &&
                      Near(world_normal[1], 0.0, 2e-7) &&
                      Near(world_normal[2], std::cos(expected_angle), 2e-7),
                  "direct plane normal encodes the requested slope exactly");
            const double actual_angle = std::atan2(-world_normal[0], world_normal[2]);
            Check(Near(actual_angle, expected_angle, 2e-7),
                  "compiled direct surface angle is exact");
            Check(model->geom_priority[direct] == 1,
                  "direct slope contact priority is 1");
            Check(Near(model->geom_friction[3 * direct], 1.0),
                  "compiled direct slope sliding friction is 1.0");
            Check(model->geom_group[direct] <= 2,
                  "direct slope is visible to Rough height scan");

            std::cout << "direct slope " << angle_deg << "deg: normal=("
                      << world_normal[0] << ',' << world_normal[1] << ','
                      << world_normal[2] << "), friction="
                      << model->geom_friction[3 * direct] << '\n';
        }
    }

    // Step from the standing keyframe and inspect the contact parameters that
    // MuJoCo actually compiled, rather than trusting only the XML geom value.
    mjData* data = mj_makeData(model);
    Check(data != nullptr, "MuJoCo data allocation succeeds");
    if (data) {
        mj_resetDataKeyframe(model, data, 0);
        bool saw_terrain_contact = false;
        bool effective_friction_is_one = true;
        for (int step = 0; step < 500; ++step) {
            mj_step(model, data);
            for (int contact_id = 0; contact_id < data->ncon; ++contact_id) {
                const mjContact& contact = data->contact[contact_id];
                bool touches_terrain = false;
                for (int geom_id : terrain_geoms) {
                    if (contact.geom[0] == geom_id || contact.geom[1] == geom_id) {
                        touches_terrain = true;
                        break;
                    }
                }
                if (!touches_terrain) continue;
                saw_terrain_contact = true;
                effective_friction_is_one &= Near(contact.friction[0], 1.0);
            }
        }
        Check(saw_terrain_contact, "standing reset produces terrain contact");
        Check(effective_friction_is_one,
              "effective compiled terrain contact sliding friction is 1.0");
        mj_deleteData(data);
    }

    mj_deleteModel(model);
    if (failures != 0) return 1;
    std::cout << "slope_scene_contract_test: PASS\n";
    return 0;
}

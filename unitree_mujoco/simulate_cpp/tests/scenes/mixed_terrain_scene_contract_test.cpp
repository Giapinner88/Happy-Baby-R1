#include <mujoco/mujoco.h>

#include "simulator/RaycastGeomFilter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
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

int RequireGeom(const mjModel* model, const std::string& name) {
    const int id = mj_name2id(model, mjOBJ_GEOM, name.c_str());
    Check(id >= 0, "geom exists: " + name);
    return id;
}

void WorldPoint(const mjModel* model, int geom_id, const mjtNum local[3], mjtNum world[3]) {
    mju_rotVecQuat(world, local, model->geom_quat + 4 * geom_id);
    for (int axis = 0; axis < 3; ++axis) {
        world[axis] += model->geom_pos[3 * geom_id + axis];
    }
}

double BoxTop(const mjModel* model, int geom_id) {
    return model->geom_pos[3 * geom_id + 2] + model->geom_size[3 * geom_id + 2];
}

bool IsFoot(const mjModel* model, int geom_id) {
    const char* name = mj_id2name(model, mjOBJ_GEOM, geom_id);
    return name && std::strstr(name, "_foot") && std::strstr(name, "_collision");
}

double RayHeight(const mjModel* model, mjData* data, double x, double y) {
    const mjtNum start[3] = {x, y, 3.0};
    const mjtNum direction[3] = {0.0, 0.0, -1.0};
    const mjtByte geom_group[6] = {1, 1, 1, 0, 0, 0};
    int geom_id = -1;
    const int pelvis = mj_name2id(model, mjOBJ_BODY, "pelvis");
    const mjtNum distance = mj_ray(
        model, data, start, direction, geom_group, 1, pelvis, &geom_id);
    Check(distance >= 0.0, "ray hits terrain at (" + std::to_string(x) + "," +
                               std::to_string(y) + ")");
    return distance >= 0.0 ? start[2] - distance : std::numeric_limits<double>::quiet_NaN();
}

int RayGeom(const mjModel* model, mjData* data, double x, double y, double z,
            double* ground_z = nullptr) {
    const mjtNum start[3] = {x, y, z};
    const mjtNum direction[3] = {0.0, 0.0, -1.0};
    const mjtByte geom_group[6] = {1, 1, 1, 0, 0, 0};
    int geom_id = -1;
    const int pelvis = mj_name2id(model, mjOBJ_BODY, "pelvis");
    const mjtNum distance = mj_ray(
        model, data, start, direction, geom_group, 1, pelvis, &geom_id);
    if (ground_z) {
        *ground_z = distance >= 0.0 ? z - distance
                                    : std::numeric_limits<double>::quiet_NaN();
    }
    return distance >= 0.0 && distance <= 5.0 ? geom_id : -1;
}

void CheckDownstairsScan(const mjModel* model, mjData* data, int step_cm,
                         double lane_x) {
    const std::string prefix = "stairs_" + std::to_string(step_cm);
    const double increment = step_cm / 100.0;
    mj_resetDataKeyframe(model, data, 0);
    const double s = std::sqrt(0.5);
    data->qpos[0] = lane_x;
    data->qpos[1] = 5.15;  // trên landing, cách bậc xuống đầu tiên 25 cm
    data->qpos[2] = 0.734 + 4.0 * increment;
    data->qpos[3] = s;
    data->qpos[4] = 0.0;
    data->qpos[5] = 0.0;
    data->qpos[6] = s;     // yaw +90 deg: forward robot là world +Y
    mj_forward(model, data);

    int terrain_hits = 0;
    for (int lateral_idx = 0; lateral_idx < 11; ++lateral_idx) {
        const double lateral = -0.5 + 0.1 * lateral_idx;
        for (int forward_idx = 0; forward_idx < 17; ++forward_idx) {
            const double forward = -0.8 + 0.1 * forward_idx;
            const int geom_id = RayGeom(
                model, data, lane_x - lateral, 5.15 + forward, data->qpos[2]);
            if (geom_id >= 0 && model->geom_bodyid[geom_id] == 0) ++terrain_hits;
        }
    }
    Check(terrain_hits == 187,
          prefix + " downhill 17x11 scan sees terrain on all 187 rays");

    double first_ground_z = 0.0;
    double second_ground_z = 0.0;
    const int first = RayGeom(model, data, lane_x, 5.45, data->qpos[2], &first_ground_z);
    const int second = RayGeom(model, data, lane_x, 5.85, data->qpos[2], &second_ground_z);
    const char* first_name = first >= 0 ? mj_id2name(model, mjOBJ_GEOM, first) : nullptr;
    const char* second_name = second >= 0 ? mj_id2name(model, mjOBJ_GEOM, second) : nullptr;
    Check(first_name && std::string(first_name) == prefix + "_down_1" &&
              Near(first_ground_z, 3.0 * increment, 2e-6),
          prefix + " scan sees first descending tread 30 cm ahead");
    Check(second_name && std::string(second_name) == prefix + "_down_2" &&
              Near(second_ground_z, 2.0 * increment, 2e-6),
          prefix + " scan sees second descending tread 70 cm ahead");
}

void CheckSlope(const mjModel* model, mjData* data, int angle_deg, double lane_y,
                std::vector<int>& terrain_geoms) {
    const std::string prefix = "slope_" + std::to_string(angle_deg);
    const int up = RequireGeom(model, prefix + "_up");
    const int plateau = RequireGeom(model, prefix + "_plateau");
    const int down = RequireGeom(model, prefix + "_down");
    if (up < 0 || plateau < 0 || down < 0) return;
    terrain_geoms.insert(terrain_geoms.end(), {up, plateau, down});

    const mjtNum half_length = model->geom_size[3 * up];
    const mjtNum half_thickness = model->geom_size[3 * up + 2];
    const mjtNum up_start_local[3] = {-half_length, 0.0, half_thickness};
    const mjtNum up_end_local[3] = {half_length, 0.0, half_thickness};
    const mjtNum down_start_local[3] = {-half_length, 0.0, half_thickness};
    const mjtNum down_end_local[3] = {half_length, 0.0, half_thickness};
    mjtNum up_start[3], up_end[3], down_start[3], down_end[3];
    WorldPoint(model, up, up_start_local, up_start);
    WorldPoint(model, up, up_end_local, up_end);
    WorldPoint(model, down, down_start_local, down_start);
    WorldPoint(model, down, down_end_local, down_end);

    const double expected_angle = angle_deg * mjPI / 180.0;
    const double expected_height = 3.0 * std::sin(expected_angle);
    const double up_angle = std::atan2(up_end[2] - up_start[2], up_end[0] - up_start[0]);
    const double down_angle = std::atan2(
        down_end[2] - down_start[2], down_end[0] - down_start[0]);
    const double up_surface_length = std::hypot(
        up_end[0] - up_start[0], up_end[2] - up_start[2]);

    Check(Near(up_angle, expected_angle, 2e-7), prefix + " uphill angle is exact");
    Check(Near(down_angle, -expected_angle, 2e-7), prefix + " downhill angle is exact");
    Check(Near(up_surface_length, 3.0, 2e-7), prefix + " inclined surface is 3 m long");
    Check(Near(up_start[0], 3.0, 2e-7) && Near(up_start[2], 0.0, 2e-7),
          prefix + " starts continuously at floor z=0");
    Check(Near(up_end[2], expected_height, 2e-7), prefix + " reaches expected height");
    Check(Near(up_end[0], model->geom_pos[3 * plateau] - model->geom_size[3 * plateau], 2e-7) &&
              Near(BoxTop(model, plateau), expected_height, 2e-7),
          prefix + " uphill joins plateau continuously");
    Check(Near(down_start[0], model->geom_pos[3 * plateau] + model->geom_size[3 * plateau], 2e-7) &&
              Near(down_start[2], expected_height, 2e-7),
          prefix + " plateau joins downhill continuously");
    Check(Near(down_end[2], 0.0, 2e-7), prefix + " returns continuously to floor z=0");
    Check(Near(model->geom_pos[3 * up + 1], lane_y) &&
              Near(model->geom_pos[3 * plateau + 1], lane_y) &&
              Near(model->geom_pos[3 * down + 1], lane_y),
          prefix + " stays in its independent lane");

    const double ray_x = 0.5 * (up_start[0] + up_end[0]);
    Check(Near(RayHeight(model, data, ray_x, lane_y), 0.5 * expected_height, 2e-6),
          prefix + " is visible at the expected height to Rough raycasting");
}

void CheckStairs(const mjModel* model, mjData* data, int step_cm, double lane_x,
                 std::vector<int>& terrain_geoms) {
    const std::string prefix = "stairs_" + std::to_string(step_cm);
    const double increment = step_cm / 100.0;
    double previous_upper_y = 3.0;

    for (int step = 1; step <= 4; ++step) {
        const int id = RequireGeom(model, prefix + "_up_" + std::to_string(step));
        if (id < 0) continue;
        terrain_geoms.push_back(id);
        Check(Near(BoxTop(model, id), step * increment),
              prefix + " uphill step " + std::to_string(step) + " has exact height");
        Check(Near(2.0 * model->geom_size[3 * id + 1], 0.35),
              prefix + " tread depth is 35 cm");
        const double lower_y = model->geom_pos[3 * id + 1] - model->geom_size[3 * id + 1];
        Check(Near(lower_y, previous_upper_y), prefix + " uphill treads have no gap");
        previous_upper_y = model->geom_pos[3 * id + 1] + model->geom_size[3 * id + 1];
    }

    const int landing = RequireGeom(model, prefix + "_landing");
    if (landing >= 0) {
        terrain_geoms.push_back(landing);
        Check(Near(BoxTop(model, landing), 4.0 * increment),
              prefix + " landing has exact maximum height");
        Check(Near(2.0 * model->geom_size[3 * landing + 1], 1.0),
              prefix + " landing is 1 m long");
        Check(Near(model->geom_pos[3 * landing + 1] - model->geom_size[3 * landing + 1],
                   previous_upper_y),
              prefix + " uphill joins landing without a gap");
        previous_upper_y = model->geom_pos[3 * landing + 1] +
                           model->geom_size[3 * landing + 1];
    }

    for (int step = 1; step <= 3; ++step) {
        const int id = RequireGeom(model, prefix + "_down_" + std::to_string(step));
        if (id < 0) continue;
        terrain_geoms.push_back(id);
        Check(Near(BoxTop(model, id), (4 - step) * increment),
              prefix + " downhill step " + std::to_string(step) + " has exact height");
        const double lower_y = model->geom_pos[3 * id + 1] - model->geom_size[3 * id + 1];
        Check(Near(lower_y, previous_upper_y), prefix + " downhill treads have no gap");
        previous_upper_y = model->geom_pos[3 * id + 1] + model->geom_size[3 * id + 1];
    }

    Check(Near(RayHeight(model, data, lane_x, 4.225), 4.0 * increment, 2e-6),
          prefix + " is visible at the expected height to Rough raycasting");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: mixed_terrain_scene_contract_test SCENE_XML\n";
        return 2;
    }

    char error[1024] = {};
    mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!model) {
        std::cerr << "FAIL: cannot load " << argv[1] << ": " << error << '\n';
        return 1;
    }
    mjData* data = mj_makeData(model);
    Check(data != nullptr, "MuJoCo data allocation succeeds");
    if (!data) {
        mj_deleteModel(model);
        return 1;
    }
    const int pelvis = mj_name2id(model, mjOBJ_BODY, "pelvis");
    Check(pelvis >= 0, "scene contains pelvis body for Rough ray filtering");
    const int excluded_robot_geoms =
        RaycastGeomFilter::ExcludeBodySubtree(model, pelvis);
    Check(excluded_robot_geoms > 0,
          "Rough ray filter excludes the complete robot body subtree");

    mj_resetDataKeyframe(model, data, 0);
    mj_forward(model, data);

    Check(model->nq == 31, "scene preserves the R1 nq=31 contract");
    Check(model->nu == 24, "scene preserves the R1 nu=24 contract");
    Check(model->nkey >= 1, "scene has a standing keyframe");
    if (model->nkey >= 1) {
        const mjtNum* q = model->key_qpos;
        Check(Near(q[0], 0.0) && Near(q[1], 0.0) && Near(q[2], 0.734),
              "standing keyframe is in the clear flat spawn zone");
        Check(Near(q[3], 1.0) && Near(q[4], 0.0) && Near(q[5], 0.0) && Near(q[6], 0.0),
              "standing keyframe pelvis quaternion is upright identity");
    }

    std::vector<int> terrain_geoms;
    const int floor = RequireGeom(model, "mixed_floor");
    if (floor >= 0) {
        terrain_geoms.push_back(floor);
        Check(model->geom_type[floor] == mjGEOM_PLANE, "wide base floor is a plane");
    }

    CheckSlope(model, data, 5, 3.0, terrain_geoms);
    CheckSlope(model, data, 15, 0.0, terrain_geoms);
    CheckSlope(model, data, 30, -3.0, terrain_geoms);
    CheckStairs(model, data, 5, -1.0, terrain_geoms);
    CheckStairs(model, data, 10, 1.0, terrain_geoms);
    CheckDownstairsScan(model, data, 5, -1.0);
    CheckDownstairsScan(model, data, 10, 1.0);

    // Các kiểm tra còn lại bắt đầu lại từ standing reset trên nền phẳng.
    mj_resetDataKeyframe(model, data, 0);
    mj_forward(model, data);

    double min_rough_height = std::numeric_limits<double>::infinity();
    double max_rough_height = -std::numeric_limits<double>::infinity();
    for (int tile = 0; tile < 64; ++tile) {
        const std::string suffix = tile < 10 ? "0" + std::to_string(tile) : std::to_string(tile);
        const int id = RequireGeom(model, "rough_tile_" + suffix);
        if (id < 0) continue;
        terrain_geoms.push_back(id);
        const double height = BoxTop(model, id);
        min_rough_height = std::min(min_rough_height, height);
        max_rough_height = std::max(max_rough_height, height);
        Check(Near(model->geom_size[3 * id], 0.251) &&
                  Near(model->geom_size[3 * id + 1], 0.251),
              "rough tiles overlap slightly so the patch has no holes");
    }
    Check(Near(min_rough_height, 0.02) && Near(max_rough_height, 0.08),
          "rough patch has deterministic 2-8 cm relief");
    Check(Near(RayHeight(model, data, -5.25, -0.75), 0.08, 2e-6),
          "rough patch is visible at the expected height to Rough raycasting");

    for (int obstacle = 1; obstacle <= 6; ++obstacle) {
        const std::string suffix = obstacle < 10 ? "0" + std::to_string(obstacle)
                                                 : std::to_string(obstacle);
        const int id = RequireGeom(model, "low_obstacle_" + suffix);
        if (id < 0) continue;
        terrain_geoms.push_back(id);
        Check(Near(BoxTop(model, id), 0.05),
              "low obstacle " + suffix + " top is exactly 5 cm");
    }
    Check(Near(RayHeight(model, data, 0.0, -3.5), 0.05, 2e-6),
          "5 cm obstacle is visible to Rough raycasting");

    Check(terrain_geoms.size() == 96,
          "scene contains exactly 96 named terrain collision geoms");
    for (int id : terrain_geoms) {
        Check(model->geom_priority[id] == 1, "all terrain contact priorities are 1");
        Check(model->geom_condim[id] == 3, "all terrain geoms use condim=3");
        Check(model->geom_group[id] <= 2, "all terrain geoms are visible to Rough raycasting");
        Check(Near(model->geom_friction[3 * id], 1.0),
              "all terrain sliding friction values compile to 1.0");
    }

    for (const auto& point : std::vector<std::pair<double, double>>{
             {-1.9, -1.9}, {-1.9, 1.9}, {1.9, -1.9}, {1.9, 1.9}}) {
        Check(Near(RayHeight(model, data, point.first, point.second), 0.0, 2e-6),
              "central 4x4 m spawn zone remains flat and clear");
    }

    bool saw_foot_floor_contact = false;
    bool effective_contact_is_correct = true;
    for (int step = 0; step < 500; ++step) {
        mj_step(model, data);
        for (int contact_id = 0; contact_id < data->ncon; ++contact_id) {
            const mjContact& contact = data->contact[contact_id];
            const bool foot_floor =
                (contact.geom[0] == floor && IsFoot(model, contact.geom[1])) ||
                (contact.geom[1] == floor && IsFoot(model, contact.geom[0]));
            if (!foot_floor) continue;
            saw_foot_floor_contact = true;
            effective_contact_is_correct &= contact.dim == 6 &&
                                            Near(contact.friction[0], 1.0);
        }
    }
    Check(saw_foot_floor_contact, "standing reset produces foot-floor contact");
    Check(effective_contact_is_correct,
          "effective foot-floor contact uses dim=6 and sliding friction=1.0");

    mj_deleteData(data);
    mj_deleteModel(model);
    if (failures != 0) return 1;
    std::cout << "mixed_terrain_scene_contract_test: PASS\n";
    return 0;
}

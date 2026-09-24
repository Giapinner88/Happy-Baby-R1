#pragma once

#include <mujoco/mujoco.h>

namespace RaycastGeomFilter {

inline bool IsBodyInSubtree(const mjModel* model, int body_id, int root_body_id) {
    while (body_id > 0) {
        if (body_id == root_body_id) return true;
        body_id = model->body_parentid[body_id];
    }
    return root_body_id == 0 && body_id == 0;
}

// MuJoCo mj_ray chỉ nhận một bodyexclude, nên các geom ở link con (hip/elbow/...)
// vẫn có thể che height scan. Chuyển toàn bộ geom trong subtree robot sang group 5;
// Rough ray chỉ bật group 0/1/2, còn mô hình simulator thật không bị thay đổi.
inline int ExcludeBodySubtree(mjModel* model, int root_body_id, int excluded_group = 5) {
    if (!model || root_body_id < 0 || excluded_group < 0 || excluded_group >= mjNGROUP) {
        return 0;
    }

    int excluded = 0;
    for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
        if (!IsBodyInSubtree(model, model->geom_bodyid[geom_id], root_body_id)) continue;
        model->geom_group[geom_id] = excluded_group;
        ++excluded;
    }
    return excluded;
}

}  // namespace RaycastGeomFilter

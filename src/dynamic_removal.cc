#include "dynamic_removal.h"

#include <cmath>

namespace faster_lio {

void DynamicObjectFilter::ClassifyGround(PointCloudType::Ptr &cloud_body) {
    if (!options_.enable || !cloud_body) {
        return;
    }

    for (auto &pt : cloud_body->points) {
        // Simple height-based ground classification in body (LiDAR) frame.
        // Points whose z-coordinate is below the threshold are considered ground.
        if (pt.z < options_.ground_height_threshold) {
            pt.normal_z = LABEL_GROUND;
        } else {
            pt.normal_z = LABEL_NON_GROUND;
        }
    }
}

void DynamicObjectFilter::DetectDynamic(const PointCloudType::Ptr &cloud_body,
                                        const PointCloudType::Ptr &cloud_world,
                                        const std::vector<PointVector> &nearest_points,
                                        std::vector<bool> &is_dynamic) {
    if (!options_.enable || !cloud_body || !cloud_world) {
        return;
    }

    const int num_points = static_cast<int>(cloud_body->size());
    is_dynamic.assign(num_points, false);

    for (int i = 0; i < num_points; ++i) {
        const PointType &pt_body = cloud_body->points[i];

        // Points well above the sensor (e.g. trees, buildings) are assumed
        // to be static structures.
        if (pt_body.z > options_.dynamic_height_max) {
            continue;
        }

        const float label = pt_body.normal_z;  // ground label of current point

        // Only non-ground points can be classified as dynamic.
        if (label == LABEL_GROUND) {
            continue;
        }

        const auto &neighbours = nearest_points[i];

        // Not enough neighbours → cannot decide.
        if (static_cast<int>(neighbours.size()) < options_.min_neighbor_points) {
            continue;
        }

        // Count how many neighbours are ground points.
        int num_ground = 0;
        for (const auto &nb : neighbours) {
            if (nb.normal_z == LABEL_GROUND) {
                ++num_ground;
            }
        }

        double ground_ratio =
            static_cast<double>(num_ground) / static_cast<double>(neighbours.size());

        // If the surrounding map area is predominantly ground but the
        // incoming point is non-ground, it is likely a dynamic object.
        if (ground_ratio > options_.dynamic_ground_ratio) {
            is_dynamic[i] = true;
        }
    }
}

}  // namespace faster_lio

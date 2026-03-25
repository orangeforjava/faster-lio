#ifndef FASTER_LIO_DYNAMIC_REMOVAL_H
#define FASTER_LIO_DYNAMIC_REMOVAL_H

#include <vector>

#include "common_lib.h"

namespace faster_lio {

/// Ground label constants stored in the normal_z field of PointXYZINormal
constexpr float LABEL_GROUND = 0.0f;
constexpr float LABEL_NON_GROUND = 1.0f;

/**
 * Dynamic object filter based on label consistency (reference: dynamic_lio).
 *
 * Approach:
 *  1. Classify each point in the body frame as ground or non-ground using a
 *     simple height threshold (the z-coordinate in the LiDAR body frame is
 *     compared against a configurable threshold).
 *  2. After state estimation, check label consistency between each new
 *     point and its nearest neighbours already stored in the map.
 *     - A non-ground point whose neighbours are predominantly ground is
 *       considered dynamic and excluded from the map update.
 *  3. Only static points are added to the IVox local map.
 */
class DynamicObjectFilter {
   public:
    struct Options {
        bool enable = false;                     ///< master switch
        float ground_height_threshold = -1.5f;   ///< points with z < threshold in body frame are ground
        float dynamic_height_max = 3.5f;         ///< points higher than this in body frame are assumed static
        float dynamic_ground_ratio = 0.1f;       ///< if ratio of ground neighbours > this, mark non-ground point as dynamic
        int min_neighbor_points = 5;             ///< minimum neighbours needed to apply consistency check
    };

    DynamicObjectFilter() = default;
    ~DynamicObjectFilter() = default;

    Options &GetOptions() { return options_; }
    const Options &GetOptions() const { return options_; }

    bool IsEnabled() const { return options_.enable; }

    /**
     * Classify points in the body-frame cloud as ground / non-ground.
     * The label is written into the normal_z field of each point:
     *   normal_z == LABEL_GROUND     (0)  -> ground
     *   normal_z == LABEL_NON_GROUND (1)  -> non-ground
     */
    void ClassifyGround(PointCloudType::Ptr &cloud_body);

    /**
     * Detect dynamic points by checking label consistency between the
     * current scan and the local map.
     *
     * @param cloud_body      downsampled scan in body frame (normal_z stores ground label)
     * @param cloud_world     downsampled scan in world frame
     * @param nearest_points  per-point nearest neighbours from the IVox map
     * @param is_dynamic      [out] per-point boolean, true = dynamic
     */
    void DetectDynamic(const PointCloudType::Ptr &cloud_body,
                       const PointCloudType::Ptr &cloud_world,
                       const std::vector<PointVector> &nearest_points,
                       std::vector<bool> &is_dynamic);

   private:
    Options options_;
};

}  // namespace faster_lio

#endif  // FASTER_LIO_DYNAMIC_REMOVAL_H

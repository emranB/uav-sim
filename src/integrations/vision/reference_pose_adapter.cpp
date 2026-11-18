/**
 * Bridge verified image poses into ORB-SLAM's normal local-map validation.
 */
#include "reference_pose_adapter.h"
#include "reference_pose.h"
#include "Converter.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include <limits>

namespace ORB_SLAM3 {
static bool seed_from_points(Frame &frame, const cv::Mat &descriptors,
                             const std::vector<MapPoint*> &points, Map *active_map) {
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    std::vector<cv::Point3f> world(descriptors.rows, {invalid, invalid, invalid});
    for (size_t index = 0; index < points.size() && index < world.size(); ++index) {
        if (points[index] != nullptr && !points[index]->isBad() &&
            points[index]->GetMap() == active_map) {
            const auto position = points[index]->GetWorldPos();
            world[index] = {position.x(), position.y(), position.z()};
        }
    }
    std::vector<cv::Point2f> pixels;
    for (const auto &feature : frame.mvKeysUn) {
        pixels.push_back(feature.pt);
    }
    observation::PoseSeed seed;
    if (!observation::estimate_reference_pose(descriptors, world,
                                             frame.mDescriptors, pixels, frame.mK, seed)) {
        return false;
    }
    std::fill(frame.mvpMapPoints.begin(), frame.mvpMapPoints.end(), nullptr);
    for (const auto &association : seed.associations) {
        frame.mvpMapPoints[association.first] = points[association.second];
    }
    frame.SetPose(Converter::toSophus(seed.camera_from_map));
    return true;
}

bool seed_reference_frame(Frame &frame, const Frame &previous, KeyFrame *reference, Map *active_map) {
    // The generic matcher is calibrated for the rectified pinhole path only.
    if (frame.Nleft != -1 || active_map == nullptr) {
        return false;
    }
    if (previous.HasPose() && previous.mpReferenceKF != nullptr &&
        previous.mpReferenceKF->GetMap() == active_map) {
        auto points = previous.mvpMapPoints;
        for (size_t index = 0; index < points.size(); ++index) {
            if (previous.mvbOutlier[index]) {
                points[index] = nullptr;
            }
        }
        if (seed_from_points(frame, previous.mDescriptors, points, active_map)) {
            return true;
        }
    }
    return reference != nullptr && !reference->isBad() && reference->GetMap() == active_map &&
        seed_from_points(frame, reference->mDescriptors, reference->GetMapPointMatches(), active_map);
}
}

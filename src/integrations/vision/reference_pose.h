/**
 * Seed observational image poses from measured stereo correspondences.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

namespace observation {
struct PoseSeed {
    cv::Mat camera_from_map;
    std::vector<std::pair<int, int>> associations; // current feature, reference feature
};

inline bool estimate_reference_pose(
    const cv::Mat &reference_descriptors, const std::vector<cv::Point3f> &reference_points,
    const cv::Mat &descriptors, const std::vector<cv::Point2f> &pixels,
    const cv::Mat &intrinsics, PoseSeed &result) {
    result = {};
    if (descriptors.rows < 30 || reference_descriptors.rows < 30 ||
        descriptors.rows != static_cast<int>(pixels.size()) ||
        reference_descriptors.rows != static_cast<int>(reference_points.size())) {
        return false;
    }
    std::vector<std::vector<cv::DMatch>> neighbors;
    cv::BFMatcher(cv::NORM_HAMMING).knnMatch(descriptors, reference_descriptors, neighbors, 2);
    std::vector<cv::DMatch> candidates;
    for (const auto &pair : neighbors) {
        if (pair.size() == 2 && pair[0].distance < 64 &&
            pair[0].distance < 0.7f * pair[1].distance) {
            candidates.push_back(pair[0]);
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
        return a.distance < b.distance;
    });
    std::set<int> used;
    std::vector<cv::Point3f> world;
    std::vector<cv::Point2f> image;
    std::vector<std::pair<int, int>> indices;
    for (const auto &match : candidates) {
        const auto &point = reference_points[match.trainIdx];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !used.insert(match.trainIdx).second) {
            continue;
        }
        world.push_back(point);
        image.push_back(pixels[match.queryIdx]);
        indices.emplace_back(match.queryIdx, match.trainIdx);
    }
    if (world.size() < 30) {
        return false;
    }
    cv::Mat rotation, translation, inliers;
    if (!cv::solvePnPRansac(world, image, intrinsics, cv::noArray(), rotation, translation,
                           false, 150, 2.0, 0.995, inliers, cv::SOLVEPNP_EPNP) ||
        inliers.rows < 30 || inliers.rows * 2 < static_cast<int>(world.size())) {
        return false;
    }
    std::vector<cv::Point3f> accepted_world;
    std::vector<cv::Point2f> accepted_image;
    for (int row = 0; row < inliers.rows; ++row) {
        const int index = inliers.at<int>(row);
        accepted_world.push_back(world[index]);
        accepted_image.push_back(image[index]);
        result.associations.push_back(indices[index]);
    }
    cv::solvePnPRefineLM(accepted_world, accepted_image, intrinsics, cv::noArray(),
                         rotation, translation);
    cv::Mat matrix;
    cv::Rodrigues(rotation, matrix);
    if (!cv::checkRange(matrix) || !cv::checkRange(translation)) {
        result = {};
        return false;
    }
    std::vector<cv::Point2f> projected;
    cv::projectPoints(accepted_world, rotation, translation, intrinsics, cv::noArray(), projected);
    for (size_t index = 0; index < accepted_world.size(); ++index) {
        const auto &point = accepted_world[index];
        const double depth = matrix.at<double>(2,0) * point.x + matrix.at<double>(2,1) * point.y
            + matrix.at<double>(2,2) * point.z + translation.at<double>(2);
        if (depth <= 0 || cv::norm(projected[index] - accepted_image[index]) > 3) {
            result = {};
            return false;
        }
    }
    result.camera_from_map = cv::Mat::eye(4, 4, CV_32F);
    matrix.convertTo(result.camera_from_map(cv::Rect(0, 0, 3, 3)), CV_32F);
    translation.convertTo(result.camera_from_map(cv::Rect(3, 0, 1, 3)), CV_32F);
    return true;
}
} // namespace observation

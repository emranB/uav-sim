/**
 * Check deterministic pose geometry without simulator or vehicle transports.
 */
#include "reference_pose.h"
#include <iostream>
#include <stdexcept>

int main() {
    cv::setNumThreads(1);
    cv::setRNGSeed(27);
    cv::Mat descriptors(200, 32, CV_8U);
    cv::randu(descriptors, 0, 256);
    cv::Mat changed_descriptors = descriptors.clone();
    for (int row = 0; row < changed_descriptors.rows; ++row) {
        for (int column = 0; column < 13; ++column) {
            changed_descriptors.at<unsigned char>(row, column) ^= 0x0f;
        }
    }
    std::vector<cv::Point3f> world;
    cv::RNG random(27);
    for (int index = 0; index < 200; ++index) {
        world.emplace_back(random.uniform(-2.f, 2.f), random.uniform(-2.f, 2.f),
                           random.uniform(5.f, 15.f));
    }
    const cv::Mat intrinsics = (cv::Mat_<double>(3,3) << 380,0,320,0,380,240,0,0,1);
    const cv::Mat rotation = (cv::Mat_<double>(3,1) << 0,0,1.2);
    const cv::Mat translation = (cv::Mat_<double>(3,1) << 0.5,-0.2,0.3);
    std::vector<cv::Point2f> pixels;
    cv::projectPoints(world, rotation, translation, intrinsics, cv::noArray(), pixels);
    for (size_t index = 0; index < pixels.size(); index += 5) {
        pixels[index] += cv::Point2f(150, -100);
    }
    observation::PoseSeed seed;
    if (!observation::estimate_reference_pose(descriptors, world, changed_descriptors, pixels,
                                              intrinsics, seed) || seed.associations.size() < 150) {
        throw std::runtime_error("Rotated reference pose with outliers was not recovered");
    }
    cv::Mat recovered;
    seed.camera_from_map(cv::Rect(3,0,1,3)).convertTo(recovered, CV_64F);
    if (cv::norm(recovered, translation, cv::NORM_L2) > 0.001) {
        throw std::runtime_error("Recovered translation exceeds its measured tolerance");
    }
    std::vector<cv::Point3f> sparse_world(world.begin(), world.begin() + 35);
    std::vector<cv::Point2f> sparse_pixels;
    cv::projectPoints(sparse_world, rotation, translation, intrinsics, cv::noArray(), sparse_pixels);
    if (!observation::estimate_reference_pose(descriptors.rowRange(0,35), sparse_world,
                                              descriptors.rowRange(0,35), sparse_pixels,
                                              intrinsics, seed)) {
        throw std::runtime_error("Thirty-five verified correspondences were unnecessarily rejected");
    }
    sparse_world.resize(29);
    sparse_pixels.resize(29);
    if (observation::estimate_reference_pose(descriptors.rowRange(0,29), sparse_world,
                                             descriptors.rowRange(0,29), sparse_pixels,
                                             intrinsics, seed)) {
        throw std::runtime_error("Pose with fewer than thirty correspondences was accepted");
    }
    for (auto &pixel : pixels) {
        pixel = {random.uniform(0.f, 640.f), random.uniform(0.f, 480.f)};
    }
    if (observation::estimate_reference_pose(descriptors, world, descriptors, pixels,
                                            intrinsics, seed)) {
        throw std::runtime_error("Unrelated image geometry was incorrectly accepted");
    }
    std::cout << "Reference pose rotation/outlier and rejection tests passed\n";
}

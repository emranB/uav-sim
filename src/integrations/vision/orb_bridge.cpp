/**
 * Run the read-only stereo estimator without linking vehicle-command transports.
 */
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include "MapPoint.h"
#include "Map.h"
#include "System.h"

namespace {
constexpr uint32_t kMaximumImageBytes = 1024 * 1024;
constexpr uint32_t kMaximumSnapshotBytes = 3 * 1024 * 1024;

uint64_t decode_little_endian(const uint8_t *bytes, size_t length) {
    uint64_t value = 0;
    for (size_t index = 0; index < length; ++index) {
        value |= static_cast<uint64_t>(bytes[index]) << (8 * index);
    }
    return value;
}

void write_all(int fd, const void *buffer, size_t length) {
    const auto *position = static_cast<const uint8_t *>(buffer);
    while (length > 0) {
        const ssize_t count = write(fd, position, length);
        if (count < 0 && errno == EINTR) {
            continue;
        }
        if (count <= 0) {
            throw std::runtime_error("Cannot write pose pipe response");
        }
        position += count;
        length -= count;
    }
}

cv::Mat read_image(uint32_t length) {
    if (length == 0 || length > kMaximumImageBytes) {
        throw std::runtime_error("Image length exceeds the pose pipe limit");
    }
    std::vector<uint8_t> encoded(length);
    if (!std::cin.read(reinterpret_cast<char *>(encoded.data()), length)) {
        throw std::runtime_error("Truncated pose pipe image");
    }
    return cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
}

struct PoseOutput {
    int fd = -1;
    ~PoseOutput() {
        if (fd >= 0) {
            close(fd);
        }
    }

    void redirect_diagnostics() {
        fd = dup(STDOUT_FILENO);
        if (fd < 0 || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
            throw std::runtime_error("Cannot isolate pose pipe diagnostics");
        }
    }

    void send(const nlohmann::json &pose) const {
        const auto encoded = pose.dump();
        if (encoded.empty() || encoded.size() > kMaximumSnapshotBytes) {
            throw std::runtime_error("Pose pipe response exceeds its byte budget");
        }
        std::array<uint8_t, 4> header{};
        for (size_t index = 0; index < header.size(); ++index) {
            header[index] = (encoded.size() >> (8 * index)) & 0xff;
        }
        write_all(fd, header.data(), header.size());
        write_all(fd, encoded.data(), encoded.size());
    }
};
}  // namespace

int main(int argc, char **argv) {
    if (argc != 5 && argc != 6) {
        std::cerr << "Expected vocabulary, calibration, frame manifest and output directory\n";
        return 2;
    }
    try {
        cv::setNumThreads(1);
        const bool live = argc == 6 && std::string(argv[5]) == "--live";
        const bool pipe = argc == 6 && std::string(argv[5]) == "--pipe";
        if (argc == 6 && !live && !pipe) {
            throw std::runtime_error("Unknown native stereo transport");
        }
        PoseOutput pose_output;
        if (pipe) {
            pose_output.redirect_diagnostics();
        }
        std::ifstream input(argv[3]);
        const auto manifest = live || pipe ? nlohmann::json() : nlohmann::json::parse(input);
        ORB_SLAM3::System mapping(argv[1], argv[2], ORB_SLAM3::System::STEREO, false);
        std::ofstream observations;
        if (!pipe) {
            observations.open(std::string(argv[4]) + "/observations.jsonl");
            if (!observations) {
                throw std::runtime_error("Cannot open mapping observation output");
            }
        }
        std::deque<nlohmann::json> trajectory;
        long active_map_id = -1;
        unsigned long map_revision = 0;
        long frame_map_id = -1;
        long frame_origin_id = -1;
        unsigned long map_generation = 0;
        auto process_frame = [&](const cv::Mat &left, const cv::Mat &right, double timestamp) {
            if (left.empty() || right.empty()) {
                throw std::runtime_error("Cannot decode recorded stereo pair");
            }
            const auto camera_from_map = mapping.TrackStereo(left, right, timestamp);
            const auto map_from_camera = camera_from_map.inverse();
            const auto position = map_from_camera.translation();
            const int state = mapping.GetTrackingState();
            if (state != 2) {
                trajectory.clear();
            }
            ORB_SLAM3::Map *active_map = nullptr;
            for (auto *point : mapping.GetTrackedMapPoints()) {
                if (state == 2 && point != nullptr && !point->isBad()) {
                    active_map = point->GetMap();
                    break;
                }
            }
            const long current_map_id = active_map == nullptr ? -1 : active_map->GetId();
            const auto *origin = active_map == nullptr ? nullptr : active_map->GetOriginKF();
            const long current_origin_id = origin == nullptr ? -1 : origin->mnId;
            // ResetActiveMap can reuse a map ID while replacing its coordinate origin.
            if (state == 2 && current_map_id >= 0 && current_origin_id >= 0) {
                if (frame_map_id >= 0 && (current_map_id != frame_map_id ||
                                         current_origin_id != frame_origin_id)) {
                    ++map_generation;
                    trajectory.clear();
                }
                frame_map_id = current_map_id;
                frame_origin_id = current_origin_id;
            }
            auto pose = nlohmann::json({
                {"timestamp_seconds", timestamp}, {"tracking_state", state},
                {"position_map_m", {position.x(), position.y(), position.z()}},
                {"position_valid", state == 2}, {"map_id", current_map_id},
                {"map_origin_id", current_origin_id}, {"map_generation", map_generation}
            });
            if (pipe) {
                const auto rotation = map_from_camera.unit_quaternion();
                pose["orientation_xyzw"] = {rotation.x(), rotation.y(), rotation.z(), rotation.w()};
                pose_output.send(pose);
                return;
            }
            observations << pose.dump() << '\n';
            observations.flush();
            if (live) {
                if (mapping.MapChanged() || current_map_id != active_map_id) {
                    ++map_revision;
                    trajectory.clear();
                }
                active_map_id = current_map_id;
                if (state == 2) {
                    trajectory.push_back({position.x(), position.y(), position.z()});
                    if (trajectory.size() > 10000) {
                        trajectory.pop_front();
                    }
                }
                nlohmann::json points = nlohmann::json::array();
                const auto mapped_points = active_map == nullptr
                    ? std::vector<ORB_SLAM3::MapPoint*>() : active_map->GetAllMapPoints();
                // Export the current map, not just landmarks visible in this frame.
                const size_t stride = std::max<size_t>(1, (mapped_points.size() + 19999) / 20000);
                for (size_t index = 0; index < mapped_points.size(); index += stride) {
                    auto *point = mapped_points[index];
                    if (point != nullptr && !point->isBad() && points.size() < 20000) {
                        const auto landmark = point->GetWorldPos();
                        points.push_back({landmark.x(), landmark.y(), landmark.z()});
                    }
                }
                nlohmann::json path = nlohmann::json::array();
                for (const auto &point : trajectory) {
                    path.push_back(point);
                }
                const std::string destination = std::string(argv[4]) + "/snapshot.json";
                const auto rotation = map_from_camera.unit_quaternion();
                {
                    std::ofstream snapshot(destination + ".tmp");
                    snapshot << nlohmann::json({
                        {"timestamp_seconds", timestamp}, {"tracking_state", state},
                        {"position_valid", state == 2}, {"trajectory", path},
                        {"map_id", current_map_id}, {"map_revision", map_revision},
                        {"map_origin_id", current_origin_id}, {"map_generation", map_generation},
                        {"position_map_m", {position.x(), position.y(), position.z()}},
                        {"orientation_xyzw", {rotation.x(), rotation.y(), rotation.z(), rotation.w()}},
                        {"landmarks", points}
                    }).dump();
                    if (!snapshot) {
                        throw std::runtime_error("Cannot write live map snapshot");
                    }
                }
                if (std::rename((destination + ".tmp").c_str(), destination.c_str()) != 0) {
                    throw std::runtime_error("Cannot publish live map snapshot");
                }
            }
        };
        const auto process_file = [&](const nlohmann::json &frame) {
            const auto left = cv::imread(frame.at("left").get<std::string>(), cv::IMREAD_GRAYSCALE);
            const auto right = cv::imread(frame.at("right").get<std::string>(), cv::IMREAD_GRAYSCALE);
            process_frame(left, right, frame.at("timestamp_seconds").get<double>());
        };
        if (pipe) {
            uint64_t previous_stamp = 0;
            bool received_first = false;
            for (;;) {
                std::array<uint8_t, 16> header{};
                if (!std::cin.read(reinterpret_cast<char *>(header.data()), header.size())) {
                    if (std::cin.eof() && std::cin.gcount() == 0) {
                        break;
                    }
                    throw std::runtime_error("Truncated pose pipe header");
                }
                const auto stamp = decode_little_endian(header.data(), 8);
                if (received_first && stamp <= previous_stamp) {
                    throw std::runtime_error("Pose pipe capture timestamps must increase");
                }
                const auto left = read_image(decode_little_endian(header.data() + 8, 4));
                const auto right = read_image(decode_little_endian(header.data() + 12, 4));
                process_frame(left, right, static_cast<double>(stamp) / 1e9);
                previous_stamp = stamp;
                received_first = true;
            }
        } else if (live) {
            std::string line;
            while (std::getline(std::cin, line)) {
                process_file(nlohmann::json::parse(line));
            }
        } else {
            for (const auto &frame : manifest.at("frames")) {
                process_file(frame);
            }
        }
        mapping.Shutdown();
        if (pipe) {
            return 0;
        }
        mapping.SaveTrajectoryTUM(std::string(argv[4]) + "/trajectory.tum");
        mapping.SaveKeyFrameTrajectoryTUM(std::string(argv[4]) + "/keyframes.tum");
        nlohmann::json landmarks = nlohmann::json::array();
        for (auto *point : mapping.GetTrackedMapPoints()) {
            if (point != nullptr && !point->isBad()) {
                const auto position = point->GetWorldPos();
                landmarks.push_back({position.x(), position.y(), position.z()});
            }
        }
        std::ofstream output(std::string(argv[4]) + "/landmarks.json");
        output << nlohmann::json({
            {"schema_version", 1}, {"frame", "ORB-SLAM local map; metres"},
            {"scope", "last tracked landmark set after shutdown"}, {"points", landmarks}
        }).dump(2) << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Recorded mapping failed: " << error.what() << '\n';
        return 1;
    }
}

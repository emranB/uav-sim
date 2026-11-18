/**
 * Declare the adapter that seeds reference frames for observational tracking.
 */
#pragma once
namespace ORB_SLAM3 {
class Frame;
class KeyFrame;
class Map;
bool seed_reference_frame(Frame &frame, const Frame &previous, KeyFrame *reference, Map *active_map);
}

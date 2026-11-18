"""Apply the small observational pose-seeding extension to the pinned upstream revision."""

from pathlib import Path


def main() -> None:
    root = Path("/opt/orb-slam3")
    revision = "0df83dde1c85c7ab91a0d47de7a29685d046f637"
    if (root / "revision.txt").read_text().strip() != revision:
        raise RuntimeError("Review the tracking extension before changing upstream revisions")
    path = root / "src/Tracking.cc"
    source = path.read_text()
    anchor = """            if(bOK)
            {
                bOK = TrackLocalMap();

            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;"""
    replacement = """            if(bOK)
            {
                bOK = TrackLocalMap();
            }
            // Image-only reseeding preserves the normal local-map inlier checks.
            if(!bOK && mSensor == System::STEREO &&
               seed_reference_frame(mCurrentFrame, mLastFrame, mpReferenceKF, pCurrentMap))
            {
                bOK = TrackLocalMap();
                if(bOK)
                    cout << "Reference image pose seed accepted" << endl;
            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;"""
    if source.count(anchor) != 1:
        raise RuntimeError("Upstream tracking hook changed; refusing an ambiguous patch")
    source = source.replace(anchor, replacement)
    initialization_anchor = "void Tracking::StereoInitialization()\n{\n"
    if source.count(initialization_anchor) != 1:
        raise RuntimeError("Upstream stereo initialization hook changed")
    initialization_guard = """    if (mSensor == System::STEREO && !mpCamera2 &&
        !observation::has_stereo_initialization_support(mCurrentFrame.mvDepth))
    {
        return;
    }
"""
    source = source.replace(initialization_anchor, initialization_anchor + initialization_guard)
    path.write_text(
        '#include "stereo_initialization.h"\n#include "reference_pose_adapter.h"\n' + source
    )
    with (root / "CMakeLists.txt").open("a") as stream:
        stream.write(
            "\ntarget_sources(ORB_SLAM3 PRIVATE /opt/vision-extension/reference_pose_adapter.cpp)\n"
            "target_include_directories(ORB_SLAM3 PRIVATE /opt/vision-extension)\n"
        )


if __name__ == "__main__":
    main()

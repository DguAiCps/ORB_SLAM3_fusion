/**
 * RealSense D435i RGB-D Dataset Collection Example
 *
 * This example demonstrates real-time dataset collection of triangulated vs RGB-D
 * depth measurements using Intel RealSense D435i camera.
 *
 * Usage: ./rgbd_realsense_dataset_collection Vocabulary/ORBvoc.txt Examples/RGB-D/RealSense_D435i.yaml
 *
 * Features:
 * - Real-time RGB-D SLAM with RealSense D435i
 * - Automatic dataset collection at every Local Bundle Adjustment
 * - Triangulated vs RGB-D depth comparison
 * - JSON output for research analysis
 */

#include <System.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <fstream>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <sstream>

#include "librealsense2/rsutil.h"
#include "DatasetCollector.h"

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current,
                     const std::vector<rs2::stream_profile>& prev);

int main(int argc, char **argv) try {
    if (argc < 3) {
        cerr << endl
             << "Usage: ./rgbd_realsense_dataset_collection path_to_vocabulary path_to_settings"
             << endl;
        return 1;
    }

    // Create output directory for dataset collection
    mkdir("./realsense_dataset_output", 0755);

    cout << "ORB-SLAM3 RealSense D435i Dataset Collection" << endl;
    cout << "===========================================" << endl;
    cout << "Features:" << endl;
    cout << "- Real-time RGB-D SLAM" << endl;
    cout << "- Automatic triangulation vs RGB-D dataset collection" << endl;
    cout << "- Output directory: ./realsense_dataset_output/" << endl;
    cout << "- Press Ctrl+C to stop and save data" << endl;
    cout << "===========================================" << endl << endl;

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    // Configure dataset collector
    ORB_SLAM3::DatasetCollector& collector = ORB_SLAM3::DatasetCollector::GetInstance();
    collector.Initialize("./realsense_dataset_output", true);

    // Optional: Customize thresholds for RealSense D435i
    collector.SetDepthThresholds(0.3f, 10.0f);  // D435i range: 0.3m - 10m
    collector.SetTriangulationThreshold(1.0f);   // 1 degree minimum angle

    cout << "Starting RealSense pipeline..." << endl;

    // RealSense pipeline setup
    rs2::pipeline pipe;
    rs2::config cfg;

    // Configure streams
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Start pipeline
    rs2::pipeline_profile selection = pipe.start(cfg);
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    // Get camera intrinsics
    auto depth_intrin = depth_stream.get_intrinsics();
    auto color_intrin = color_stream.get_intrinsics();

    cout << "Camera intrinsics:" << endl;
    cout << "Depth: " << depth_intrin.width << "x" << depth_intrin.height
         << " fx=" << depth_intrin.fx << " fy=" << depth_intrin.fy << endl;
    cout << "Color: " << color_intrin.width << "x" << color_intrin.height
         << " fx=" << color_intrin.fx << " fy=" << color_intrin.fy << endl;

    // Set up dataset collector with actual camera parameters
    collector.SetCameraParameters(depth_intrin.fx, depth_intrin.fy,
                                 depth_intrin.ppx, depth_intrin.ppy);

    // Align depth to color
    rs2::align align_to_color(RS2_STREAM_COLOR);

    signal(SIGINT, exit_loop_handler);
    b_continue_session = true;

    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    cout << "\nStarting SLAM with dataset collection..." << endl;
    cout << "Press Ctrl+C to stop and finalize dataset" << endl << endl;

    while (b_continue_session) {
        // Wait for frames
        rs2::frameset frameset = pipe.wait_for_frames();

        // Align frames
        auto aligned_frames = align_to_color.process(frameset);
        auto color_frame = aligned_frames.get_color_frame();
        auto depth_frame = aligned_frames.get_depth_frame();

        if (!color_frame || !depth_frame) continue;

        // Convert to OpenCV format
        cv::Mat color_image(cv::Size(color_intrin.width, color_intrin.height),
                           CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth_image(cv::Size(depth_intrin.width, depth_intrin.height),
                           CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // Get timestamp
        double timestamp = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();

        frame_count++;

        // Track with SLAM (dataset collection happens automatically)
        SLAM.TrackRGBD(color_image, depth_image, timestamp);

        // Progress indicator
        if (frame_count % 30 == 0) {
            cout << "Processed " << frame_count << " frames, Time: "
                 << std::fixed << std::setprecision(2) << timestamp << "s" << endl;
        }
    }

    cout << "\nStopping SLAM system..." << endl;
    SLAM.Shutdown();

    // Dataset collection statistics
    auto end_time = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration<double>(end_time - start_time).count();

    cout << "\n===========================================" << endl;
    cout << "Dataset Collection Complete!" << endl;
    cout << "===========================================" << endl;
    cout << "Total frames processed: " << frame_count << endl;
    cout << "Total time: " << std::fixed << std::setprecision(2) << total_time << "s" << endl;
    cout << "Average FPS: " << std::fixed << std::setprecision(1)
         << (frame_count / total_time) << endl;
    cout << "Output directory: ./realsense_dataset_output/" << endl;
    cout << "Check for JSON files containing triangulation vs RGB-D data" << endl;

    // Save trajectory
    SLAM.SaveTrajectoryTUM("RealSenseTrajectory.txt");
    cout << "Trajectory saved to: RealSenseTrajectory.txt" << endl;

    return 0;

} catch (const rs2::error & e) {
    cerr << "RealSense error calling " << e.get_failed_function() << "("
         << e.get_failed_args() << "):\n    " << e.what() << endl;
    return EXIT_FAILURE;
} catch (const exception & e) {
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams) {
    // Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    // We prioritize color streams to make the view look better for the user.
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams) {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream == RS2_STREAM_DEPTH) {
            depth_stream_found = true;
        } else if (profile_stream == RS2_STREAM_COLOR) {
            color_stream_found = true;
        }
    }

    if (depth_stream_found && color_stream_found) {
        align_to = RS2_STREAM_COLOR;
    } else if (depth_stream_found) {
        align_to = RS2_STREAM_INFRARED;
    }

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current,
                     const std::vector<rs2::stream_profile>& prev) {
    for (auto&& sp : current) {
        // If previous profile is empty
        if (prev.empty()) return true;

        auto itr = std::find_if(std::begin(prev), std::end(prev),
                               [&sp](const rs2::stream_profile& current_sp) {
                                   return sp.unique_id() == current_sp.unique_id();
                               });
        if (itr == prev.end()) {
            return true;
        }
    }
    return false;
}
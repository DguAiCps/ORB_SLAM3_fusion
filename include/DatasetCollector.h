/**
* This file is part of ORB-SLAM3
* Dataset Collection for Triangulation vs RGB-D Analysis
*/

#ifndef DATASETCOLLECTOR_H
#define DATASETCOLLECTOR_H

#include <vector>
#include <string>
#include <fstream>
#include <mutex>
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <json/json.h>

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Atlas.h"
#include "GeometricCamera.h"
#include "GeometricTools.h"

namespace ORB_SLAM3 {

struct PointData {
    int point_id;
    Eigen::Vector3f triangulated_xyz;
    Eigen::Vector3f rgbd_xyz;
    float triangulation_angle_degrees;
    Eigen::Vector3f optimized_xyz;
    int num_observations;
    bool is_observable;
    bool is_valid;
};

struct FrameData {
    int frame_id;
    double timestamp;
    Sophus::SE3f frame_pose;
    bool is_keyframe;
    int ba_iteration;
    std::vector<PointData> points;
};

class DatasetCollector {
public:
    static DatasetCollector& GetInstance();

    void Initialize(const std::string& output_dir, bool enabled = true);
    void SetCameraParameters(float fx, float fy, float cx, float cy);
    void SetDepthThresholds(float min_depth, float max_depth);
    void SetTriangulationThreshold(float min_angle_degrees);

    // Main data collection interfaces
    void CollectPreBAData(KeyFrame* pCurrentKF, KeyFrame* pLastKF, const std::vector<MapPoint*>& vpLocalMapPoints);
    void CollectPostBAData(KeyFrame* pCurrentKF, const std::vector<MapPoint*>& vpLocalMapPoints);
    void CollectTrackingFrameData(Frame* pCurrentFrame, KeyFrame* pLastKF);

    // Core functionality
    bool TriangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                         const Sophus::SE3f& Tcw1, const Sophus::SE3f& Tcw2,
                         GeometricCamera* pCamera1, GeometricCamera* pCamera2,
                         Eigen::Vector3f& x3D, float& angle_degrees);

    bool VerifyObservability(MapPoint* pMP, KeyFrame* pFrame);
    bool VerifyObservability(MapPoint* pMP, Frame* pFrame);
    bool CheckDataValidity(MapPoint* pMP, float rgbdDepth, float triAngle);
    float CalculateTriangulationAngle(const Eigen::Vector3f& ray1, const Eigen::Vector3f& ray2);

    // Output functions
    void SaveFrameData(const FrameData& frameData);
    void SaveMetadata();
    void Finalize();

    // Status check
    bool IsEnabled() const { return mbEnabled; }

    // Thread safety
    void Lock() { mMutex.lock(); }
    void Unlock() { mMutex.unlock(); }

private:
    DatasetCollector() = default;
    ~DatasetCollector() = default;
    DatasetCollector(const DatasetCollector&) = delete;
    DatasetCollector& operator=(const DatasetCollector&) = delete;

    // Configuration
    bool mbEnabled;
    std::string mOutputDir;

    // Camera parameters
    float mfx, mfy, mcx, mcy;

    // Thresholds
    float mMinDepth, mMaxDepth;
    float mMinTriangulationAngle;

    // BA iteration counter
    int mBAIteration;

    // Thread safety
    std::mutex mMutex;

    // Helper functions
    std::string GenerateFilename(int frame_id, int ba_iteration);
    Json::Value PointDataToJson(const PointData& point);
    Json::Value FrameDataToJson(const FrameData& frame);

    // Constants
    static constexpr float MIN_DEPTH_DEFAULT = 0.3f;
    static constexpr float MAX_DEPTH_DEFAULT = 10.0f;
    static constexpr float MIN_TRIANGULATION_ANGLE_DEFAULT = 1.0f; // degrees
    static constexpr float ERROR_THRESHOLD = 0.1f; // 10cm
};

} // namespace ORB_SLAM3

#endif // DATASETCOLLECTOR_H
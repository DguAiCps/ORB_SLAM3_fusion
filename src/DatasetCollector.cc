/**
* This file is part of ORB-SLAM3
* Dataset Collection for Triangulation vs RGB-D Analysis
*/

#include "DatasetCollector.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <sys/stat.h>
#include <sys/types.h>

namespace ORB_SLAM3 {

DatasetCollector& DatasetCollector::GetInstance() {
    static DatasetCollector instance;
    return instance;
}

void DatasetCollector::Initialize(const std::string& output_dir, bool enabled) {
    std::lock_guard<std::mutex> lock(mMutex);

    mbEnabled = enabled;
    if (!mbEnabled) return;

    mOutputDir = output_dir;
    mBAIteration = 0;

    // Create output directory if it doesn't exist
    mkdir(mOutputDir.c_str(), 0755);

    // Initialize with default thresholds
    mMinDepth = MIN_DEPTH_DEFAULT;
    mMaxDepth = MAX_DEPTH_DEFAULT;
    mMinTriangulationAngle = MIN_TRIANGULATION_ANGLE_DEFAULT;

    std::cout << "[DatasetCollector] Initialized - Output: " << mOutputDir << std::endl;
}

void DatasetCollector::SetCameraParameters(float fx, float fy, float cx, float cy) {
    std::lock_guard<std::mutex> lock(mMutex);
    mfx = fx; mfy = fy; mcx = cx; mcy = cy;
}

void DatasetCollector::SetDepthThresholds(float min_depth, float max_depth) {
    std::lock_guard<std::mutex> lock(mMutex);
    mMinDepth = min_depth;
    mMaxDepth = max_depth;
}

void DatasetCollector::SetTriangulationThreshold(float min_angle_degrees) {
    std::lock_guard<std::mutex> lock(mMutex);
    mMinTriangulationAngle = min_angle_degrees;
}

void DatasetCollector::CollectPreBAData(KeyFrame* pCurrentKF, KeyFrame* pLastKF, const std::vector<MapPoint*>& vpLocalMapPoints) {
    if (!mbEnabled) return;

    std::lock_guard<std::mutex> lock(mMutex);

    // Increment BA iteration counter
    mBAIteration++;

    FrameData frameData;
    frameData.frame_id = pCurrentKF->mnId;
    frameData.timestamp = pCurrentKF->mTimeStamp;
    frameData.frame_pose = pCurrentKF->GetPose();
    frameData.is_keyframe = true;
    frameData.ba_iteration = mBAIteration;

    // Process points visible in current keyframe
    const std::vector<MapPoint*> vpMapPointMatches = pCurrentKF->GetMapPointMatches();

    for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
        MapPoint* pMP = vpMapPointMatches[i];
        if (!pMP || pMP->isBad()) continue;

        // Skip if not in local map points
        if (std::find(vpLocalMapPoints.begin(), vpLocalMapPoints.end(), pMP) == vpLocalMapPoints.end()) {
            continue;
        }

        PointData pointData;
        pointData.point_id = pMP->mnId;
        pointData.num_observations = pMP->Observations();

        // Get RGB-D depth
        float rgbd_depth = pCurrentKF->mvDepth[i];
        if (std::isnan(rgbd_depth) || rgbd_depth <= 0) continue;

        // Get RGB-D 3D point
        Eigen::Vector3f x3D_rgbd;
        if (!pCurrentKF->UnprojectStereo(i, x3D_rgbd)) continue;
        pointData.rgbd_xyz = x3D_rgbd;

        // Use triangulation results from existing Tracking.cc implementation (lines 2078-2158)
        if (i < pCurrentKF->mvTriangulatedPoints.size() &&
            i < pCurrentKF->mvTriangulationValid.size() &&
            pCurrentKF->mvTriangulationValid[i]) {
            // Use triangulated value from Tracking thread
            pointData.triangulated_xyz = pCurrentKF->mvTriangulatedPoints[i];

            // Calculate triangulation angle from the two points
            Eigen::Vector3f ray1 = (pointData.triangulated_xyz - pCurrentKF->GetCameraCenter()).normalized();
            if (pLastKF) {
                Eigen::Vector3f ray2 = (pointData.triangulated_xyz - pLastKF->GetCameraCenter()).normalized();
                pointData.triangulation_angle_degrees = CalculateTriangulationAngle(ray1, ray2);
            } else {
                pointData.triangulation_angle_degrees = 0.0f;
            }

            pointData.is_valid = CheckDataValidity(pMP, rgbd_depth, pointData.triangulation_angle_degrees);
        } else {
            // No valid triangulation available, use current MapPoint position
            pointData.triangulated_xyz = pMP->GetWorldPos();
            pointData.triangulation_angle_degrees = 0.0f;
            pointData.is_valid = CheckDataValidity(pMP, rgbd_depth, 0.0f);
        }

        // Store pre-BA optimized position (current MapPoint position)
        pointData.optimized_xyz = pMP->GetWorldPos();
        pointData.is_observable = VerifyObservability(pMP, pCurrentKF);

        frameData.points.push_back(pointData);
    }

    // Save pre-BA data
    SaveFrameData(frameData);

    std::cout << "[DatasetCollector] Pre-BA: Frame " << frameData.frame_id
              << " BA iteration " << mBAIteration
              << " Points: " << frameData.points.size() << std::endl;
}

void DatasetCollector::CollectPostBAData(KeyFrame* pCurrentKF, const std::vector<MapPoint*>& vpLocalMapPoints) {
    if (!mbEnabled) return;

    std::lock_guard<std::mutex> lock(mMutex);

    FrameData frameData;
    frameData.frame_id = pCurrentKF->mnId;
    frameData.timestamp = pCurrentKF->mTimeStamp;
    frameData.frame_pose = pCurrentKF->GetPose();
    frameData.is_keyframe = true;
    frameData.ba_iteration = mBAIteration; // Use same iteration number

    // Process points visible in current keyframe after BA
    const std::vector<MapPoint*> vpMapPointMatches = pCurrentKF->GetMapPointMatches();

    for (size_t i = 0; i < vpMapPointMatches.size(); i++) {
        MapPoint* pMP = vpMapPointMatches[i];
        if (!pMP || pMP->isBad()) continue;

        // Skip if not in local map points
        if (std::find(vpLocalMapPoints.begin(), vpLocalMapPoints.end(), pMP) == vpLocalMapPoints.end()) {
            continue;
        }

        PointData pointData;
        pointData.point_id = pMP->mnId;
        pointData.num_observations = pMP->Observations();

        // Get RGB-D depth (unchanged by BA)
        float rgbd_depth = pCurrentKF->mvDepth[i];
        if (std::isnan(rgbd_depth) || rgbd_depth <= 0) continue;

        // Get RGB-D 3D point (unchanged by BA)
        Eigen::Vector3f x3D_rgbd;
        if (!pCurrentKF->UnprojectStereo(i, x3D_rgbd)) continue;
        pointData.rgbd_xyz = x3D_rgbd;

        // Triangulated position (same as pre-BA, since it's geometric)
        pointData.triangulated_xyz = x3D_rgbd; // Placeholder - should match pre-BA
        pointData.triangulation_angle_degrees = 0.0f; // Placeholder

        // Post-BA optimized position (updated MapPoint position)
        pointData.optimized_xyz = pMP->GetWorldPos();
        pointData.is_observable = VerifyObservability(pMP, pCurrentKF);
        pointData.is_valid = CheckDataValidity(pMP, rgbd_depth, 0.0f);

        frameData.points.push_back(pointData);
    }

    // Save post-BA data with suffix
    std::string filename = GenerateFilename(frameData.frame_id, frameData.ba_iteration) + "_postBA.json";
    std::string filepath = mOutputDir + "/" + filename;

    std::ofstream file(filepath);
    if (file.is_open()) {
        Json::Value root = FrameDataToJson(frameData);
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(root, &file);
        file.close();
        std::cout << "[DatasetCollector] Post-BA JSON saved: " << filepath << std::endl;
    } else {
        std::cerr << "[DatasetCollector] Failed to open Post-BA file: " << filepath << std::endl;
    }

    std::cout << "[DatasetCollector] Post-BA: Frame " << frameData.frame_id
              << " BA iteration " << mBAIteration
              << " Points: " << frameData.points.size() << std::endl;
}

void DatasetCollector::CollectTrackingFrameData(Frame* pCurrentFrame, KeyFrame* pLastKF) {
    if (!mbEnabled || !pCurrentFrame || !pLastKF) return;

    std::lock_guard<std::mutex> lock(mMutex);

    FrameData frameData;
    frameData.frame_id = pCurrentFrame->mnId;
    frameData.timestamp = pCurrentFrame->mTimeStamp;
    frameData.frame_pose = pCurrentFrame->GetPose();
    frameData.is_keyframe = false;  // This is a regular tracking frame
    frameData.ba_iteration = -1;    // Not associated with BA iteration

    // Only collect points that have BOTH valid triangulation AND valid correspondence
    // This ensures we only process points that were actually triangulated between
    // current frame and last keyframe
    for (size_t i = 0; i < pCurrentFrame->mvpMapPoints.size(); i++) {
        MapPoint* pMP = pCurrentFrame->mvpMapPoints[i];
        if (!pMP || pMP->isBad()) continue;

        // CRITICAL CHECK 1: Must have valid triangulation data
        if (i >= pCurrentFrame->mvTriangulationValid.size() ||
            !pCurrentFrame->mvTriangulationValid[i]) continue;

        // CRITICAL CHECK 2: Must have valid triangulated point
        if (i >= pCurrentFrame->mvTriangulatedPoints.size()) continue;

        // CRITICAL CHECK 3: Verify the MapPoint is actually observed in last keyframe
        // This ensures the triangulation was valid
        std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();
        if (observations.find(pLastKF) == observations.end()) {
            // Point not observed in last keyframe - skip triangulation comparison
            continue;
        }

        PointData pointData;
        pointData.point_id = pMP->mnId;
        pointData.num_observations = pMP->Observations();

        // Get RGB-D depth from current frame
        float rgbd_depth = pCurrentFrame->mvDepth[i];
        if (std::isnan(rgbd_depth) || rgbd_depth <= 0) continue;

        // Get RGB-D 3D point from current frame
        Eigen::Vector3f x3D_rgbd;
        if (!pCurrentFrame->UnprojectStereo(i, x3D_rgbd)) continue;
        pointData.rgbd_xyz = x3D_rgbd;

        // Get triangulated point (validated above)
        pointData.triangulated_xyz = pCurrentFrame->mvTriangulatedPoints[i];

        // Calculate actual triangulation angle using stored keypoints
        size_t lastKF_idx = std::get<0>(observations[pLastKF]);
        if (lastKF_idx < pLastKF->mvKeysUn.size() && i < pCurrentFrame->mvKeysUn.size()) {
            cv::KeyPoint kp_curr = pCurrentFrame->mvKeysUn[i];
            cv::KeyPoint kp_last = pLastKF->mvKeysUn[lastKF_idx];

            // Calculate rays for triangulation angle
            Eigen::Vector3f x_curr = pCurrentFrame->mpCamera->unprojectEig(kp_curr.pt);
            Eigen::Vector3f x_last = pLastKF->mpCamera->unprojectEig(kp_last.pt);

            Sophus::SE3f Tcw_curr = pCurrentFrame->GetPose();
            Sophus::SE3f Tcw_last = pLastKF->GetPose();

            Eigen::Matrix3f Rwc_curr = Tcw_curr.rotationMatrix().transpose();
            Eigen::Matrix3f Rwc_last = Tcw_last.rotationMatrix().transpose();

            Eigen::Vector3f ray_curr = Rwc_curr * x_curr;
            Eigen::Vector3f ray_last = Rwc_last * x_last;

            pointData.triangulation_angle_degrees = CalculateTriangulationAngle(ray_curr, ray_last);
        } else {
            pointData.triangulation_angle_degrees = 0.0f;
        }

        // Current MapPoint position (optimized)
        pointData.optimized_xyz = pMP->GetWorldPos();

        // Verify observability in current frame
        pointData.is_observable = VerifyObservability(pMP, pCurrentFrame);

        // Validate data quality
        pointData.is_valid = CheckDataValidity(pMP, rgbd_depth, pointData.triangulation_angle_degrees);

        frameData.points.push_back(pointData);
    }

    // Only save if we have meaningful data
    if (frameData.points.size() > 0) {
        std::string filename = "tracking_frame_" + std::to_string(frameData.frame_id) + ".json";
        std::string filepath = mOutputDir + "/" + filename;

        std::ofstream file(filepath);
        if (file.is_open()) {
            Json::Value root = FrameDataToJson(frameData);
            Json::StreamWriterBuilder builder;
            builder["indentation"] = "  ";
            std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
            writer->write(root, &file);
            file.close();
            std::cout << "[DatasetCollector] Tracking frame JSON saved: " << filepath
                      << " (Points: " << frameData.points.size() << ")" << std::endl;
        } else {
            std::cerr << "[DatasetCollector] Failed to open tracking frame file: " << filepath << std::endl;
        }
    }

    std::cout << "[DatasetCollector] Tracking Frame: " << frameData.frame_id
              << " Valid triangulated points: " << frameData.points.size() << std::endl;
}

bool DatasetCollector::TriangulatePoint(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                       const Sophus::SE3f& Tcw1, const Sophus::SE3f& Tcw2,
                                       GeometricCamera* pCamera1, GeometricCamera* pCamera2,
                                       Eigen::Vector3f& x3D, float& angle_degrees) {

    // Convert keypoints to normalized coordinates
    Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
    Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

    // Calculate transformation matrices
    Eigen::Matrix<float,3,4> eigTcw1 = Tcw1.matrix3x4();
    Eigen::Matrix<float,3,4> eigTcw2 = Tcw2.matrix3x4();

    // Calculate ray directions in world coordinates
    Eigen::Matrix3f Rwc1 = eigTcw1.block<3,3>(0,0).transpose();
    Eigen::Matrix3f Rwc2 = eigTcw2.block<3,3>(0,0).transpose();

    Eigen::Vector3f ray1 = Rwc1 * xn1;
    Eigen::Vector3f ray2 = Rwc2 * xn2;

    // Calculate triangulation angle
    angle_degrees = CalculateTriangulationAngle(ray1, ray2);

    // Perform triangulation using GeometricTools
    bool success = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);

    if (!success) return false;

    // Additional validity checks
    Eigen::Vector3f tcw1 = Tcw1.translation();
    Eigen::Vector3f tcw2 = Tcw2.translation();
    Eigen::Matrix3f Rcw1 = eigTcw1.block<3,3>(0,0);
    Eigen::Matrix3f Rcw2 = eigTcw2.block<3,3>(0,0);

    // Check if point is in front of both cameras
    float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
    float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);

    return (z1 > 0 && z2 > 0 && angle_degrees >= mMinTriangulationAngle);
}

bool DatasetCollector::VerifyObservability(MapPoint* pMP, KeyFrame* pFrame) {
    if (!pMP || !pFrame) return false;

    // Project 3D point to frame
    Eigen::Vector3f x3D = pMP->GetWorldPos();
    Sophus::SE3f Tcw = pFrame->GetPose();

    // Transform to camera coordinates
    Eigen::Vector3f x3Dc = Tcw * x3D;

    // Check if point is in front of camera
    if (x3Dc(2) <= 0) return false;

    // Project to image coordinates
    cv::Point2f uv = pFrame->mpCamera->project(cv::Point3f(x3Dc(0), x3Dc(1), x3Dc(2)));

    // Check if within frame bounds
    if (uv.x < 0 || uv.x >= pFrame->mnMaxX || uv.y < 0 || uv.y >= pFrame->mnMaxY) {
        return false;
    }

    // Check depth range
    float depth = x3Dc(2);
    return (depth >= mMinDepth && depth <= mMaxDepth);
}

bool DatasetCollector::VerifyObservability(MapPoint* pMP, Frame* pFrame) {
    if (!pMP || !pFrame) return false;

    // Project 3D point to frame
    Eigen::Vector3f x3D = pMP->GetWorldPos();
    Sophus::SE3f Tcw = pFrame->GetPose();

    // Transform to camera coordinates
    Eigen::Vector3f x3Dc = Tcw * x3D;

    // Check if point is in front of camera
    if (x3Dc(2) <= 0) return false;

    // Project to image coordinates
    cv::Point2f uv = pFrame->mpCamera->project(cv::Point3f(x3Dc(0), x3Dc(1), x3Dc(2)));

    // Check if within frame bounds
    if (uv.x < 0 || uv.x >= pFrame->mnMaxX || uv.y < 0 || uv.y >= pFrame->mnMaxY) {
        return false;
    }

    // Check depth range
    float depth = x3Dc(2);
    return (depth >= mMinDepth && depth <= mMaxDepth);
}

bool DatasetCollector::CheckDataValidity(MapPoint* pMP, float rgbdDepth, float triAngle) {
    if (!pMP) return false;

    // Triangulation angle check
    if (triAngle > 0 && triAngle < mMinTriangulationAngle) return false;

    // RGB-D depth checks
    if (std::isnan(rgbdDepth) || std::isinf(rgbdDepth)) return false;
    if (rgbdDepth < mMinDepth || rgbdDepth > mMaxDepth) return false;

    // Point quality checks
    if (pMP->Observations() < 2) return false;
    if (pMP->isBad()) return false;

    return true;
}

float DatasetCollector::CalculateTriangulationAngle(const Eigen::Vector3f& ray1, const Eigen::Vector3f& ray2) {
    float cosAngle = ray1.dot(ray2) / (ray1.norm() * ray2.norm());
    cosAngle = std::max(-1.0f, std::min(1.0f, cosAngle)); // Clamp to [-1, 1]
    float angle_rad = std::acos(cosAngle);
    return angle_rad * 180.0f / M_PI; // Convert to degrees
}

void DatasetCollector::SaveFrameData(const FrameData& frameData) {
    if (!mbEnabled) return;

    std::string filename = GenerateFilename(frameData.frame_id, frameData.ba_iteration) + ".json";
    std::string filepath = mOutputDir + "/" + filename;

    std::ofstream file(filepath);
    if (file.is_open()) {
        Json::Value root = FrameDataToJson(frameData);
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(root, &file);
        file.close();
        std::cout << "[DatasetCollector] Pre-BA JSON saved: " << filepath << std::endl;
    } else {
        std::cerr << "[DatasetCollector] Failed to open file: " << filepath << std::endl;
    }
}

void DatasetCollector::SaveMetadata() {
    if (!mbEnabled) return;

    std::string filepath = mOutputDir + "/metadata.json";
    std::ofstream file(filepath);

    if (file.is_open()) {
        Json::Value metadata;
        metadata["description"] = "ORB-SLAM3 Local Bundle Adjustment Dataset";
        metadata["collection_type"] = "Local BA only";
        metadata["camera"]["fx"] = mfx;
        metadata["camera"]["fy"] = mfy;
        metadata["camera"]["cx"] = mcx;
        metadata["camera"]["cy"] = mcy;
        metadata["thresholds"]["min_depth"] = mMinDepth;
        metadata["thresholds"]["max_depth"] = mMaxDepth;
        metadata["thresholds"]["min_triangulation_angle"] = mMinTriangulationAngle;
        metadata["total_ba_iterations"] = mBAIteration;

        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(metadata, &file);
        file.close();
    }
}

void DatasetCollector::Finalize() {
    if (!mbEnabled) return;

    std::lock_guard<std::mutex> lock(mMutex);
    SaveMetadata();
    std::cout << "[DatasetCollector] Finalized - Total BA iterations: " << mBAIteration << std::endl;
}

std::string DatasetCollector::GenerateFilename(int frame_id, int ba_iteration) {
    std::ostringstream oss;
    oss << "frame_" << std::setfill('0') << std::setw(6) << frame_id
        << "_localba" << std::setfill('0') << std::setw(4) << ba_iteration;
    return oss.str();
}

Json::Value DatasetCollector::PointDataToJson(const PointData& point) {
    Json::Value pointJson;
    pointJson["point_id"] = point.point_id;

    pointJson["triangulated_xyz"] = Json::Value(Json::arrayValue);
    pointJson["triangulated_xyz"].append(point.triangulated_xyz(0));
    pointJson["triangulated_xyz"].append(point.triangulated_xyz(1));
    pointJson["triangulated_xyz"].append(point.triangulated_xyz(2));

    pointJson["rgbd_xyz"] = Json::Value(Json::arrayValue);
    pointJson["rgbd_xyz"].append(point.rgbd_xyz(0));
    pointJson["rgbd_xyz"].append(point.rgbd_xyz(1));
    pointJson["rgbd_xyz"].append(point.rgbd_xyz(2));

    pointJson["triangulation_angle_degrees"] = point.triangulation_angle_degrees;

    pointJson["optimized_xyz"] = Json::Value(Json::arrayValue);
    pointJson["optimized_xyz"].append(point.optimized_xyz(0));
    pointJson["optimized_xyz"].append(point.optimized_xyz(1));
    pointJson["optimized_xyz"].append(point.optimized_xyz(2));

    pointJson["num_observations"] = point.num_observations;
    pointJson["is_observable"] = point.is_observable;
    pointJson["is_valid"] = point.is_valid;

    return pointJson;
}

Json::Value DatasetCollector::FrameDataToJson(const FrameData& frame) {
    Json::Value frameJson;
    frameJson["frame_id"] = frame.frame_id;
    frameJson["timestamp"] = frame.timestamp;
    frameJson["is_keyframe"] = frame.is_keyframe;
    frameJson["ba_iteration"] = frame.ba_iteration;

    // Convert SE3 pose to 4x4 matrix
    Eigen::Matrix4f pose_matrix = frame.frame_pose.matrix();
    frameJson["frame_pose"] = Json::Value(Json::arrayValue);
    for (int i = 0; i < 4; i++) {
        Json::Value row(Json::arrayValue);
        for (int j = 0; j < 4; j++) {
            row.append(pose_matrix(i, j));
        }
        frameJson["frame_pose"].append(row);
    }

    frameJson["points"] = Json::Value(Json::arrayValue);
    for (const auto& point : frame.points) {
        frameJson["points"].append(PointDataToJson(point));
    }

    return frameJson;
}

} // namespace ORB_SLAM3
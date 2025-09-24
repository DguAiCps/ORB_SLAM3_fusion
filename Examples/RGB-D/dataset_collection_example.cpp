/**
* Example demonstrating ORB-SLAM3 dataset collection for triangulation vs RGB-D analysis
*
* Usage: ./dataset_collection_example Vocabulary/ORBvoc.txt Examples/RGB-D/RealSense_D435i.yaml path_to_sequence
*
* This example runs RGB-D SLAM with automatic dataset collection enabled.
* Output will be saved to ./dataset_output/ directory.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "DatasetCollector.h"

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./dataset_collection_example path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[3])+"/associations.txt";
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "ORB-SLAM3 Dataset Collection Example" << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "Dataset collection enabled for RGB-D mode" << endl;
    cout << "Output directory: ./dataset_output/" << endl;
    cout << "-------" << endl << endl;

    // Create output directory
    mkdir("./dataset_output", 0755);

    // Configure dataset collector (optional - defaults are usually fine)
    ORB_SLAM3::DatasetCollector& collector = ORB_SLAM3::DatasetCollector::GetInstance();

    // Example customization (uncomment to use):
    // collector.SetDepthThresholds(0.5f, 8.0f);  // 50cm to 8m depth range
    // collector.SetTriangulationThreshold(1.5f); // 1.5 degree minimum triangulation angle

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        // Dataset collection happens automatically during Local Bundle Adjustment
        SLAM.TrackRGBD(imRGB, imD, tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);

        // Progress indicator
        if (ni % 50 == 0) {
            cout << "Processed " << ni << "/" << nImages << " frames" << endl;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }

    cout << "-------" << endl << endl;
    cout << "Dataset Collection Results:" << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    cout << "Check ./dataset_output/ for collected data files" << endl;
    cout << "Metadata saved in ./dataset_output/metadata.json" << endl;

    // Count collected files
    int json_count = 0;
    DIR *dir = opendir("./dataset_output");
    if (dir) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            std::string filename = entry->d_name;
            if (filename.length() > 5 && filename.substr(filename.length() - 5) == ".json") {
                json_count++;
            }
        }
        closedir(dir);
    }
    cout << "Total JSON files created: " << json_count << endl;

    // Save trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
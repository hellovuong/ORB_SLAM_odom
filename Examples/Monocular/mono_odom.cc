/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<string>     // std::string, std::to_string
#include<Eigen/Dense>
#include<Eigen/Core>
#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
void LoadOdomPQ(const string &strOdomPath, vector<double> &vTimeStamps, vector<Eigen::Vector3d> &vPosition, 
                vector<Eigen::Quaterniond> &vOrientation);
void LoadOdom(const string &strOdomPath, vector<double> &vTimeStamps, vector<g2o::SE2> &vOdo);
int main(int argc, char **argv)
{



    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_odom path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    string dataPath = argv[3];
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    vector<Eigen::Vector3d> vPosition;
    vector<Eigen::Quaterniond> vOrientation;
    vector<g2o::SE2> vOdo;
    vector<double> vTimestampsOdom;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
    std::cout << "LOAED" << std::endl;
    //LoadOdomPQ(string(argv[3]), vTimestampsOdom, vPosition, vOrientation);
    LoadOdom(string(argv[3]), vTimestampsOdom,vOdo);
    std::cout << "LOAED" << std::endl;
    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // string fullOdomName = dataPath + "/odom.txt";
    // ifstream odomFile(fullOdomName);
    // float x, y, z, q_x, q_y, q_z, q_w, twist_lin_x, twist_lin_y, twist_lin_z, twist_ang_x, twist_ang_y, twist_ang_z;
    // string odomLine;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        std::cout<<tframe<<' '<<vOdo[ni].translation().x()<<' '<<vOdo[ni].translation().y()<<std::endl;
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackOdomMono(im, vOdo[ni], tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

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
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryOpenLoris("KeyFrameTrajectory_body.txt");    
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_cam.txt");
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/color/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        vstrImageFilenames[i] = strPrefixLeft + std::to_string(vTimestamps[i]) + ".png";
    }
}

void LoadOdomPQ(const string &strOdomPath, vector<double> &vTimeStamps, vector<Eigen::Vector3d> &vPosition, vector<Eigen::Quaterniond> &vOrientation)
{
    ifstream fOdom;
    fOdom.open(strOdomPath + "/odom_interp.txt");
    vPosition.reserve(5000);
    vOrientation.reserve(5000);
    double ts;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    std::cout<<"Reading Odometry Data:...\n";
    while(!fOdom.eof())
    {
        string s;
        getline(fOdom,s);
        std::cout<<s<<std::endl;
        if(s[0] == '#')
            continue;
        
        stringstream ss(s);
        ss >> ts;
        ss >> p.x() >> p.y() >>p.z();
        ss >> q.x() >> q.y() >> q.z() >> q.w();
        std::cout<<std::fixed<<std::setprecision(9);
        std::cout<<ts<<std::endl;
        std::cout<<p<<' '<<q<<std::endl;
        vPosition.push_back(p);
        vOrientation.push_back(q);
    }
    std::cout<<"Done..! Reading "<< vPosition.size()<<std::endl;
}


void LoadOdom(const string &strOdomPath, vector<double> &vTimeStamps, vector<g2o::SE2> &vOdo)
{
    ifstream fOdom;
    fOdom.open(strOdomPath + "/odom_interp.txt");
    vOdo.reserve(5000);
    double ts;
    Eigen::Vector3d xyr;
    std::cout<<"Reading Odometry Data:...\n";
    while(!fOdom.eof())
    {
        string s;
        getline(fOdom,s);
        //std::cout<<s<<std::endl;
        if(s[0] == '#')
            continue;
        
        stringstream ss(s);
        ss >> ts;
        vTimeStamps.push_back(ts);
        ss >> xyr(0) >> xyr(1) >> xyr(2);
        vOdo.push_back(g2o::SE2(xyr));
    }
    std::cout<<"Done..! Reading "<< vOdo.size()<<std::endl;
}
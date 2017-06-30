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

#include<opencv2/core/core.hpp>

#include<System.h>

#include "../src/IMU/imudata.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);
void LoadImus(const string &strImuPath, vector<ygz::IMUData> &vImus);

#include <stdlib.h>
#include <signal.h>
volatile sig_atomic_t sigflag = 0;
void sigint_function(int sig)
{
    LOG(INFO)<<"SIGINT catch"<<endl;
    sigflag = 1;
}

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        //cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file" << endl;
        cerr << endl << "Usage: ./mono_euroc_vins  vocabulary  settings  image_folder  image_times_file  imu_path" << endl;
        cerr<< endl <<"For example: "<<endl<<
               "./mono_euroc_vins ../../Vocabulary/ORBvoc.bin ../ROS/ORB_VIO/euroc.yaml  /media/jp/JingpangPassport/3dataset/EuRoC-VIO/zipfiles/V1_01_easy/mav0/cam0/data /media/jp/JingpangPassport/3dataset/EuRoC-VIO/zipfiles/V1_01_easy/mav0/cam0/data.csv /media/jp/JingpangPassport/3dataset/EuRoC-VIO/zipfiles/V1_01_easy/mav0/imu0/data.csv"
               <<endl;
        return 1;
    }
    
    signal(SIGINT, sigint_function);

    vector<ygz::IMUData> vImus;
    LoadImus(argv[5], vImus);
    int nImus = vImus.size();
    cout << "Imus in data: " << nImus << endl;
    if(nImus<=0)
    {
        cerr << "ERROR: Failed to load imus" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();
    cout << "Images in data: " << nImages << endl;

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ygz::System SLAM(argv[1],argv[2],ygz::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    int imagestart = 0;
    const double startimutime = vImus[0]._t;
    printf("startimutime: %.6f\n",startimutime);
    printf("imagestarttime: %.6f\n",vTimestamps[0]);
    while(1)
    {
        //printf("imagestarttime: %.6f\n",vTimestamps[imagestart]);
        if(startimutime <= vTimestamps[imagestart])
            break;
        imagestart++;

        if(sigflag) // ctrl-c exit
            return 1;
    }
    cout<<"imagestart:"<<imagestart<<endl;

    // Main loop
    long imuindex = 0;
    cv::Mat im;
    for(int ni=imagestart; ni<nImages; ni++)
    {
        if(sigflag) // ctrl-c exit
            break;

        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        vector<ygz::IMUData> vimu;
        while(1)
        {
            const ygz::IMUData& imudata = vImus[imuindex];
            if(imudata._t >= tframe)
                break;
            vimu.push_back(imudata);
            imuindex++;
        }

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        if(!SLAM.mpParams->GetUseIMUFlag())
            SLAM.TrackMonocular(im,tframe);
        else
        {
            SLAM.TrackMonoVI(im,vimu,tframe);
        }

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
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImus(const string &strImuPath, vector<ygz::IMUData> &vImus)
{
    ifstream fImus;
    fImus.open(strImuPath.c_str());
    vImus.reserve(30000);
    //int testcnt = 10;
    while(!fImus.eof())
    {
        string s;
        getline(fImus,s);
        if(!s.empty())
        {
            char c = s.at(0);
            if(c<'0' || c>'9')
                continue;

            stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            double data[10];    // timestamp, wx,wy,wz, ax,ay,az
            while(ss >> tmpd)
            {
                data[cnt] = tmpd;
                cnt++;
                if(cnt == 7)
                    break;
                if(ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            data[0] *= 1e-9;
            ygz::IMUData imudata(data[1],data[2],data[3],
                                       data[4],data[5],data[6], data[0]);
            vImus.push_back(imudata);

        }
    }
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    //int testcnt=10;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            char c = s.at(0);
            if(c<'0' || c>'9')
                continue;

            stringstream ss;
            ss << s;
            long tmpi;
            int cnt = 0;
            while(ss >> tmpi)
            {
                cnt++;
                if(cnt == 1)
                {
                    //cout<<tmpi<<endl;
                    break;
                }
                if(ss.peek() == ',' || ss.peek() == ' ' || ss.peek() == '.')
                    ss.ignore();
            }
            //string tmpstr(strImagePath + "/" + ss.str() + ".png");
            string tmpstr(strImagePath + "/" + to_string(tmpi) + ".png");
            //cout<<tmpstr<<endl;
            vstrImages.push_back(tmpstr);
            //double t;
            //ss >> t;
            vTimeStamps.push_back(tmpi*1e-9);

        }

        //if(testcnt--<=0)
        //    break;
    }
    /*for(size_t i=0; i<vstrImages.size(); i++)
    {
        printf("image: %s\n timestamp: %.9f\n",vstrImages[i].c_str(),vTimeStamps[i]);
    }*/
}

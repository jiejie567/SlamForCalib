/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM1,ORB_SLAM3::System* pSLAM2):mpSLAM1(pSLAM1),mpSLAM2(pSLAM2){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft1,const sensor_msgs::ImageConstPtr& msgRight1, const sensor_msgs::ImageConstPtr& msgRGB2,const sensor_msgs::ImageConstPtr& msgD2);

    bool do_rectify;
    cv::Mat M1l1,M2l1,M1r1,M2r1; //最后的数字表示大相机序号
    cv::Mat M1l2,M2l2,M1r2,M2r2;
    ORB_SLAM3::System* mpSLAM1;
    ORB_SLAM3::System* mpSLAM2;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo and RGBD");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings1 path_to_settings2 do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM1(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    ORB_SLAM3::System SLAM2(argv[1],argv[3],ORB_SLAM3::System::STEREO,false);
    ImageGrabber igb(&SLAM1,&SLAM2);

    stringstream ss(argv[4]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l1,igb.M2l1);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r1,igb.M2r1);

         // Load settings related to stereo calibration
        cv::FileStorage fsSettings2(argv[3], cv::FileStorage::READ);
        if(!fsSettings2.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        // cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings2["LEFT.K"] >> K_l;
        fsSettings2["RIGHT.K"] >> K_r;

        fsSettings2["LEFT.P"] >> P_l;
        fsSettings2["RIGHT.P"] >> P_r;

        fsSettings2["LEFT.R"] >> R_l;
        fsSettings2["RIGHT.R"] >> R_r;

        fsSettings2["LEFT.D"] >> D_l;
        fsSettings2["RIGHT.D"] >> D_r;

        rows_l = fsSettings2["LEFT.height"];
        cols_l = fsSettings2["LEFT.width"];
        rows_r = fsSettings2["RIGHT.height"];
        cols_r = fsSettings2["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l2,igb.M2l2);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r2,igb.M2r2);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub1(nh, "/cam0/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub1(nh, "/cam1/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> left_sub2(nh, "/cam0/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub2(nh, "/cam1/image_raw", 1);
      cout<<"建立订阅"<<endl;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub1,right_sub1, left_sub2,right_sub2);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2,_3,_4));

    ros::spin();

    // Stop all threads
     cout<<"ros::spin结束"<<endl;
    // Stop all threads
    SLAM1.Shutdown();
    SLAM2.Shutdown();

    cout<<"SLAM已关闭"<<endl;
    ros::shutdown();
    SLAM1.MergeSLAMs(&SLAM2);
    
    cout<<"节点结束"<<endl;
    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft1,const sensor_msgs::ImageConstPtr& msgRight1,const sensor_msgs::ImageConstPtr& msgLeft2,const sensor_msgs::ImageConstPtr& msgRight2)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft1;
    try
    {
        cv_ptrLeft1 = cv_bridge::toCvShare(msgLeft1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
        cv_bridge::CvImageConstPtr cv_ptrLeft2;
    try
    {
        cv_ptrLeft2 = cv_bridge::toCvShare(msgLeft2);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight1;
    try
    {
        cv_ptrRight1 = cv_bridge::toCvShare(msgRight1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptrRight2;
    try
    {
        cv_ptrRight2 = cv_bridge::toCvShare(msgRight2);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if(do_rectify)
    {
        cv::Mat imLeft1, imRight1, imLeft2, imRight2;

        cv::remap(cv_ptrLeft1->image,imLeft1,M1l1,M2l1,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight1->image,imRight1,M1r1,M2r1,cv::INTER_LINEAR);

        cv::remap(cv_ptrLeft2->image,imLeft2,M1l2,M2l2,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight2->image,imRight2,M1r2,M2r2,cv::INTER_LINEAR);

        mpSLAM1->TrackStereo(imLeft1,imRight1,cv_ptrLeft1->header.stamp.toSec());
        mpSLAM2->TrackStereo(imLeft2,imRight2,cv_ptrLeft1->header.stamp.toSec());
    }
    else
    {
        mpSLAM1->TrackStereo(cv_ptrLeft1->image,cv_ptrRight1->image,cv_ptrLeft1->header.stamp.toSec());
        mpSLAM2->TrackStereo(cv_ptrLeft2->image,cv_ptrRight2->image,cv_ptrLeft2->header.stamp.toSec());
    }

}



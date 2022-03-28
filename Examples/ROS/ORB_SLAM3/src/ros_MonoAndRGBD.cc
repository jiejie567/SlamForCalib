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

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB1, const sensor_msgs::ImageConstPtr& msgRGB2, const sensor_msgs::ImageConstPtr& msgD2);

    ORB_SLAM3::System* mpSLAM1;
    ORB_SLAM3::System* mpSLAM2;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "monoAndRGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 MonoAndRGBD path_to_vocabulary path_to_settings_1 path_to_settings_2" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM1(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    ORB_SLAM3::System SLAM2(argv[1],argv[3],ORB_SLAM3::System::RGBD,false);
    

    ImageGrabber igb(&SLAM1,&SLAM2);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub1(nh, "/camera/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub2(nh, "/camera2/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub2(nh, "/camera2/aligned_depth_to_color/image_raw", 100);
    cout<<"建立订阅"<<endl;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub1, rgb_sub2, depth_sub2);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3));

    ros::spin();
    cout<<"ros::spin结束"<<endl;
    // Stop all threads
    SLAM1.Shutdown();
    SLAM2.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    cout<<"SLAM已关闭"<<endl;
    ros::shutdown();
    SLAM1.MergeSLAMs(&SLAM2);
    
    cout<<"节点结束"<<endl;
    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB1,const sensor_msgs::ImageConstPtr& msgRGB2,const sensor_msgs::ImageConstPtr& msgD2)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB1;
    try
    {
        cv_ptrRGB1 = cv_bridge::toCvShare(msgRGB1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    
        // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB2;
    try
    {
        cv_ptrRGB2 = cv_bridge::toCvShare(msgRGB2);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptrD2;
    try
    {
        cv_ptrD2 = cv_bridge::toCvShare(msgD2);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM1->TrackMonocular(cv_ptrRGB1->image,cv_ptrRGB1->header.stamp.toSec());
    mpSLAM2->TrackRGBD(cv_ptrRGB2->image,cv_ptrD2->image,cv_ptrRGB2->header.stamp.toSec());
}



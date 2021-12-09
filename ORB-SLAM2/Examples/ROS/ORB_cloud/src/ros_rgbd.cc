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
#include"../../../include/System.h"
#include "../../../include/Converter.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/compression_profiles.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include "pointcloudmapping.h"
#include"System.h"
#include <sensor_msgs/PointCloud2.h>
#include <condition_variable>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include<tf/transform_broadcaster.h>
using namespace std;
using namespace ORB_SLAM2;
class PointCloudMapping;
class ImageGrabber
{
public:
    ImageGrabber(ros::NodeHandle* node,ORB_SLAM2::System* pSLAM,ros::Publisher* pose_pub = nullptr,
    ros::Publisher* odom_pub = nullptr,ros::Publisher* linear_pub = nullptr,
    ros::Publisher* angular_pub = nullptr):node(node),mpSLAM(pSLAM),pose_pub(pose_pub),odom_pub(odom_pub),linear_pub(linear_pub)
    ,angular_pub(angular_pub){

    	cloudpub = make_shared<thread>( bind(&ImageGrabber::pub, this) ); 

    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    bool is_ignore = false;
    ros::Publisher* pose_pub;
    ros::Publisher* odom_pub;
    ros::Publisher* angular_pub;
    ros::Publisher* linear_pub;
    ORB_SLAM2::System* mpSLAM;
    shared_ptr<thread> cloudpub;
    ros::NodeHandle* node;
    void pub();
};
geometry_msgs::Vector3   linear,angular;

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu){
    linear.x = -(imu->linear_acceleration).z;
    linear.y =  (imu->linear_acceleration).x;
    linear.z =  (imu->linear_acceleration).y;
    angular.x =  -(imu->angular_velocity).z;
    angular.y =  (imu->angular_velocity).x;
    angular.z =  (imu->angular_velocity).y;
}
void ImageGrabber::pub(){
	string frameid="world";
	ros::Publisher cloud_pub = node->advertise<sensor_msgs::PointCloud2>("/ORB_SLAM2/global_cloud",1,true);
	pcl::PointCloud<pcl::PointXYZ> pclmsg;
	sensor_msgs::PointCloud2 msg;
        while(1){

            std::unique_lock<std::mutex> lock((mpSLAM->mpPointCloudMapping)->g_pub_Mutex);
	    ((mpSLAM->mpPointCloudMapping)->g_pub).wait(lock);

            PointCloudMapping::PointCloud::Ptr pointmap = (mpSLAM->mpPointCloudMapping)->globalMap;
            for(int i=0;i<pointmap->points.size();i++)
            {
                pcl::PointXYZ point;
                point.x=pointmap->points[i].z;
                point.y=-pointmap->points[i].x;
                point.z=-pointmap->points[i].y;
                pclmsg.points.push_back(point);
            }
            lock.unlock();
	    pcl::toROSMsg(pclmsg,msg);
            msg.header.stamp=ros::Time::now();
            msg.header.frame_id=frameid;
	    cloud_pub.publish(msg);
	    usleep(1000);


	}

}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    ros::Publisher pose_pub;
    ros::Publisher odom_pub;
    ros::Publisher linear_pub;
    ros::Publisher angular_pub;
    ros::NodeHandle nh;
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ORB_SLAM2/pose", 20);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/ORB_SLAM2/odom", 20);
    linear_pub = nh.advertise<geometry_msgs::Vector3>("/ORB_SLAM2/linear", 20);
    angular_pub = nh.advertise<geometry_msgs::Vector3>("/ORB_SLAM2/angular", 20);
	ros::Subscriber imu = nh.subscribe("/camera/imu", 1, imu_callback);
    
    ImageGrabber igb(&nh,&SLAM,&pose_pub,&odom_pub,&linear_pub,&angular_pub);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    cv::Mat Tcw ;
    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec()); //获取位姿变换

    geometry_msgs::PoseStamped pose;
    nav_msgs::Odometry odom;
    ros::Time current_time = ros::Time::now();
    
    odom.header.stamp = current_time;
    odom.header.frame_id ="odom";
    pose.header.stamp = current_time;
    pose.header.frame_id ="path";
    try{
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

        tf::Transform new_transform;
        new_transform.setOrigin(tf::Vector3((twc.at<float>(0, 2)), -(twc.at<float>(0, 0)), -(twc.at<float>(0, 1))));
        tf::Quaternion quaternion(q[2], -q[0], -q[1], q[3]);

        new_transform.setRotation(quaternion);
        tf::poseTFToMsg(new_transform, pose.pose);
        odom.pose.pose = pose.pose;
        if (!is_ignore){
            odom.twist.twist.linear =  linear;
            odom.twist.twist.angular= angular;
            odom_pub->publish(odom);
            linear_pub->publish(linear);
            angular_pub->publish(angular);
        }
        pose_pub->publish(pose);
        is_ignore = false;
    }
    catch(...){
        is_ignore = true;
    }
    
}



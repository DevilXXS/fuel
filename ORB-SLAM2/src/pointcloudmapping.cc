/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "Converter.h"
#include "PointCloude.h"
#include "System.h"

int currentloopcount = 0;
PointCloudMapping::PointCloudMapping(double resolution_,double meank_,double thresh_)
{
    this->resolution = resolution_;
    this->meank = thresh_;
    this->thresh = thresh_;
    statistical_filter.setMeanK(meank);
    statistical_filter.setStddevMulThresh(thresh);
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );
    
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth,int idk,vector<KeyFrame*> vpKFs)
{
    cout<<"receive a keyframe, id = "<<idk<<" 第"<<kf->mnId<<"个"<<endl;
    //cout<<"vpKFs数量"<<vpKFs.size()<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    currentvpKFs = vpKFs;
    //colorImgs.push_back( color.clone() );
    //depthImgs.push_back( depth.clone() );
    PointCloude pointcloude;
    pointcloude.pcID = idk;
    pointcloude.T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    pointcloude.pcE = generatePointCloud(kf,color,depth);
    pointcloud.push_back(pointcloude);
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)//,Eigen::Isometry3d T
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>5)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
    //Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    //PointCloud::Ptr cloud(new PointCloud);
    //pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    //cloud->is_dense = false;
    
    cout<<"generate point cloud for kf "<<endl;
    return tmp;
    
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    //pcl::visualization::PCLVisualizer viewer("viewer");
    while(1)
    {
        cout<<"viewer_init\n";
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
	cout<<"viewer1\n";
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );//keyFrameUpdateMutex 别的地方没有，感觉没用的锁，只是为了可以用wait 
            keyFrameUpdated.wait( lck_keyframeUpdated );  //keyFrameUpdated   锁进程，阻塞状态，直到关键帧插入完毕
        }
        cout<<"viewer2\n";
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );//keyframeMutex插入关键帧的锁  
            N = keyframes.size();
        }
        if(loopbusy || bStop)
        {
          //cout<<"loopbusy || bStop"<<endl;
	    //cout<<loopbusy<<"	"<<bStop<<'\n';
            continue;
        }
	cout<<"viewer3\n";
        //cout<<lastKeyframeSize<<"    "<<N<<endl;
        if(lastKeyframeSize == N)
            cloudbusy = false;
        //cout<<"待处理点云个数 = "<<N<<endl;
          cloudbusy = true;
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {

          
            PointCloud::Ptr p (new PointCloud);
            pcl::transformPointCloud( *(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());
            //cout<<"处理好第i个点云"<<i<<endl;
            *globalMap += *p;
            //PointCloud::Ptr tmp(new PointCloud());
            //voxel.setInputCloud( globalMap );
           // voxel.filter( *tmp );
            //globalMap->swap( *tmp );
           
 
        }
        cout<<"viewer4\n";
        // depth filter and statistical removal 
        PointCloud::Ptr tmp1 ( new PointCloud );

        std::unique_lock<std::mutex> lock_(g_pub_Mutex);

        statistical_filter.setInputCloud(globalMap);
        statistical_filter.filter( *tmp1 );

        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *globalMap );
        //globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        //viewer.removeAllPointClouds();
        //viewer.removeAllShapes();
        //viewer.addPointCloud(globalMap);
        //viewer.spinOnce();
        cout<<"show global map, size="<<N<<"   "<<globalMap->points.size()<<endl;
        lastKeyframeSize = N;
        cloudbusy = false;

        lock_.unlock();
        g_pub.notify_one();
        //usleep(2000);
        //*globalMap = *tmp1;
        
        //if()
        //{
	    
	//}
    }
}
void PointCloudMapping::save()
{
	pcl::io::savePCDFile( "result.pcd", *globalMap );
	cout<<"globalMap save finished"<<endl;
}
void PointCloudMapping::updatecloud()
{
	if(!cloudbusy)
	{
		loopbusy = true;
		cout<<"startloopmappoint"<<endl;
        PointCloud::Ptr tmp1(new PointCloud);
		for (int i=0;i<currentvpKFs.size();i++)
		{
		    for (int j=0;j<pointcloud.size();j++)
		    {   
				if(pointcloud[j].pcID==currentvpKFs[i]->mnFrameId) 
				{   
					Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
					PointCloud::Ptr cloud(new PointCloud);
					pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, T.inverse().matrix());
					*tmp1 +=*cloud;

					//cout<<"第pointcloud"<<j<<"与第vpKFs"<<i<<"匹配"<<endl;
					continue;
				}
			}
		}
        cout<<"finishloopmap"<<endl;
        PointCloud::Ptr tmp2(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *tmp2 );
        globalMap->swap( *tmp2 );
        //viewer.showCloud( globalMap );
        loopbusy = false;
        //cloudbusy = true;
        loopcount++;

        //*globalMap = *tmp1;
	}
}

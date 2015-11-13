#ifndef ros_publisher_H
#define ros_publisher_H

//std C++
#include <iostream>
#include <iomanip>
#include <cstdio>

// ros standard
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
//#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <visualization_msgs/MarkerArray.h>

//ros cargo ants
#include <cargo_ants_msgs/ObjectTrackAtSdfArray.h>

////at_udp2ros
#include "decode_bytes.h"
#include "constants.h"

// //constants
// #define NUM_IBEO_LAYERS 8 
// #define IBEO_NUM_POINTS 1000

struct lidar_udp_data {
	double tmsg;
	unsigned int sizeMsg;
    unsigned short scanNo;
    unsigned int scanStat;
    double syncPhOff;
    double startTime;
    double endTime;
    unsigned short angTicks;
    short startAng;
    short endAng;
    unsigned short scanPts;
    float mntYaw;
    float mntPitch;
    float mntRoll;
    float mntX;
    float mntY;
    float mntZ;
    unsigned short flags;
    bool upper_layer;
    std::vector<unsigned char> scanPtLayer;
	std::vector<unsigned char> scanPtEcho;
	std::vector<unsigned char> scanFlags;
	std::vector<float> scanPtAngle;
	std::vector<float> scanPtDist;
	std::vector<float> scanPtEchoWidth;
    std::vector<unsigned short> scanPtRsv;

    lidar_udp_data() :
		scanPtLayer(1000),
		scanPtEcho(1000),
		scanFlags(1000),
		scanPtAngle(1000),
		scanPtDist(1000),
		scanPtEchoWidth(1000),
		scanPtRsv(1000)
    {
    	//
    }

    bool fill(unsigned char *buf)
    {
    	scanPtLayer.assign(&buf[80], &buf[1080]);
		scanPtEcho.assign(&buf[1080], &buf[2080]);
		scanFlags.assign(&buf[2080], &buf[3080]);

    	if (bool zeros = std::all_of(scanPtLayer.begin(), scanPtLayer.end(), [](int i) { return i==0; }))
    	{
    		std::cout << "full of zeros ibeo message. Upper layer: " << bool(flags & 0x0040) << std::endl;
    		return false;
    	}

    	tmsg      = getDouble(buf, 0, true);
    	sizeMsg   = getUint32(buf, 8, true);
    	scanNo    = getUint16(buf, 12, true);
    	scanStat  = getUint32(buf, 14, true); //TBC
    	syncPhOff = getDouble(buf, 18, true);
    	startTime = getDouble(buf, 26, true);
    	endTime   = getDouble(buf, 34, true);
    	angTicks  = getUint16(buf, 42, true);
    	startAng  = getSingle(buf, 44, true);
    	endAng    = getSingle(buf, 48, true);
    	scanPts   = getUint16(buf, 52, true);
    	mntYaw    = getSingle(buf, 54, true);
    	mntPitch  = getSingle(buf, 58, true);
    	mntRoll   = getSingle(buf, 62, true);
    	mntX      = getSingle(buf, 66, true);
    	mntY      = getSingle(buf, 70, true);
    	mntZ      = getSingle(buf, 74, true);
    	flags     = getUint16(buf, 78, true);
    	upper_layer = (flags & 0x0040);
    	//printf("flags: %i --- %i\n", flags, (flags & 0x0040) );
    	//std::cout << "processed ibeo message, upper layer: " << upper_layer << std::endl;

		for (unsigned int i=0; i<1000; i++)
		{
			scanPtAngle[i]     = getSingle(buf, 3080+i*4, true);
			scanPtDist[i]      = getSingle(buf, 7080+i*4, true);
			scanPtEchoWidth[i] = getSingle(buf,11080+i*4, true);
			scanPtRsv[i]       = getUint16(buf,15080+i*2, true);
		}

//		std::cout << std::endl << " ********************************************* " << std::endl;
//		printf("tmsg: %12.12f\n", tmsg);
//		std::cout << "sizeMsg: " << sizeMsg << std::endl;
//		std::cout << "scanNo: " << scanNo << std::endl;
//		std::cout << "scanStat: " << scanStat << std::endl;
//		std::cout << "syncPhOff: " << syncPhOff << std::endl;
//		std::cout << "startTime: " << startTime << std::endl;
//		std::cout << "endTime: " << endTime << std::endl;
//		std::cout << "angTicks: " << angTicks << std::endl;
//		std::cout << "startAng: " << startAng << std::endl;
//		std::cout << "endAng: " << endAng << std::endl;
//		std::cout << "scanPts: " << scanPts << std::endl;
//		std::cout << "mntYaw: " << mntYaw << std::endl;
//		std::cout << "mntPitch: " << mntPitch << std::endl;
//		std::cout << "mntRoll: " << mntRoll << std::endl;
//		std::cout << "mntX: " << mntX << std::endl;
//		std::cout << "mntY: " << mntY << std::endl;
//		std::cout << "mntZ: " << mntZ << std::endl;
//		std::cout << "flags: " << flags << std::endl;
//		for (unsigned int ii = 0; ii<scanPtLayer.size(); ii++)
//		{
//			std::cout << (unsigned int)scanPtLayer.at(ii) << "\t";
//			std::cout << (unsigned int)scanPtEcho.at(ii) << "\t";
//			std::cout << (unsigned int)scanFlags.at(ii) << "\t";
//			std::cout << scanPtAngle.at(ii) << "\t";
//			std::cout << scanPtDist.at(ii) << "\t";
//			std::cout << scanPtEchoWidth.at(ii) << std::endl;
//		}
//		std::cout << std::endl;

		return true;
    }
};

class ROSPublisher
{
	protected:
		//node handle
		ros::NodeHandle nh_;
		
		//odom publisher and message
		ros::Publisher odom_pub_;
		nav_msgs::Odometry odom_msg_;
		
		//aux variables for odom integration
		float px_, py_, th_; //pose
		ros::Time ts_current_, ts_past_; //time
		
		//odometry transform broadcatser
        tf::TransformBroadcaster tfb_;

		//odom publisher and message
		ros::Publisher tracks_pub_;
		cargo_ants_msgs::ObjectTrackAtSdfArray tracks_msg_;
		
		//Markers for tracks
		ros::Publisher tracks_markers_pub_;
		visualization_msgs::MarkerArray markers_tracks_msg_;
		
        //lidar publisher array and message array
		ros::Publisher scan_pub_[NUM_IBEO_LAYERS];
        ros::Publisher pcl_pub_upper;   // for the ibeo 4th upper layers
        ros::Publisher pcl_pub_lower;   // for the ibeo 4th lower layers
		sensor_msgs::LaserScan scan_[NUM_IBEO_LAYERS];
        sensor_msgs::PointCloud pcl_ibeo_;
		
		// lidar data struct
		lidar_udp_data lidar_data_;
		unsigned int N_ranges_;

	    // auxiliar variables for layer clasification
		std::vector<unsigned int> upper_layers_;
		std::vector<unsigned int> lower_layers_;
		std::vector<double> sin_betas_;

	public:
		ROSPublisher(int _op_mode);
		~ROSPublisher();
		void publishSdf(unsigned char *buf, size_t length, const ros::Time& _stamp);
		bool fillLidarData(unsigned char *buf, size_t length);
		void publishScans(const ros::Time& _stamp);
        void publishPointCloud(const ros::Time& _stamp);
};
#endif


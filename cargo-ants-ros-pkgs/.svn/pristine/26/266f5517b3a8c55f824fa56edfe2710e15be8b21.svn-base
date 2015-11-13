//local dependencies
#include "ros_publisher.h"

ROSPublisher::ROSPublisher(int _op_mode):
    nh_(ros::this_node::getName()),
    upper_layers_({4,5,6,7}),
	lower_layers_({0,1,2,3}),
	sin_betas_({sin(-2.8*M_PI/180), sin(-2*M_PI/180), sin(-1.2*M_PI/180), sin(-0.4*M_PI/180), sin(0.4*M_PI/180), sin(1.2*M_PI/180), sin(2*M_PI/180), sin(2.8*M_PI/180)})
{
	//auxiliar string for lidar topic naming
	std::stringstream lidar;
	
	//odometry integration inits
	px_ = 0;
	py_ = 0;
	th_ = 0;
	ts_current_ = ros::Time::now(); 
	
    // init publishers
	switch (_op_mode)
	{
		case 1: //sdf
			odom_pub_= nh_.advertise<nav_msgs::Odometry>("odom", 50);
			tracks_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tracks", 50);
			tracks_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 50);
			break; 
			
		case 2: //ibeo
            pcl_pub_lower = nh_.advertise<sensor_msgs::PointCloud>("at_front_ibeo_pcl_lower", 50);
            pcl_pub_upper = nh_.advertise<sensor_msgs::PointCloud>("at_front_ibeo_pcl_upper", 50);

			for (unsigned int j = 0; j < NUM_IBEO_LAYERS; j++)
			{
				// init constant laser scan message fields
				lidar.str("");
				lidar << "at_front_ibeo_layer" << j;
				scan_[j].header.frame_id = lidar.str();
				scan_[j].range_min = 0.8;
				scan_[j].range_max = 200;
				scan_[j].angle_min = -60 * M_PI / 180.;//50;
				scan_[j].angle_max = 50 * M_PI / 180.;
				scan_[j].angle_increment = 0.25 * M_PI / 180.;
				N_ranges_ = (scan_[j].angle_max - scan_[j].angle_min) / scan_[j].angle_increment;
				scan_[j].time_increment = 0;
				scan_[j].ranges.resize(N_ranges_);
				scan_[j].intensities.resize(N_ranges_);

				// init publisher
				lidar.str("");
				lidar << "scan_" << j;
				scan_pub_[j] = nh_.advertise<sensor_msgs::LaserScan>(lidar.str(), 50);
			}
			break;
			
		default: 
			std::cout << "ROSPublisher::ROSPublisher(): Unknown op mode. ROS publishers not initialized properly." << std::endl;
			break; 
	}
}

ROSPublisher::~ROSPublisher()
{
    //
	std::cout << "~ROSPublisher(): " << __LINE__ << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////
//   This function publishes the SDF data. It receives a message data of 
//   unsigned chars, which was previously built from the data coming 
//   from  4 UDP packets
//
void ROSPublisher::publishSdf(unsigned char *buf, size_t length, const ros::Time& _stamp)
{
	//odom variables 
	float vx,wz; //2D twist (lateral velocity not provided in raw message.)
	double dt; 
	ros::Duration dt_ros;
	
	//track variables
	double tx, ty, tvx, tvy, theading;
	unsigned int objValid_uint, objId_uint, objVehType_uint; 
	
	//time management
	ts_past_ = ts_current_;
    ts_current_ = _stamp;
	//double ts_current_received_raw = getDouble(buf, 4, true); //using ROS time, no the one provided by the message
	//printf("ts_current_: %1.10f\n", (double)ts_current_.sec + (double)ts_current_.nsec/(double)1e9);
     		
    //get data from raw message
	//px = getSingle(buf, 48, true);//x-pos: Longitudinal position of ego, [m] (egoLongPos in document) 
	//py = getSingle(buf, 52, true);//y-pos: Lateral position of ego, [m] (egoLatPos in document) 
	//th = getSingle(buf, 56, true);//heading: heading angle, [rad] (egoHeadingAngle in document) 
	vx = getSingle(buf, 40, true);//vx: Ego vehicle forward velocity, [m/s] (egoSpeed in document)
	wz = getSingle(buf, 60, true);//wx: Ego vehicle yaw rate [TBC: rad/s] (egoYawRate in document)	
	
	//integrate 2D twist to get odometry pose
	dt_ros = ts_current_ - ts_past_;
	dt = ((double)dt_ros.sec + (double)dt_ros.nsec/(double)1e9);
	th_ = th_ + wz*dt/2.; 
	if (th_<-M_PI) th_ = 2*M_PI+th_; 
	else if (th_>=M_PI) th_ = -2*M_PI+th_;
	px_ = px_ + vx*dt*cos(th_);
	py_ = py_ + vx*dt*sin(th_);
	th_ = th_ + wz*dt/2.; 
	if (th_<-M_PI) th_ = 2*M_PI+th_; 
	else if (th_>=M_PI) th_ = -2*M_PI+th_;
	
	//fill & publish odometry message
	//header
	odom_msg_.header.stamp = ts_current_; 
	odom_msg_.header.frame_id = "odom";
	odom_msg_.child_frame_id = "at_footprint";
	//pose
	odom_msg_.pose.pose.position.x = px_;
	odom_msg_.pose.pose.position.y = py_;
	odom_msg_.pose.pose.position.z = 0.;
	//pose orientation
	tf::Quaternion qt = tf::createQuaternionFromYaw(th_);
	odom_msg_.pose.pose.orientation.x = qt.getX();
	odom_msg_.pose.pose.orientation.y = qt.getY();
	odom_msg_.pose.pose.orientation.z = qt.getZ();
	odom_msg_.pose.pose.orientation.w = qt.getW();
	//twist (2D)
	odom_msg_.twist.twist.linear.x = vx;
	odom_msg_.twist.twist.linear.y = 0.;//lateral velocity not provided in raw message. Forced to 0.  
	odom_msg_.twist.twist.linear.z = 0.;
	odom_msg_.twist.twist.angular.x = 0.;
	odom_msg_.twist.twist.angular.y = 0.;
	odom_msg_.twist.twist.angular.z = wz;
	//publish
	odom_pub_.publish(odom_msg_);
	
	//fill and broadcast transform
	tf::Transform odom2base;
	odom2base.setOrigin( tf::Vector3(px_, py_, 0.) );
	odom2base.setRotation(qt);
	tfb_.sendTransform( tf::StampedTransform(odom2base, ts_current_, "odom", "at_footprint") );  
 
	//Fill tracks and marker array messages, and publish then
	unsigned char num_objects_uc = buf[16]; 
	//std::cout << "Num Objects uc: " << (unsigned int)num_objects_uc << std::endl; 

	//fill track message header and resize array
	tracks_msg_.header.frame_id = "at_footprint";
	tracks_msg_.header.stamp = ros::Time::now(); 
	tracks_msg_.tracks.resize(num_objects_uc );
			
	//resize marker array
	markers_tracks_msg_.markers.clear();
	markers_tracks_msg_.markers.resize( num_objects_uc );
	unsigned int obj_count = 0;
	
	//fill markers
	for ( unsigned int ii=0; ii<(unsigned int)num_objects_uc; ii++ )
	{
		//get message data. 73 is the size in bytes of all fields corresponding to a single object 
		objValid_uint = (unsigned int)buf[97+ii*73]; 
		objId_uint = getUint32(buf,98+ii*73,true);
		objVehType_uint = (unsigned int)buf[102+ii*73];
		tx = getSingle(buf,104+ii*73,true);
		ty = getSingle(buf,108+ii*73,true);
		tvx = getSingle(buf,128+ii*73,true);
		tvy = getSingle(buf,132+ii*73,true);
		theading = getSingle(buf,112+ii*73,true);
		
		//Raw print of message data
// 		std::cout << "objValid_uint: " << objValid_uint << std::endl;
// 		std::cout << "objId_uint: " << objId_uint << std::endl;
// 		std::cout << "objVehType_uint: " << objVehType_uint << std::endl;
// 		std::cout << "objLongPos: " << tx << std::endl;
// 		std::cout << "objLatPos: " << ty << std::endl;
		
		//fill markers
		if ( objValid_uint > 0 ) //>0 means object valid according documentation (Specification_of_UDP_communication.pdf) 
		{
			//tracks
			tracks_msg_.tracks[obj_count].id = objId_uint;
			tracks_msg_.tracks[obj_count].id = objVehType_uint;
			tracks_msg_.tracks[obj_count].position.x = tx;
			tracks_msg_.tracks[obj_count].position.y = ty;
			tracks_msg_.tracks[obj_count].position.z = 0.;
			tracks_msg_.tracks[obj_count].velocity.x = tvx;
			tracks_msg_.tracks[obj_count].velocity.y = tvy;
			tracks_msg_.tracks[obj_count].velocity.z = 0.;
			tracks_msg_.tracks[obj_count].heading = theading;
			
			//markers
			markers_tracks_msg_.markers[obj_count].header.frame_id = "at_footprint";
			markers_tracks_msg_.markers[obj_count].header.stamp = ros::Time::now();
			markers_tracks_msg_.markers[obj_count].id = objId_uint;
			markers_tracks_msg_.markers[obj_count].type = visualization_msgs::Marker::CYLINDER;
			markers_tracks_msg_.markers[obj_count].action = visualization_msgs::Marker::ADD;
			markers_tracks_msg_.markers[obj_count].pose.position.x = tx;
			markers_tracks_msg_.markers[obj_count].pose.position.y = ty;
			markers_tracks_msg_.markers[obj_count].pose.position.z = 0.;
			markers_tracks_msg_.markers[obj_count].scale.x = 0.5;
			markers_tracks_msg_.markers[obj_count].scale.y = 0.5;
			markers_tracks_msg_.markers[obj_count].scale.z = 0.5;
			markers_tracks_msg_.markers[obj_count].color.a = 1.;
			markers_tracks_msg_.markers[obj_count].color.r = 1.;
			markers_tracks_msg_.markers[obj_count].color.g = 0.2;
			markers_tracks_msg_.markers[obj_count].color.b = 0.2;
			markers_tracks_msg_.markers[obj_count].lifetime = ros::Duration(0.1);
			obj_count ++;
			
		}		
	}
	
	//resize again with tha actual num of objects
	tracks_msg_.tracks.resize(obj_count);
	markers_tracks_msg_.markers.resize(obj_count);

	//publish
	tracks_pub_.publish(tracks_msg_);
	tracks_markers_pub_.publish(markers_tracks_msg_);

};

bool ROSPublisher::fillLidarData(unsigned char *buf, size_t length)
{
	// fill lidar data from buffer
	return lidar_data_.fill(buf);
}

/////////////////////////////////////////////////////////////////////////////////
//   This function publishes the LIDAR data in 8 laserscan messages. It receives a message data of
//   unsigned chars, which was previously built from the data coming 
//   from 12 UDP packets
//
void ROSPublisher::publishScans(const ros::Time& _stamp)
{
	assert(lidar_data_.scanPtLayer.size() == 1000 && "bad scanPtLayer.size()");
	assert(lidar_data_.scanPtAngle.size() == 1000 && "bad scanPtAngle.size()");
	assert(lidar_data_.scanPtDist.size() == 1000 && "bad scanPtDist.size()");
	assert(lidar_data_.scanPtEchoWidth.size() == 1000 && "bad scanPtEchoWidth.size()");

	unsigned int layer_idx, idx, begin_layer, end_layer;
	begin_layer = (lidar_data_.upper_layer ? 4 : 0);
	end_layer   = (lidar_data_.upper_layer ? 8 : 4);

	for (unsigned int j = begin_layer; j < end_layer; j++)
    {
		// fill stamp
		scan_[j].header.stamp = _stamp;

		// reset ranges and intensities
		std::fill(scan_[j].ranges.begin(), scan_[j].ranges.end(), 0); //reset ranges
		std::fill(scan_[j].intensities.begin(), scan_[j].intensities.end(), 0); //reset intensities
    }

	unsigned int zeros_in = 0, zeros_out = 0;
	// fill ranges
    for (unsigned int i = 0; i<lidar_data_.scanPtLayer.size(); i++)//lidar_data_.scanPts
    {
    	if (lidar_data_.scanPtLayer[i] < 4 && !(lidar_data_.scanFlags[i] & 0x01) && !(lidar_data_.scanFlags[i] & 0x02) && !(lidar_data_.scanFlags[i] & 0x08))
    	{
			if (lidar_data_.upper_layer)
				layer_idx = upper_layers_[lidar_data_.scanPtLayer[i]];
			else
				layer_idx = lower_layers_[lidar_data_.scanPtLayer[i]];

			idx = nearbyint(( lidar_data_.scanPtAngle[i] * M_PI / 180 - scan_[layer_idx].angle_min) / scan_[layer_idx].angle_increment);

			if (idx >= 0 && idx < N_ranges_ && (scan_[layer_idx].ranges[idx] == 0 || scan_[layer_idx].ranges[idx] > lidar_data_.scanPtDist[i]))
			{
				scan_[layer_idx].ranges[idx] = lidar_data_.scanPtDist[i];
				scan_[layer_idx].intensities[idx] = lidar_data_.scanPtEchoWidth[i];
			}
    	}
    }
    // Publish ros messages
    for (unsigned int j = begin_layer; j < end_layer; j++)
    	scan_pub_[j].publish(scan_[j]);
}

/////////////////////////////////////////////////////////////////////////////////
//   This function publishes the LIDAR data in a PointCloud message. It receives a message data of
//   unsigned chars, which was previously built from the data coming
//   from 12 UDP packets
//
void ROSPublisher::publishPointCloud(const ros::Time& _stamp){

    // Extra information structures. Int will encode pulsewidth received in the point and distance the layer
    sensor_msgs::ChannelFloat32 channelInt;
    sensor_msgs::ChannelFloat32 channelLayer;
    sensor_msgs::ChannelFloat32 channelFlag;
//    sensor_msgs::ChannelFloat32 radialDistance;
    channelInt.name = "pulse_width";
    channelLayer.name = "layer";
    channelFlag.name = "flag";
    channelInt.values.reserve(lidar_data_.scanPtAngle.size());   // TBD: size = numPoints?
    channelLayer.values.reserve(lidar_data_.scanPtAngle.size());
    channelFlag.values.reserve(lidar_data_.scanPtAngle.size());
    pcl_ibeo_.channels.clear();

    unsigned int layer_idx;

    // Fill Header
    this->pcl_ibeo_.header.frame_id = "at_front_ibeo";
    this->pcl_ibeo_.header.stamp = _stamp;

    // Fill data of the message
    geometry_msgs::Point32 pointPCL;
    this->pcl_ibeo_.points.clear();
    this->pcl_ibeo_.points.reserve(lidar_data_.scanPtAngle.size());  // TBD: size = numPoints on scan
    for (auto ii = 0; ii < lidar_data_.scanPtAngle.size() && ii < 1000; ii++)
    {
    	if (lidar_data_.scanPtLayer[ii] < 4 && !(lidar_data_.scanFlags[ii] & 0x01) && !(lidar_data_.scanFlags[ii] & 0x02) && !(lidar_data_.scanFlags[ii] & 0x08))
    	{
			if (lidar_data_.upper_layer)
				layer_idx = upper_layers_[lidar_data_.scanPtLayer[ii]];
			else
				layer_idx = lower_layers_[lidar_data_.scanPtLayer[ii]];

			pointPCL.x = std::cos(M_PI*lidar_data_.scanPtAngle[ii]/180)*lidar_data_.scanPtDist[ii];
			pointPCL.y = std::sin(M_PI*lidar_data_.scanPtAngle[ii]/180)*lidar_data_.scanPtDist[ii];
			pointPCL.z = lidar_data_.scanPtDist[ii]*sin_betas_[layer_idx];

            pcl_ibeo_.points.push_back(pointPCL);
			channelLayer.values.push_back(layer_idx);
			channelInt.values.push_back(lidar_data_.scanPtEchoWidth[ii]);
            channelFlag.values.push_back(lidar_data_.scanFlags[ii]);
        }
    }
    // filling the channels
    pcl_ibeo_.channels.push_back(channelInt);
    pcl_ibeo_.channels.push_back(channelLayer);
    pcl_ibeo_.channels.push_back(channelFlag);


    if (lidar_data_.upper_layer)
        pcl_pub_upper.publish(pcl_ibeo_);
    else
        pcl_pub_lower.publish(pcl_ibeo_);

    //    // Fill Message PCL 2
    //    this->pcl_ibeo.height = 1;
    //    this->pcl_ibeo.width = lidar_data_.scanPtDist.size();

    //    sensor_msgs::PointField ptfield;
    //    ptfield.name = "xyz";
    //    ptfield.offset = 0;
    //    ptfield.datatype = ptfield.UINT32;
    //    ptfield.count = 3;
    //    this->pcl_ibeo.fields.push_back(ptfield);

    //    this->pcl_ibeo.is_bigendian = false;
    //    this->pcl_ibeo.point_step = 4;
    //    this->pcl_ibeo.row_step = 4*this->pcl_ibeo.width;

    //    for (auto ii = 0; ii< this->pcl_ibeo.width; ii++)
    //    {
    //        unsigned int x_pt = std::cos(lidar_data_.scanPtAngle[ii])*lidar_data_.scanPtDist[ii];
    //        unsigned int y_pt = std::sin(lidar_data_.scanPtAngle[ii])*lidar_data_.scanPtDist[ii];
    //        unsigned int z_pt = 1;
    //    }

}

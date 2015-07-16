#include "local_map_node.h"

LocalMapNode::LocalMapNode() :
    nh_(ros::this_node::getName()), 
    grid_(30,20,0.25,0.25,30,10,10)
{
    //set desired loop rate
    rate_ = 10.0; //Hz

    //allocate the grid_ object. 
    //TODO: get params from nh_.getParam() ... Edit .cfg + .yaml files
    //Grid2D *grid_ = new Grid2D(10,10,0.5,0.5,10,5,20);
    grid_.print(); 
    
    //set up subscribers with callbacks
    odometry_subscriber_ = nh_.subscribe("odometry", 100, &LocalMapNode::odometryCallback, this);
    front_laser_subscriber_ = nh_.subscribe("front_laser", 100, &LocalMapNode::laserCallback, this);
    
    //advertise publishers
    grid_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_map", 100);
    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 100);
}

LocalMapNode::~LocalMapNode()
{
    //free memory 
    //delete grid_;
}

double LocalMapNode::getRate() const
{
    return rate_;
}

void LocalMapNode::process()
{
    std::cout << "LocalMapNode::process()" << std::endl;
}

void LocalMapNode::publish()
{
    //fill message header
    grid_msg_.header.stamp = ros::Time::now(); 
    grid_msg_.header.frame_id = "base";
    
    //fill map info
    grid_msg_.info.resolution = grid_.getGridStep();
    grid_msg_.info.width = grid_.getNx();
    grid_msg_.info.height = grid_.getNy();
    grid_msg_.info.origin.position.x = grid_.getXmax()-grid_.getSizeX();
    grid_msg_.info.origin.position.y = grid_.getYmax()-grid_.getSizeY();
    grid_msg_.info.origin.position.z = 0.;
    grid_msg_.info.origin.orientation.x = 0.;
    grid_msg_.info.origin.orientation.y = 0.;
    grid_msg_.info.origin.orientation.z = 0.;
    grid_msg_.info.origin.orientation.w = 1.;

    //fill map data
    grid_.getGrid(grid_msg_.data);
    
    //publish
    grid_publisher_.publish(grid_msg_);
}

void LocalMapNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& _msg)
{
    // CASE TWIST. Preferred case, but TNO "october-rosbag" does not provide twist ...
    //TODO: dt as a config param or read header time diff between consecutive messages. Now @ 5Hz -> 0.2s
    //grid_.odometryUpdate(_msg->twist.twist.linear.x, _msg->twist.twist.linear.y, _msg->twist.twist.angular.z, 0.2); 
    
    // CASE RELATIVE ODOM (2D case). Set 1s as integ. time
    grid_.odometryUpdate(_msg->pose.pose.position.x, _msg->pose.pose.position.y, _msg->twist.twist.angular.z, 1);
}

void LocalMapNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& _msg)
{
    Eigen::MatrixXd points(2,_msg->ranges.size());
    unsigned int ii = 0; 
    unsigned int ii_ok = 0; 
    double azimuth = _msg->angle_min; 
    
    //filter out bad scan points (NaN, out of range) and convert to xy
    for ( ii = 0; ii<_msg->ranges.size(); ii++ )
    {
        //check data integrity
        if ( (_msg->ranges[ii]>_msg->range_min) && (_msg->ranges[ii]<_msg->range_max) && !isnan(_msg->ranges[ii]) && !isinf(_msg->ranges[ii]) )
        {
            //transform to euclidean
            points(0,ii_ok) = (double)_msg->ranges[ii]*cos(azimuth)+3.5; //TODO: avoid this hard TF !!!
            points(1,ii_ok) = (double)_msg->ranges[ii]*sin(azimuth);
            ii_ok ++;
        }
        
        //update azimuth
        azimuth += _msg->angle_increment; 
    }
    
    //resize points matrix
    //points.conservativeResize(2,ii_ok);
    //std::cout << " LocalMapNode::laserCallback(). ii_ok: " << ii_ok << std::endl;
    points.resize(2,ii_ok);// ; wouldn't be required since new size is smaller or equal
    
    //update grid with lidar
    grid_.lidarUpdate(points);
}

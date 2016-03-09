#include "slam_node.h"

SlamNode::SlamNode() :
    nh_(ros::this_node::getName())
{    
    int param_x; 
    std::ostringstream topic_name;
    std::string laser_link_name; 
    unsigned int laser_idx_offset; 
    std::ostringstream laser_link_name_full; 
    tf::StampedTransform base_2_sensor;
    tf::Matrix3x3 rmat; 

    //1. get algorithm params from dynamic reconfigure
    nh_.getParam("verbose_mode", this->verbose_);
    nh_.getParam("slam_rate", this->rate_);
    nh_.getParam("num_lasers", param_x); num_lasers_ = (unsigned int)param_x;
    nh_.getParam("base_link_name", this->base_link_name_);
    nh_.getParam("laser_idx_offset", param_x); laser_idx_offset = (unsigned int)param_x;
    nh_.getParam("laser_link_name", laser_link_name);
    
    
    //2. Set up publishers and subscribers
    //set up odometry subscriber with callback
    odometry_subscriber_ = nh_.subscribe("odometry", 100, &SlamNode::odometryCallback, this);
    if ( verbose_ ) std::cout << "SlamNode(): " << __LINE__ <<  std::endl;
    
    //set up laser subscribers with callbacks
    laser_subscribers_.resize(num_lasers_); 
    for (unsigned int ii = 0; ii<laser_subscribers_.size(); ii++)
    {
        //set subscribers
        topic_name.str(""); //clears previous content
        topic_name << "laser" << ii;
        laser_subscribers_.at(ii) = nh_.subscribe(topic_name.str(), 1, &SlamNode::laserCallback, this);
    }
    
    //advertise publishers
    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 100);
    
    
    //3. Get sensor mouniting positions from tf
    //set laser_link_id_map_ and laser_mounting_points_ with the homogeneous matrix (assumed fixed)
    laser_mounting_points_.resize(num_lasers_);
    for (unsigned int ii = 0; ii<laser_subscribers_.size(); ii++)
    {
        laser_link_name_full.str("");
        laser_link_name_full << laser_link_name << (ii+laser_idx_offset);
        laser_link_id_map_[laser_link_name_full.str()] = ii; //set mapping between link name and id
        //std::cout << "laser_link_name_full: " << laser_link_name_full.str() << std::endl;
        
        if ( tfl_.waitForTransform(base_link_name_, laser_link_name_full.str(), ros::Time(0), ros::Duration(5.)) )
        {
            //look up for transform at TF
            tfl_.lookupTransform(base_link_name_, laser_link_name_full.str(), ros::Time(0), base_2_sensor);

            //rotation matrix
            rmat.setRotation(base_2_sensor.getRotation());
            
            //set homogeneous matrix:
            laser_mounting_points_.at(ii) << rmat.getRow(0).x(), rmat.getRow(0).y(), rmat.getRow(0).z(), base_2_sensor.getOrigin().x(),
                                             rmat.getRow(1).x(), rmat.getRow(1).y(), rmat.getRow(1).z(), base_2_sensor.getOrigin().y(),
                                             rmat.getRow(2).x(), rmat.getRow(2).y(), rmat.getRow(2).z(), base_2_sensor.getOrigin().z(),
                                             0,0,0,1;
                                             
            //debug
            if (verbose_)
            {
                std::cout << "H mat from " << base_link_name_ << " to " << laser_link_name_full.str() << ":" << std::endl;
                std::cout << laser_mounting_points_.at(ii) << std::endl; 
            }
        }
        else
        {
            std::cout << "WARNING: Homogeneous Transform from " << base_link_name_ << " to " << laser_link_name_full.str() << " not set. Identity assumed" << std::endl;
            laser_mounting_points_.at(ii) =  Eigen::Matrix4d::Identity();
        }
    }
        
        
    //4. INIT wolf
    //Create Wolf problem
    //wolf_problem_ = new WolfProblem()
    
    //create & add sensors & processors
    //odom_sensor_ = new SensorOdom2D
    //wolf_problem_->getHardwarePtr()->addSensor(odom_sensor_);
    //laser_sensor_(ii) = SensorLaser2D //for each laser
    //wolf_problem_->getHardwarePtr()->addSensor(laser_sensor_(ii)); //for each laser
    //laser_sensor_(ii)->addProcessor(new ProcessorLaser2D());
    
    //Create and init first frame
    //really need ?
    
    // init ceres
//     ceres_solver_options_.minimizer_type = ceres::TRUST_REGION; //ceres::LINE_SEARCH
//     ceres_solver_options_.max_line_search_step_contraction = 1e-3;
//     ceres_solver_options_.max_num_iterations = max_iterations_;
//     ceres_problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
//     ceres_problem_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
//     ceres_problem_options_.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
//     ceres_manager_ = new CeresManager(wofl_problem_, ceres_problem_options_);
    
    
    
    //print init info
    std::cout << std::endl << "SLAM Running with the following config: " << std::endl;
    std::cout << "\tverbose: " << verbose_ << std::endl;
    std::cout << "\trate: " << rate_ << std::endl;
    std::cout << "\tnum_lasers: " << num_lasers_ << std::endl;
    std::cout << "\tbase_link_name: " << base_link_name_ << std::endl;
}

SlamNode::~SlamNode()
{
    //
}

double SlamNode::getRate() const
{
    return rate_;
}

void SlamNode::process()
{
    //TODO
}

void SlamNode::publish()
{
    //fill messages
    
    //publish
    //marker_publisher_.publish(marker_msg_);
}

void SlamNode::odometryCallback(const nav_msgs::Odometry::ConstPtr& _msg)
{
    //TODO
}

void SlamNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr& _msg)
{
    //TODO
}

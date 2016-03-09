#ifndef SLAM_NODE_H
#define SLAM_NODE_H

//WOLF
#include "wolf/wolf.h"
#include "wolf/sensor_odom_2D.h"
#include "wolf/capture_odom_2D.h"
#include "wolf/sensor_laser_2D.h"
#include "wolf/capture_laser_2D.h"
#include "wolf/processor_laser_2D.h"
#include "wolf/wolf_problem.h"
#include "wolf/ceres_wrapper/ceres_manager.h"

//ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//ROS dynamic configure
#include <cargo_ants_slam/cargo_ants_slam_Config.h>

//Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector> //required to std::vector<Eigen::Matrix4d> . See http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html 

//std includes
#include <iostream>
#include <sstream>
#include <map>

/** \brief ROS Wrapper class to Wolf library objects
 *
 * ROS Wrapper class to Wolf library objects
 * 
 */
class SlamNode
{
    protected: 
        //ros node handle
        ros::NodeHandle nh_;
               
        //odometry subscriber
        ros::Subscriber odometry_subscriber_;
        
        //sensor subscribers
        std::vector<ros::Subscriber> laser_subscribers_;        
        
        //marker message and publisher
        visualization_msgs::MarkerArray marker_msg_;        
        ros::Publisher marker_publisher_;   
        
        //transform listener/boradcaster
        tf::TransformListener tfl_;
        tf::TransformBroadcaster tfb_;
        
        //verbose flag
        bool verbose_;         
        
        //whished update rate at the main loop, [hz]        
        double rate_;
                       
        //number of lasers
        unsigned int num_lasers_;
        
        //name of the base_link as published by tf
        std::string base_link_name_; 
        
        //sensor mounting points
        std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > laser_mounting_points_;
        
        //Associates laser_link_name with index in vectors: laser_subscribers_ and laser_mounting_points_
        std::map<std::string, unsigned int> laser_link_id_map_;
        
        //WOLF
        SensorOdom2D* odom_sensor_;
        std::vector<SensorLaser2D*> laser_sensor_;
        WolfProblem *wolf_problem_; 
        
        //Ceres
        CeresManager* ceres_manager_; //Wolf-Ceres interface
        ceres::Solver::Options ceres_solver_options_;
        ceres::Problem::Options ceres_problem_options_;
         
    public:
        /** \brief Default constructor
        * 
        * This constructor initializes the node. 
        * 
        */
        SlamNode();

        /** \brief Destructor
        * 
        * This destructor frees all necessary dynamic memory allocated within this class.
        */
        ~SlamNode();
        
        /** \brief Returns rate_
         * 
         * Returns rate_
         * 
         **/
        double getRate() const; 

        /** \brief Main process 
        * 
        * Main process flow
        * 
        **/
        void process();
                    
        /** \brief Fill output messages
        * 
        * Fills main output and debug messages
        */
        void publish();
        
    protected: 
        // odometry subscriber callback
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& _msg);
        
        // lidar subscriber callback
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& _msg);        
        
};
#endif



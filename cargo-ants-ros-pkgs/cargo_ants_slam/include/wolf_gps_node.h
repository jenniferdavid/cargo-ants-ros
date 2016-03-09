//
// Created by ptirindelli on 8/02/16.
//

#ifndef WOLF_ROS_WOLF_GPS_NODE_H
#define WOLF_ROS_WOLF_GPS_NODE_H


/**************************
 *      WOLF includes     *
 **************************/
#include "wolf/processor_gps.h"
#include "wolf/sensor_gps.h"
#include "wolf/sensor_odom_2D.h"
#include "wolf/wolf.h"
#include "wolf/wolf_problem.h"
#include "wolf/capture_motion.h"

/**************************
 *     CERES includes     *
 **************************/
#include "wolf/ceres_wrapper/ceres_manager.h"

/**************************
 *      raw_gps_utils     *
 **************************/
#include "raw_gps_utils/satellites_obs.h"

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>

#include "iri_common_drivers_msgs/SatellitePseudorangeArray.h"
#include "iri_common_drivers_msgs/NavSatFix_ecef.h"
#include <nav_msgs/Odometry.h>

/**************************
 *      STD includes      *
 **************************/
#include <iostream>
#include <queue>

class WolfGPSNode
{
public:
    WolfGPSNode(SensorGPS* _gps_sensor_ptr,
                const FrameStructure _frame_structure,
                SensorBase* _sensor_prior_ptr,
                const Eigen::VectorXs& _prior,
                const Eigen::MatrixXs& _prior_cov,
                const unsigned int& _trajectory_size = 10,
                const WolfScalar& _new_frame_elapsed_time = 0.1);
    virtual ~WolfGPSNode();

    void createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp);
    void createFrame(const TimeStamp& _time_stamp);
    void manageWindow();

    void obsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg);

protected:
    void initCeresManager();
    void updateWolfProblem();
    bool checkNewFrame(CaptureBase* new_capture);

protected:
    //Debug stuff
    bool ceresVerbose = false;
    ros::Time time_last_obs_;
    ros::Duration down_sampling_interval_ = ros::Duration(0, 5e8);

    /*
     * WOLF stuff
     */

    // GPS sensor
    SensorGPS* gps_sensor_ptr_;
    // Odometry sensor
    SensorOdom2D* odom_sensor_ptr_;

    // Wolf Problem
    WolfProblem* problem_;
    FrameStructure frame_structure_;

    //pointer to a sensor providing predictions
    SensorBase* sensor_prior_;

    //auxiliar/temporary iterators, frames and captures
    FrameBaseIter first_window_frame_;
    FrameBase* current_frame_;
    FrameBase* last_key_frame_;
    CaptureMotion* last_capture_relative_;
    CaptureMotion* second_last_capture_relative_;
    std::queue<CaptureBase*> new_captures_;

    //Manager parameters
    unsigned int trajectory_size_;
    WolfScalar new_frame_elapsed_time_;

    // Ceres
    ceres::Solver::Options ceres_options_;
    ceres::Problem::Options problem_options_;
    CeresManager* ceres_manager_;

    /*
     * ROS stuff
     */
    // Subscribers
    ros::Subscriber obs_sub_; // obs (measurements) subscriber
    // Publishers
    ros::Publisher wolf_fix_pub_;
    // ROS node handle
    ros::NodeHandle nh_;

    //Odometry callback
    ros::Time last_odom_stamp_;
    ros::Subscriber odometry_subscriber_;
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);

};


#endif //WOLF_ROS_WOLF_GPS_NODE_H

#include <wolf/sensor_gps.h>//
// Created by ptirindelli on 8/02/16.
//

#include "wolf_gps_node.h"


WolfGPSNode::WolfGPSNode(SensorGPS* _gps_sensor_ptr,
                         const FrameStructure _frame_structure,
                         SensorBase* _sensor_prior_ptr,
                         const Eigen::VectorXs& _prior,
                         const Eigen::MatrixXs& _prior_cov,
                         const unsigned int& _trajectory_size,
                         const WolfScalar& _new_frame_elapsed_time) :
        nh_(ros::this_node::getName()),
        gps_sensor_ptr_(_gps_sensor_ptr),
        problem_(new WolfProblem()),
        frame_structure_(_frame_structure),
        sensor_prior_(_sensor_prior_ptr),
        current_frame_(nullptr),
        last_key_frame_(nullptr),
        last_capture_relative_(nullptr),
        trajectory_size_(_trajectory_size),
        new_frame_elapsed_time_(_new_frame_elapsed_time),
        last_odom_stamp_(0)
{
    if (_frame_structure == PO_2D)
        assert( _prior.size() == 3 &&
                _prior_cov.cols() == 3 &&
                _prior_cov.rows() == 3 &&
                "Wrong init_frame state vector or covariance matrix size");
    else
        assert( _prior.size() == 7 &&
                _prior_cov.cols() == 7 &&
                _prior_cov.rows() == 7 &&
                "Wrong init_frame state vector or covariance matrix size");

    std::cout << "WolfGPSNode::WolfGPSNode(...) -- constructor\n";

    // init wolf odom sensor
    WolfScalar odom_std[2];
    odom_std[0] = 0.2; // odometry_translational_std
    odom_std[1] = 0.2; // odometry_rotational_std
    odom_sensor_ptr_ = new SensorOdom2D(new StateBlock(Eigen::Vector2s::Zero()), new StateBlock(Eigen::Vector1s::Zero()), odom_std[0], odom_std[1]);//both arguments initialized on top


    // Add processor to gps sensor
    gps_sensor_ptr_->addProcessor(new ProcessorGPS());
    //std::cout << "WolfGPSNode::WolfGPSNode(...) -- processor added \n";


    // Initial frame
    //std::cout << " creating first frame\n";
    createFrame(_prior, TimeStamp(0));
    first_window_frame_ = problem_->getTrajectoryPtr()->getFrameListPtr()->begin();
    //std::cout << " first_window_frame_" << std::endl;

    // Initialize ceres manager
    initCeresManager();

    // Adding the sensor
    problem_->getHardwarePtr()->addSensor(gps_sensor_ptr_);

    // Subscriber
    obs_sub_ = nh_.subscribe("/sat_pseudoranges", 1000, &WolfGPSNode::obsCallback, this);
    // Publisher
    wolf_fix_pub_ = nh_.advertise<iri_common_drivers_msgs::NavSatFix_ecef>("/wolf_fix", 5000);

}

WolfGPSNode::~WolfGPSNode()
{
    std::cout << std::endl << " ========= WolfGPSNode DESTRUCTOR (should not crash) =============" << std::endl;
    problem_->destruct();

    //TODO check if is correct to put this here
    std::cout << std::endl << " ========= destroying ceres manager (now seg fault) =============" << std::endl;
    delete ceres_manager_;
}

void WolfGPSNode::createFrame(const Eigen::VectorXs& _frame_state, const TimeStamp& _time_stamp)
{
//    std::cout << "creating new frame..." << std::endl;

    // current frame -> KEYFRAME
    last_key_frame_ = current_frame_;

    // ---------------------- CREATE NEW FRAME ---------------------
    // Create frame
    switch ( frame_structure_)
    {
        case PO_2D:
        {
            assert( _frame_state.size() == 3 && "Wrong init_frame state vector or covariance matrix size");

            problem_->getTrajectoryPtr()->addFrame(new FrameBase(_time_stamp,
                                                                 new StateBlock(_frame_state.head(2)),
                                                                 new StateBlock(_frame_state.tail(1))));
            break;
        }
        case PO_3D:
        {
            assert( _frame_state.size() == 7 && "Wrong init_frame state vector or covariance matrix size");

            problem_->getTrajectoryPtr()->addFrame(new FrameBase(_time_stamp,
                                                                 new StateBlock(_frame_state.head(3)),
                                                                 new StateBlock(_frame_state.tail(4),ST_QUATERNION)));
            break;
        }
        default:
        {
            assert( "Unknown frame structure");
        }
    }

//    std::cout << "frame created" << std::endl;

    // Store new current frame
    current_frame_ = problem_->getLastFramePtr();
//    std::cout << "current_frame_" << std::endl;

    // ---------------------- KEY FRAME ---------------------
    if (last_key_frame_ != nullptr)
    {
        //std::cout << "Processing last frame non-odometry captures " << current_frame_->getCaptureListPtr()->size() << std::endl;
        for (auto capture_it = last_key_frame_->getCaptureListPtr()->begin(); capture_it != last_key_frame_->getCaptureListPtr()->end(); capture_it++)
            if ((*capture_it)->getSensorPtr() != sensor_prior_)
            {
                //std::cout << "processing capture " << (*capture_it)->nodeId() << std::endl;
                (*capture_it)->process();
            }


    }
    //std::cout << "Last key frame non-odometry captures processed" << std::endl;

    // ---------------------- MANAGE WINDOW OF POSES ---------------------
    manageWindow();
    //std::cout << "new frame created" << std::endl;
}

void WolfGPSNode::manageWindow()
{
    //std::cout << "managing window..." << std::endl;
    // WINDOW of FRAMES (remove or fix old frames)
    if (problem_->getTrajectoryPtr()->getFrameListPtr()->size() > trajectory_size_+1)
    {
        //std::cout << "first_window_frame_ " << (*first_window_frame_)->nodeId() << std::endl;
        //problem_->getTrajectoryPtr()->removeFrame(problem_->getTrajectoryPtr()->getFrameListPtr()->begin());
        (*first_window_frame_)->fix();
        first_window_frame_++;
    }
    //std::cout << "window managed" << std::endl;
}

void WolfGPSNode::createFrame(const TimeStamp& _time_stamp)
{
    //std::cout << "creating new frame from prior..." << std::endl;
    createFrame(Eigen::Vector7s::Zero(), _time_stamp);
}

void WolfGPSNode::initCeresManager()
{
    ceres_options_.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options_.max_line_search_step_contraction = 1e-3;
    ceres_options_.max_num_iterations = 1e4;
    problem_options_.cost_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options_.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options_.local_parameterization_ownership = ceres::TAKE_OWNERSHIP;

    ceres_manager_ = new CeresManager(problem_, problem_options_);

}

/*
 * TODO:
 * when a new observation arrive, save it and activate a flag "new_data"
 * and move the elaboration outside the callback:
 * something is monitoring the flag and when is on it start working
 */
void WolfGPSNode::obsCallback(const iri_common_drivers_msgs::SatellitePseudorangeArray::ConstPtr& msg)
{
    //downsampling of the observations received
    if(msg->time_ros < time_last_obs_ + down_sampling_interval_)
    {
        //std::cout << "discarding obs\n";
        return;
    }
    time_last_obs_ = msg->time_ros;

    std::cout << "WolfGPSNode::obsCallback\n";

    //std::cout << "------MSG: found " << msg->measurements.size() << " sats\n";

    std::cout << "creating CaptureGPS\n";
    // Create CaptureGPS
    rawgpsutils::SatellitesObs obs;
    obs.time_ros_sec_ = msg->time_ros.sec;
    obs.time_ros_nsec_ = msg->time_ros.nsec;
    obs.time_gps_wnc_ = msg->time_gps_wnc;
    obs.time_gps_tow_ = msg->time_gps_tow;
    for (int i = 0; i < msg->measurements.size(); ++i)
    {
        rawgpsutils::PrMeasurement sat(msg->measurements[i].sat_id,
                                       msg->measurements[i].pseudorange,
                                       msg->measurements[i].x,
                                       msg->measurements[i].y,
                                       msg->measurements[i].z,
                                       msg->measurements[i].v_x,
                                       msg->measurements[i].v_y,
                                       msg->measurements[i].v_z);

        obs.measurements_.push_back(sat);
    }
    //std::cout << "------OBS: found " << obs.measurements_.size() << " sats\n";

    TimeStamp time_stamp(obs.time_ros_sec_, obs.time_ros_nsec_);

    CaptureGPS* cpt_ptr_ = new CaptureGPS(time_stamp, gps_sensor_ptr_, obs);

    // Add capture
    std::cout << "creating CaptureGPS\n";
    new_captures_.push(cpt_ptr_);
    std::cout << "capture added" << std::endl;

    // update wolf tree
    updateWolfProblem();
    std::cout << "wolf Problem updated" << std::endl;

    ceres_manager_->update();
    std::cout << "ceres manager updated" << std::endl;

    ceres::Solver::Summary summary;

    summary = ceres_manager_->solve(ceres_options_);
    if(ceresVerbose)
        std::cout << summary.FullReport() << std::endl;

    std::cout << std::setprecision(12);
    std::cout << "\n~~~~ RESULTS ~~~~\n";
    //std::cout << "Vehicle pose " << wolf_manager_->getVehiclePose().transpose() << std::endl;
    //std::cout << "getInitVehicleP " << gps_sensor_ptr_->getInitVehiclePPtr()->getVector().transpose() << std::endl;

    std::cout << "|\tgetPPtr " << gps_sensor_ptr_->getPPtr()->getVector().transpose() << std::endl;// position of the vehicle's frame with respect to the initial pos frame
    std::cout << "|\tgetOPtr " << gps_sensor_ptr_->getOPtr()->getVector().transpose() << std::endl;// orientation of the vehicle's frame
    std::cout << "|\tgetIntrinsicPtr " << gps_sensor_ptr_->getIntrinsicPtr()->getVector().transpose() << std::endl;//intrinsic parameter  = receiver time bias
    std::cout << "|\tgetInitVehiclePPtr " << gps_sensor_ptr_->getInitVehiclePPtr()->getVector().transpose() << std::endl;// initial vehicle position (ecef)
    std::cout << "|\tgetInitVehicleOPtr " << gps_sensor_ptr_->getInitVehicleOPtr()->getVector().transpose() << std::endl;// initial vehicle orientation (ecef)
    //                        getSensorPPtr(), // position of the sensor (gps antenna) with respect to the vehicle frame
    //                        orientation of antenna is not needed, because omnidirectional
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n";


    // Publishing results
    iri_common_drivers_msgs::NavSatFix_ecef wolf_fix;
    wolf_fix.x = gps_sensor_ptr_->getPPtr()->getVector()[0];
    wolf_fix.y = gps_sensor_ptr_->getPPtr()->getVector()[1];
    wolf_fix.z = gps_sensor_ptr_->getPPtr()->getVector()[2];
    wolf_fix_pub_.publish(wolf_fix);

}

void WolfGPSNode::updateWolfProblem()
{
    //std::cout << "updating..." << std::endl;
    while (!new_captures_.empty())
    {
        // EXTRACT NEW CAPTURE
        CaptureBase* new_capture = new_captures_.front();
        new_captures_.pop();
        // OVERWRITE CURRENT STAMP
        current_frame_->setTimeStamp(new_capture->getTimeStamp());
        // INITIALIZE FIRST FRAME STAMP
        if (last_key_frame_ != nullptr && last_key_frame_->getTimeStamp().get() == 0)
            last_key_frame_->setTimeStamp(new_capture->getTimeStamp());
        // NEW KEY FRAME ?
        if (checkNewFrame(new_capture))
            createFrame(new_capture->getTimeStamp());
        // ODOMETRY SENSOR
        if (new_capture->getSensorPtr() == sensor_prior_)
        {
            std::cout << "adding odometry capture..." << new_capture->nodeId() << std::endl;

            // ADD/INTEGRATE NEW ODOMETRY TO THE LAST FRAME
            last_capture_relative_->integrateCapture((CaptureMotion*) (new_capture));
            current_frame_->setState(last_capture_relative_->computeFramePose(new_capture->getTimeStamp()));
            current_frame_->setTimeStamp(new_capture->getTimeStamp());
            delete new_capture;
        }
        else
        {
            std::cout << "adding not odometry capture..." << new_capture->nodeId() << std::endl;

            // ADD CAPTURE TO THE CURRENT FRAME (or substitute the same sensor previous capture)
            //std::cout << "searching repeated capture..." << new_capture->nodeId() << std::endl;
            CaptureBaseIter repeated_capture_it = current_frame_->hasCaptureOf(new_capture->getSensorPtr());

            if (repeated_capture_it != current_frame_->getCaptureListPtr()->end()) // repeated capture
            {
                //std::cout << "repeated capture, keeping new capture" << new_capture->nodeId() << std::endl;
                current_frame_->removeCapture(repeated_capture_it);
                current_frame_->addCapture(new_capture);
            }
            else
            {
                //std::cout << "not repeated, adding capture..." << new_capture->nodeId() << std::endl;
                current_frame_->addCapture(new_capture);
            }
        }
    }
    //std::cout << "updated" << std::endl;
}

bool WolfGPSNode::checkNewFrame(CaptureBase* new_capture)
{
    //std::cout << "checking if new frame..." << std::endl;
    // TODO: not only time, depending on the sensor...
    //std::cout << new_capture->getTimeStamp().get() << std::endl;
    return new_capture->getTimeStamp().get() - (last_key_frame_ == nullptr ? 0 : last_key_frame_->getTimeStamp().get()) > new_frame_elapsed_time_;
}
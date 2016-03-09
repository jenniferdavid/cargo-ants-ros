//ros dependencies
#include "ros/ros.h"
#include "ros/time.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

//C includes for sleep, time and main args
#include "unistd.h"

//std includes
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//Wolf includes
#include "wolf/wolf_manager.h"
#include "wolf/capture_void.h"
#include "wolf/ceres_wrapper/ceres_manager.h"

// Ceres includes
#include "ceres/ceres.h"
#include "glog/logging.h"

class FrameCostFunctor
{
    protected:
        Eigen::Map<Eigen::Vector2s> fitted_point_;
        Eigen::Map<Eigen::Vector2s> reference_point_;

    public:
        FrameCostFunctor(WolfScalar* _fitted_point_ptr, WolfScalar* _reference_point_ptr) :
            fitted_point_(_fitted_point_ptr),
            reference_point_(_reference_point_ptr)
    {

    }
    template <typename T>
    bool operator()(const T* const correction, T* residual) const
    {
        residual[0] = ( T(fitted_point_(0))*cos(correction[2]) - T(fitted_point_(1))*sin(correction[2]) + correction[0] - T(reference_point_(0)) ) *
                      ( T(fitted_point_(0))*cos(correction[2]) - T(fitted_point_(1))*sin(correction[2]) + correction[0] - T(reference_point_(0)) ) +
                      ( T(fitted_point_(0))*sin(correction[2]) + T(fitted_point_(1))*cos(correction[2]) + correction[1] - T(reference_point_(1)) ) *
                      ( T(fitted_point_(0))*sin(correction[2]) + T(fitted_point_(1))*cos(correction[2]) + correction[1] - T(reference_point_(1)) );
        return true;
    }
};

class TrajectoryICP
{
    protected:
        ceres::Solver::Options solver_options_;
        ceres::Problem* ceres_problem_;
        Eigen::Vector3s correction_;
        std::vector<Eigen::Map<Eigen::Vector3s>> frames_poses_;

    public:
        TrajectoryICP(WolfProblem* fitted_problem, WolfProblem* reference_problem) :
            correction_(Eigen::Vector3d::Zero())
        {
            solver_options_.minimizer_progress_to_stdout = false;
            solver_options_.minimizer_type = ceres::TRUST_REGION;
            solver_options_.line_search_direction_type = ceres::LBFGS;
            solver_options_.max_num_iterations = 100;

            ceres::Problem::Options problem_options;
            problem_options.cost_function_ownership = ceres::TAKE_OWNERSHIP;

            ceres_problem_ = new ceres::Problem(problem_options);

            // add constraints for each pair of frames
            auto fitted_frame_it = fitted_problem->getTrajectoryPtr()->getFrameListPtr()->begin();
            for (auto reference_frame_it  = reference_problem->getTrajectoryPtr()->getFrameListPtr()->begin();
                      reference_frame_it != reference_problem->getTrajectoryPtr()->getFrameListPtr()->end();
                      reference_frame_it++, fitted_frame_it++)
            {
                frames_poses_.push_back(Eigen::Map<Eigen::Vector3s>((*fitted_frame_it)->getPPtr()->getPtr()));
                ceres_problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<FrameCostFunctor,1,3>(new FrameCostFunctor((*fitted_frame_it)->getPPtr()->getPtr(),
                                                                                                                            (*reference_frame_it)->getPPtr()->getPtr())),
                                                                                                       nullptr,
                                                                                                       correction_.data());
            }
        }

        ~TrajectoryICP()
        {
            delete ceres_problem_;
        }

        double fit()
        {
            // Reset correction
            correction_ = Eigen::Vector3d::Zero();

            // Ceres Solver
            ceres::Solver::Summary summary;
            ceres::Solve(solver_options_, ceres_problem_, &summary);

            // apply correction
            Eigen::Matrix3s R;
            R << cos(correction_(2)),-sin(correction_(2)), 0,
                 sin(correction_(2)), cos(correction_(2)), 0,
                 0,                   0,                   1;
            for (auto pose_it : frames_poses_ )
                pose_it = R * pose_it + correction_;

            return summary.final_cost;
        }
};

class WolfPrunning
{
    protected:

        double t_sigma_ceres, t_sigma_manual, t_ig, t_prunning, t_solve_full, t_solve_prun;
        std::map<FrameBase*, unsigned int> frame_ptr_2_index_prun;
        std::map<unsigned int, FrameBase*> index_2_frame_ptr_full;
        std::map<unsigned int, FrameBase*> index_2_frame_ptr_prun;
        std::vector<Eigen::VectorXs> priors_vector;

        // Wolf problem
        WolfProblem* wolf_problem_full;
        WolfProblem* wolf_problem_prun;
        SensorBase* sensor_ptr;

        // prunning
        std::list<ConstraintBase*> ordered_ctr_ptr;
        std::list<WolfScalar> ordered_ig;
        Eigen::MatrixXs Sigma_11, Sigma_12, Sigma_13, Sigma_14,
                        Sigma_22, Sigma_23, Sigma_24,
                        Sigma_33, Sigma_34,
                        Sigma_44, Sigma_z;
        std::vector<Eigen::MatrixXs> jacobians;
        double ig_threshold;
        unsigned int N_removable_nodes;

        // fitting
        TrajectoryICP* trajectory_icp;
        double fitting_error;

        // Ceres wrapper
        ceres::Solver::Summary summary_full, summary_prun;
        ceres::Solver::Options ceres_options;
        ceres::Problem::Options problem_options;
        CeresManager* ceres_manager_full;
        CeresManager* ceres_manager_prun;
        SensorBase* prior_sensor_ptr;
        SensorBase* edge_sensor_ptr;

    public:

        WolfPrunning(const std::string& file_path, unsigned int max_vertex) :
            wolf_problem_full(new WolfProblem()),
            wolf_problem_prun(new WolfProblem()),
            sensor_ptr(new SensorBase(ODOM_2D, new StateBlock(Eigen::VectorXs::Zero(2)), new StateBlock(Eigen::VectorXs::Zero(1)), new StateBlock(Eigen::VectorXs::Zero(2)), 2)),
            Sigma_11(2,2),
            Sigma_12(2,1),
            Sigma_13(2,2),
            Sigma_14(2,1),
            Sigma_22(1,1),
            Sigma_23(1,2),
            Sigma_24(1,1),
            Sigma_33(2,2),
            Sigma_34(2,1),
            Sigma_44(1,1),
            Sigma_z(3,3),
            jacobians({Eigen::MatrixXs::Zero(3,2), Eigen::MatrixXs::Zero(3,1), Eigen::MatrixXs::Zero(3,2), Eigen::MatrixXs::Zero(3,1)}),
            trajectory_icp(nullptr),
            ceres_manager_full(new CeresManager(wolf_problem_full, problem_options)),
            ceres_manager_prun(new CeresManager(wolf_problem_prun, problem_options)),
            prior_sensor_ptr(new SensorBase(ABSOLUTE_POSE, nullptr, nullptr, nullptr, 0)),
            edge_sensor_ptr(new SensorBase(ODOM_2D, new StateBlock(Eigen::VectorXs::Zero(2)), new StateBlock(Eigen::VectorXs::Zero(1)), new StateBlock(Eigen::VectorXs::Zero(2)), 2))
        {
            ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
            ceres_options.max_line_search_step_contraction = 1e-3;
            ceres_options.max_num_iterations = 1e4;
            problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
            problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
            problem_options.local_parameterization_ownership = ceres::TAKE_OWNERSHIP;

            // LOAD GRAPH
            if (max_vertex == 0) max_vertex = 1e6;
            std::cout << "loading graph..." << std::endl;
            loadGraphFile(file_path, max_vertex);
            std::cout << "loaded!" << std::endl;

            // ADD PRIOR
            FrameBase* first_frame_full = wolf_problem_full->getTrajectoryPtr()->getFrameListPtr()->front();
            FrameBase* first_frame_prun = wolf_problem_prun->getTrajectoryPtr()->getFrameListPtr()->front();
            CaptureFix* initial_covariance_full = new CaptureFix(TimeStamp(0), prior_sensor_ptr, first_frame_full->getState(), Eigen::Matrix3s::Identity() * 0.01);
            CaptureFix* initial_covariance_prun = new CaptureFix(TimeStamp(0), prior_sensor_ptr, first_frame_prun->getState(), Eigen::Matrix3s::Identity() * 0.01);
            first_frame_full->addCapture(initial_covariance_full);
            first_frame_prun->addCapture(initial_covariance_prun);
            initial_covariance_full->process();
            initial_covariance_prun->process();
            //std::cout << "initial covariance: constraint " << initial_covariance_prun->getFeatureListPtr()->front()->getConstraintFromListPtr()->front()->nodeId() << std::endl << initial_covariance_prun->getFeatureListPtr()->front()->getMeasurementCovariance() << std::endl;

            // BUILD ICP
            trajectory_icp = new TrajectoryICP(wolf_problem_prun, wolf_problem_full);

            // BUILD SOLVER PROBLEM
            std::cout << "updating ceres..." << std::endl;
            ceres_manager_full->update();
            ceres_manager_prun->update();
            std::cout << "updated!" << std::endl;

            // COMPUTE COVARIANCES
            // whole covariance computation
            std::cout << "ceres is computing covariances..." << std::endl;
            clock_t t1 = clock();
            ceres_manager_prun->computeCovariances(ALL);//ALL_MARGINALS
            t_sigma_ceres = ((double) clock() - t1) / CLOCKS_PER_SEC;
            std::cout << "computed!" << std::endl;

            // INFORMATION GAIN
            std::cout << "information gains..." << std::endl;
            t1 = clock();
            ConstraintBaseList constraints;
            wolf_problem_prun->getTrajectoryPtr()->getConstraintList(constraints);
            for (auto c_it=constraints.begin(); c_it!=constraints.end(); c_it++)
                computeInformationGain(*c_it);

            t_ig = ((double) clock() - t1) / CLOCKS_PER_SEC;
            std::cout << "computed and ordered!" << std::endl;
            //for (auto it : ordered_ig)
            //    std::cout << it << " ";
            //std::cout << std::endl;
        }

        ~WolfPrunning()
        {
            delete prior_sensor_ptr;
            delete edge_sensor_ptr;
            delete wolf_problem_full;
            delete wolf_problem_prun;
            std::cout << "wolf problems deleted!" << std::endl;
            delete ceres_manager_full;
            delete ceres_manager_prun;
            std::cout << "ceres managers deleted!" << std::endl;
        }

        // load graph from file
        bool loadGraphFile(const std::string& file_path, const int MAX_VERTEX)
        {
            std::ifstream offLineFile;
            Eigen::Vector3s edge_vector, vertex_pose;
            Eigen::Matrix3s edge_information;
            FrameBase* vertex_frame_ptr_full;
            FrameBase* vertex_frame_ptr_prun;
            frame_ptr_2_index_prun.clear();
            unsigned int state_size = 0;

            offLineFile.open(file_path.c_str(), std::ifstream::in);
            if (offLineFile.is_open())
            {
                std::string buffer;
                unsigned int j = 0;
                // Line by line
                while (std::getline(offLineFile, buffer))
                {
                    //std::cout << "new line:" << buffer << std::endl;
                    std::string bNum;
                    unsigned int i = 0;

                    // VERTEX
                    if (buffer.at(0) == 'V')
                    {
                        while (buffer.at(i) != ' ') i++; //skip rest of VERTEX word
                        while (buffer.at(i) == ' ') i++; //skip white spaces
                        //vertex index
                        while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                        unsigned int vertex_index = atoi(bNum.c_str());
                        bNum.clear();

                        if (vertex_index <= MAX_VERTEX+1)
                        {
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // vertex pose
                            // x
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            vertex_pose(0) = atof(bNum.c_str());
                            bNum.clear();
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // y
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            vertex_pose(1) = atof(bNum.c_str());
                            bNum.clear();
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // theta
                            while (i < buffer.size() && buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            vertex_pose(2) = atof(bNum.c_str());
                            bNum.clear();
                            // add frame to problem
                            vertex_frame_ptr_full = new FrameBase(TimeStamp(0), new StateBlock(vertex_pose.head(2)), new StateBlock(vertex_pose.tail(1)));
                            vertex_frame_ptr_prun = new FrameBase(TimeStamp(0), new StateBlock(vertex_pose.head(2)), new StateBlock(vertex_pose.tail(1)));
                            wolf_problem_full->getTrajectoryPtr()->addFrame(vertex_frame_ptr_full);
                            wolf_problem_prun->getTrajectoryPtr()->addFrame(vertex_frame_ptr_prun);
                            // store
                            index_2_frame_ptr_full[vertex_index] = vertex_frame_ptr_full;
                            index_2_frame_ptr_prun[vertex_index] = vertex_frame_ptr_prun;
                            frame_ptr_2_index_prun[vertex_frame_ptr_prun] = vertex_index;
                            priors_vector.push_back(vertex_pose);
                            // increment state size
                            state_size +=3;
                            //std::cout << "Added vertex! index: " << vertex_index << " nodeId: " << vertex_frame_ptr_prun->nodeId() << std::endl << "pose: " << vertex_pose.transpose() << std::endl;
                        }
                    }
                    // EDGE
                    else if (buffer.at(0) == 'E')
                    {
                        while (buffer.at(i) != ' ') i++; //skip rest of EDGE word
                        while (buffer.at(i) == ' ') i++; //skip white spaces
                        //from
                        while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                        unsigned int edge_old = atoi(bNum.c_str());
                        bNum.clear();
                        while (buffer.at(i) == ' ') i++; //skip white spaces
                        //to index
                        while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                        unsigned int edge_new = atoi(bNum.c_str());
                        bNum.clear();

                        if (edge_new <= MAX_VERTEX+1 && edge_old <= MAX_VERTEX+1 )
                        {
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // edge vector
                            // x
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            edge_vector(0) = atof(bNum.c_str());
                            bNum.clear();
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // y
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            edge_vector(1) = atof(bNum.c_str());
                            bNum.clear();
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // theta
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            edge_vector(2) = atof(bNum.c_str());
                            bNum.clear();
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // edge covariance
                            // xx
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            edge_information(0,0) = atof(bNum.c_str());
                            bNum.clear();
                            //skip white spaces
                            while (buffer.at(i) == ' ') i++;
                            // xy
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            edge_information(0,1) = atof(bNum.c_str());
                            edge_information(1,0) = atof(bNum.c_str());
                            bNum.clear();
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // yy
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            edge_information(1,1) = atof(bNum.c_str());
                            bNum.clear();
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // thetatheta
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            edge_information(2,2) = atof(bNum.c_str());
                            bNum.clear();
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // xtheta
                            while (buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            edge_information(0,2) = atof(bNum.c_str());
                            edge_information(2,0) = atof(bNum.c_str());
                            bNum.clear();
                            while (buffer.at(i) == ' ') i++; //skip white spaces
                            // ytheta
                            while (i < buffer.size() && buffer.at(i) != ' ') bNum.push_back(buffer.at(i++));
                            edge_information(1,2) = atof(bNum.c_str());
                            edge_information(2,1) = atof(bNum.c_str());
                            bNum.clear();
                            // add capture, feature and constraint to problem
                            addEdgeToProblem(edge_vector, edge_information, index_2_frame_ptr_full[edge_old], index_2_frame_ptr_full[edge_new]);
                            addEdgeToProblem(edge_vector, edge_information, index_2_frame_ptr_prun[edge_old], index_2_frame_ptr_prun[edge_new]);
                            //std::cout << "Added edge! " << constraint_ptr_prun->nodeId() << " from vertex " << constraint_ptr_prun->getCapturePtr()->getFramePtr()->nodeId() << " to " << constraint_ptr_prun->getFrameToPtr()->nodeId() << std::endl;
                            //std::cout << "vector " << constraint_ptr_prun->getMeasurement().transpose() << std::endl;
                            //std::cout << "information " << std::endl << edge_information << std::endl;
                            //std::cout << "covariance " << std::endl << constraint_ptr_prun->getMeasurementCovariance() << std::endl;
                            j++;
                        }
                    }
                    else
                        assert("unknown line");
                }
                printf("\nGraph loaded!\n");
                return true;
            }
            else
            {
                printf("\nError opening file!\n");
                return false;
            }
        }

        void addEdgeToProblem(const Eigen::Vector3s& edge_vector, const Eigen::Matrix3s& edge_information, FrameBase* frame_old_ptr, FrameBase* frame_new_ptr)
        {
            FeatureBase* feature_ptr = new FeatureBase(edge_vector, edge_information.inverse());
            CaptureVoid* capture_ptr = new CaptureVoid(TimeStamp(0), edge_sensor_ptr);
            frame_new_ptr->addCapture(capture_ptr);
            capture_ptr->addFeature(feature_ptr);
            ConstraintOdom2DAnalytic* constraint_ptr = new ConstraintOdom2DAnalytic(feature_ptr, frame_old_ptr);
            feature_ptr->addConstraintFrom(constraint_ptr);
        }

        void wolfToFramesMarker(visualization_msgs::Marker& frames, WolfProblem* wolf_problem_ptr)
        {
            frames.points.clear();
            frames.header.stamp = ros::Time::now();
            geometry_msgs::Point point;
            for (auto fr_it = wolf_problem_ptr->getTrajectoryPtr()->getFrameListPtr()->begin();
                      fr_it != wolf_problem_ptr->getTrajectoryPtr()->getFrameListPtr()->end();
                      fr_it++) //runs the list of frames
            {
                point.x = *(*fr_it)->getPPtr()->getPtr();
                point.y = *((*fr_it)->getPPtr()->getPtr()+1);
                point.z = 0.;
                frames.points.push_back(point);
            }
        }

        void wolfToConstraintsMarker(visualization_msgs::Marker& constraints, WolfProblem* wolf_problem_ptr)
        {
            constraints.points.clear();
            constraints.header.stamp = ros::Time::now();
            geometry_msgs::Point point;

            ConstraintBaseList ctr_list;
            wolf_problem_ptr->getTrajectoryPtr()->getConstraintList(ctr_list);
            for (auto ctr : ctr_list) //runs the list of frames
            {
                if (ctr->getCategory() != CTR_FRAME)
                    continue;

                point.x = *ctr->getCapturePtr()->getFramePtr()->getPPtr()->getPtr();
                point.y = *(ctr->getCapturePtr()->getFramePtr()->getPPtr()->getPtr()+1);
                point.z = 0.;
                constraints.points.push_back(point);

                point.x = *ctr->getFrameOtherPtr()->getPPtr()->getPtr();
                point.y = *(ctr->getFrameOtherPtr()->getPPtr()->getPtr()+1);
                point.z = 0.;
                constraints.points.push_back(point);
            }
        }

        void computeInformationGain(ConstraintBase* ctr_ptr)
        {
            if (ctr_ptr->getCategory() == CTR_FRAME)
            {
                // Measurement covariance
                Sigma_z = ctr_ptr->getFeaturePtr()->getMeasurementCovariance();

                StateBlock* FrameOtherPPtr = ctr_ptr->getFrameOtherPtr()->getPPtr();
                StateBlock* FrameOtherOPtr = ctr_ptr->getFrameOtherPtr()->getOPtr();
                StateBlock* FramePPtr = ctr_ptr->getCapturePtr()->getFramePtr()->getPPtr();
                StateBlock* FrameOPtr = ctr_ptr->getCapturePtr()->getFramePtr()->getOPtr();

                // NEW WAY
                // State covariance
                wolf_problem_prun->getCovarianceBlock(FrameOtherPPtr, FrameOtherPPtr, Sigma_11);
                wolf_problem_prun->getCovarianceBlock(FrameOtherPPtr, FrameOtherOPtr, Sigma_12);
                wolf_problem_prun->getCovarianceBlock(FrameOtherPPtr, FramePPtr, Sigma_13);
                wolf_problem_prun->getCovarianceBlock(FrameOtherPPtr, FrameOPtr, Sigma_14);

                wolf_problem_prun->getCovarianceBlock(FrameOtherOPtr, FrameOtherOPtr, Sigma_22);
                wolf_problem_prun->getCovarianceBlock(FrameOtherOPtr, FramePPtr, Sigma_23);
                wolf_problem_prun->getCovarianceBlock(FrameOtherOPtr, FrameOPtr, Sigma_24);

                wolf_problem_prun->getCovarianceBlock(FramePPtr, FramePPtr, Sigma_33);
                wolf_problem_prun->getCovarianceBlock(FramePPtr, FrameOPtr, Sigma_34);

                wolf_problem_prun->getCovarianceBlock(FrameOPtr, FrameOPtr, Sigma_44);

                // jacobians
                ((ConstraintAnalytic*)ctr_ptr)->evaluatePureJacobians(jacobians);
                Eigen::MatrixXs& J1 = jacobians[0];
                Eigen::MatrixXs& J2 = jacobians[1];
                Eigen::MatrixXs& J3 = jacobians[2];
                Eigen::MatrixXs& J4 = jacobians[3];

                // Information gain
                WolfScalar IG = 0.5 * log( Sigma_z.determinant() /
                                          (Sigma_z - (J1 * Sigma_11 * J1.transpose() +
                                                      J1 * Sigma_12 * J2.transpose() +
                                                      J1 * Sigma_13 * J3.transpose() +
                                                      J1 * Sigma_14 * J4.transpose() +
                                                      J2 * Sigma_12.transpose() * J1.transpose() +
                                                      J2 * Sigma_22 * J2.transpose() +
                                                      J2 * Sigma_23 * J3.transpose() +
                                                      J2 * Sigma_24 * J4.transpose() +
                                                      J3 * Sigma_13.transpose() * J1.transpose() +
                                                      J3 * Sigma_23.transpose() * J2.transpose() +
                                                      J3 * Sigma_33 * J3.transpose() +
                                                      J3 * Sigma_34 * J4.transpose() +
                                                      J4 * Sigma_14.transpose() * J1.transpose() +
                                                      J4 * Sigma_24.transpose() * J2.transpose() +
                                                      J4 * Sigma_34.transpose() * J3.transpose() +
                                                      J4 * Sigma_44 * J4.transpose()) ).determinant() );

                //std::cout << "IG = " << IG << std::endl;

                // store links ordered (ascendent information gain)
                if (IG > 0 && !std::isnan(IG))
                {
                    // Store as a candidate to be prunned, ordered by information gain
                    auto ordered_ctr_it = ordered_ctr_ptr.begin();
                    for (auto ordered_ig_it = ordered_ig.begin(); ordered_ig_it != ordered_ig.end(); ordered_ig_it++, ordered_ctr_it++ )
                        if (IG < (*ordered_ig_it))
                        {
                            ordered_ig.insert(ordered_ig_it, IG);
                            ordered_ctr_ptr.insert(ordered_ctr_it, ctr_ptr);
                            break;
                        }
                    ordered_ig.push_back(IG);
                    ordered_ctr_ptr.push_back(ctr_ptr);
                }
            }
        }

        bool isRemovable(ConstraintBase* ctr_ptr)
        {
            FrameBase* frame_own = ctr_ptr->getCapturePtr()->getFramePtr();
            FrameBase* frame_other = ctr_ptr->getFrameOtherPtr();

            // Is it connected to another inactive constraint?
            std::list<ConstraintBase*> connected_constraints_list = (*frame_own->getConstraintToListPtr());
            frame_own->getConstraintList(connected_constraints_list);
            connected_constraints_list.insert (connected_constraints_list.end(),frame_other->getConstraintToListPtr()->begin(),frame_other->getConstraintToListPtr()->end());
            frame_other->getConstraintList(connected_constraints_list);

            for (auto c_it : connected_constraints_list)
                if ( c_it->getStatus() == CTR_INACTIVE)
                    return false;

            // Would it destroy graph integrity?
            //std::cout << "Checking graph integrity if removing..." << std::endl;
            std::vector<bool> frame_connected(wolf_problem_prun->getTrajectoryPtr()->getFrameListPtr()->size(), false);

            assert(frame_connected.size() == frame_ptr_2_index_prun.size());

            std::list<FrameBase*> pending_spread_frames({frame_own});
            frame_connected[frame_ptr_2_index_prun[frame_own]] = true;

            while (!pending_spread_frames.empty())
            {
                connected_constraints_list.clear();
                connected_constraints_list = (*pending_spread_frames.front()->getConstraintToListPtr());
                pending_spread_frames.front()->getConstraintList(connected_constraints_list);

                for (auto c_it : connected_constraints_list)
                {
                    if (c_it == ctr_ptr || c_it->getStatus() == CTR_INACTIVE)
                        continue;

                    if ( c_it->getCapturePtr()->getFramePtr() == frame_other)
                    {
                        //std::cout << "YES: Not the only way to connect the frames involved!" << std::endl;
                        return true;
                    }
                    if ( c_it->getFrameOtherPtr() == frame_other)
                    {
                        //std::cout << "YES: Not the only way to connect the frames involved!" << std::endl;
                        return true;
                    }

                    if ( c_it->getCapturePtr()->getFramePtr() != nullptr &&
                         !frame_connected[ frame_ptr_2_index_prun[ c_it->getCapturePtr()->getFramePtr() ] ])
                    {
                        frame_connected[ frame_ptr_2_index_prun[ c_it->getCapturePtr()->getFramePtr() ] ] = true;
                        pending_spread_frames.push_back(c_it->getCapturePtr()->getFramePtr());
                    }

                    if ( c_it->getFrameOtherPtr() != nullptr &&
                         !frame_connected[ frame_ptr_2_index_prun[ c_it->getFrameOtherPtr() ] ])
                    {
                        frame_connected[ frame_ptr_2_index_prun[ c_it->getFrameOtherPtr() ] ] = true;
                        pending_spread_frames.push_back(c_it->getFrameOtherPtr());
                    }
                }
                pending_spread_frames.pop_front();
            }
            //std::cout << "NO: No connectivity between two nodes apart from this constraint!" << std::endl;
            return false;
        }

        bool checkIntegrity()
        {
            //std::cout << "Checking graph integrity..." << std::endl;
            std::vector<bool> frame_connected(wolf_problem_prun->getTrajectoryPtr()->getFrameListPtr()->size(), false);
            std::list<ConstraintBase*> connected_constraints_list;

            assert(frame_connected.size() == frame_ptr_2_index_prun.size());

            std::list<FrameBase*> pending_spread_frames({index_2_frame_ptr_prun[0]});
            frame_connected[0] = true;

            while (!pending_spread_frames.empty())
            {
                connected_constraints_list.clear();
                connected_constraints_list = (*pending_spread_frames.front()->getConstraintToListPtr());
                pending_spread_frames.front()->getConstraintList(connected_constraints_list);

                for (auto c_it : connected_constraints_list)
                {
                    if (c_it->getStatus() == CTR_INACTIVE)
                        continue;

                    if ( c_it->getCapturePtr()->getFramePtr() != nullptr &&
                         !frame_connected[ frame_ptr_2_index_prun[ c_it->getCapturePtr()->getFramePtr() ] ])
                    {
                        frame_connected[ frame_ptr_2_index_prun[ c_it->getCapturePtr()->getFramePtr() ] ] = true;
                        pending_spread_frames.push_back(c_it->getCapturePtr()->getFramePtr());
                    }

                    if ( c_it->getFrameOtherPtr() != nullptr &&
                         !frame_connected[ frame_ptr_2_index_prun[ c_it->getFrameOtherPtr() ] ])
                    {
                        frame_connected[ frame_ptr_2_index_prun[ c_it->getFrameOtherPtr() ] ] = true;
                        pending_spread_frames.push_back(c_it->getFrameOtherPtr());
                    }
                }
                pending_spread_frames.pop_front();
            }
            if ( std::all_of(frame_connected.begin(), frame_connected.end(), [](bool i){return i;}) )
            {
                //std::cout << "SUCCESS: All nodes are connected!" << std::endl;
                return true;
            }
            else
            {
                std::cout << "FAIL: Not all nodes connected!" << std::endl;
                return false;
            }
        }

        void pruneAndSolve(const double& _ig_threshold, visualization_msgs::Marker& prior_solution_marker, visualization_msgs::Marker& prunned_solution_marker, visualization_msgs::Marker& full_solution_marker, visualization_msgs::Marker& prunned_constraints_marker, visualization_msgs::Marker& full_constraints_marker)
        {
            ig_threshold = _ig_threshold;

            // RESET CONSTRAINTS (ACTIVE ALL)
            for (auto c_it : ordered_ctr_ptr)
                c_it->setStatus(CTR_ACTIVE);
            ceres_manager_prun->update();

            // RESET PRIORS
            for (auto i = 0; i< priors_vector.size(); i++)
            {
                index_2_frame_ptr_full[i]->getPPtr()->setVector(priors_vector[i].head(2));
                index_2_frame_ptr_prun[i]->getPPtr()->setVector(priors_vector[i].head(2));
                index_2_frame_ptr_full[i]->getOPtr()->setVector(priors_vector[i].tail(1));
                index_2_frame_ptr_prun[i]->getOPtr()->setVector(priors_vector[i].tail(1));
            }

            // store prior marker
            wolfToFramesMarker(prior_solution_marker, wolf_problem_prun);

            // PRUNNING
            clock_t t1 = clock();
            // Check heuristic: and connectivity for each constraint
            auto ig_it = ordered_ig.begin();
            for (auto c_it = ordered_ctr_ptr.begin(); c_it != ordered_ctr_ptr.end(); c_it++, ig_it++)
                if (*ig_it > ig_threshold)
                    break;
                else if (isRemovable(*c_it))
                    (*c_it)->setStatus(CTR_INACTIVE);

            ceres_manager_prun->update();
            t_prunning = ((double) clock() - t1) / CLOCKS_PER_SEC;

            // CHECKING INTEGRITY
            checkIntegrity();
            N_removable_nodes = 0;
            std::list<ConstraintBase*> connected_constraints_list;
            for (auto fr_it : *(wolf_problem_prun->getTrajectoryPtr()->getFrameListPtr()) )
            {
                unsigned int n_constraints = 0;
                connected_constraints_list = (*fr_it->getConstraintToListPtr());
                fr_it->getConstraintList(connected_constraints_list);

                for (auto c_it : connected_constraints_list)
                {
                    if (c_it->getCategory() != CTR_ABSOLUTE)
                    {
                        n_constraints++;
                        if (n_constraints > 2)
                            break;
                    }
                }
                if (n_constraints == 2)
                    N_removable_nodes++;
            }

            // SOLVING PROBLEMS
            t1 = clock();
            summary_full = ceres_manager_full->solve(ceres_options);
            t_solve_full = ((double) clock() - t1) / CLOCKS_PER_SEC;
            //std::cout << summary_full.BriefReport() << std::endl;
            t1 = clock();
            summary_prun = ceres_manager_prun->solve(ceres_options);
            t_solve_prun = ((double) clock() - t1) / CLOCKS_PER_SEC;
            //std::cout << summary_prun.BriefReport() << std::endl;

            // FITTING
            fitting_error = trajectory_icp->fit();

            // PLOTING RESULTS
            wolfToFramesMarker(prunned_solution_marker, wolf_problem_prun);
            wolfToConstraintsMarker(prunned_constraints_marker, wolf_problem_prun);
            wolfToFramesMarker(full_solution_marker, wolf_problem_full);
            wolfToConstraintsMarker(full_constraints_marker, wolf_problem_full);
        }

        void printTimes()
        {
            std::cout << std::endl << "----------- PRUNNING ig_th = " << ig_threshold << std::endl;
            std::cout << "Total constraints:      " << summary_full.num_residual_blocks << std::endl;
            std::cout << "Remaining constraints:  " << summary_prun.num_residual_blocks << std::endl;
            std::cout << "Total nodes:            " << summary_full.num_parameter_blocks << std::endl;
            std::cout << "Remaining nodes:        " << summary_prun.num_parameter_blocks << std::endl;
            std::cout << "2-constrained nodes:    " << N_removable_nodes << std::endl;
            std::cout << "Error of prunned:       " << fitting_error << std::endl;

            std::cout << std::endl << "COMPUTATIONAL COSTS:" << std::endl;
            std::cout << "covariance computation:      " << t_sigma_ceres << "s" << std::endl;
            std::cout << "IG computation and ordering: " << t_ig << "s" << std::endl;
            std::cout << "prunning computation:        " << t_prunning << "s" << std::endl;
            std::cout << "solving full problem:        " << t_solve_full << "s" << std::endl;
            std::cout << "solving prunned problem:     " << t_solve_prun << "s" << std::endl;
        }
};

int main(int argc, char **argv)
{
    std::cout << "============= PRUNNING TEST USING WOLF IN ROS =============" << std::endl;
    int max_vertex_;

    ros::Time t1;
    double ig_threshold;

    visualization_msgs::Marker prior_marker_, prunned_solution_marker_, full_solution_marker_;
    prior_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    prior_marker_.header.frame_id = "map";
    prior_marker_.pose.position.x = 0;
    prior_marker_.pose.position.x = 0;
    prior_marker_.pose.position.x = 0;
    prior_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    prior_marker_.scale.x = 0.1;
    prior_marker_.color.r = 0; //blue
    prior_marker_.color.g = 0;
    prior_marker_.color.b = 1;
    prior_marker_.color.a = 1;
    prior_marker_.ns = "/prior";
    prior_marker_.id = 1;
    prunned_solution_marker_ = prior_marker_;
    prunned_solution_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    prunned_solution_marker_.scale.x = 0.5;
    prunned_solution_marker_.color.r = 1; //yellow
    prunned_solution_marker_.color.g = 1;
    prunned_solution_marker_.color.b = 0;
    prunned_solution_marker_.color.a = 1;
    prunned_solution_marker_.ns = "/prunned";
    prunned_solution_marker_.id = 2;
    full_solution_marker_ = prunned_solution_marker_;
    full_solution_marker_.color.r = 1; //red
    full_solution_marker_.color.g = 0;
    full_solution_marker_.color.b = 0;
    full_solution_marker_.color.a = 1;
    full_solution_marker_.ns = "/full";
    full_solution_marker_.id = 3;

    visualization_msgs::Marker prunned_constraints_marker_, full_constraints_marker_;
    prunned_constraints_marker_.type = visualization_msgs::Marker::LINE_LIST;
    prunned_constraints_marker_.header.frame_id = "map";
    prunned_constraints_marker_.pose.position.x = 0;
    prunned_constraints_marker_.pose.position.x = 0;
    prunned_constraints_marker_.pose.position.x = 0;
    prunned_constraints_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    prunned_constraints_marker_.scale.x = 0.05;
    prunned_constraints_marker_.color.r = 1; //yellow
    prunned_constraints_marker_.color.g = 1;
    prunned_constraints_marker_.color.b = 0;
    prunned_constraints_marker_.color.a = 1;
    prunned_constraints_marker_.ns = "/prunned_constraints";
    prunned_constraints_marker_.id = 4;
    full_constraints_marker_ = prunned_constraints_marker_;
    full_constraints_marker_.color.r = 1; //red
    full_constraints_marker_.color.g = 0;
    full_constraints_marker_.color.b = 0;
    full_constraints_marker_.color.a = 1;
    full_constraints_marker_.ns = "/full_constraints";
    full_constraints_marker_.id = 5;

    // loading variables
    std::string file_path_;
    // init
    google::InitGoogleLogging(argv[0]);

    //ros init
    ros::init(argc, argv, "wolf_prunnning");
    std::cout << "Ros initialized!" << std::endl;

    ros::NodeHandle nh("~");
    nh.param<std::string>("file_path", file_path_, "/home/jvallve/bags/graphs/input_M3500b_toro.graph");
    nh.param<int>("max_vertex", max_vertex_, 0);
    nh.param<double>("ig_threshold", ig_threshold, 2);
    ros::Publisher prior_publisher = nh.advertise <visualization_msgs::Marker> ("prior",1);
    ros::Publisher prunned_solution_publisher = nh.advertise <visualization_msgs::Marker> ("prunned_solution",1);
    ros::Publisher full_solution_publisher = nh.advertise <visualization_msgs::Marker> ("full_solution",1);
    ros::Publisher prunned_constraints_publisher = nh.advertise <visualization_msgs::Marker> ("prunned_constraints",1);
    ros::Publisher full_constraints_publisher = nh.advertise <visualization_msgs::Marker> ("full_constraints",1);

    // prunning object
    WolfPrunning wolf_prunning(file_path_, max_vertex_);
    ig_threshold = 0;
    t1 = ros::Time::now();

    while ( ros::ok() )
    {
        if ((ros::Time::now() - t1).toSec() > 3)
        {
            t1 = ros::Time::now();
            ig_threshold = (ig_threshold > 10 ? 0.5 : ig_threshold + 0.5);

            // solve
            wolf_prunning.pruneAndSolve(ig_threshold, prior_marker_, prunned_solution_marker_, full_solution_marker_, prunned_constraints_marker_, full_constraints_marker_);

            // print times
            wolf_prunning.printTimes();

            // publish trajectories
            prior_publisher.publish(prior_marker_);
            prunned_solution_publisher.publish(prunned_solution_marker_);
            full_solution_publisher.publish(full_solution_marker_);
            prunned_constraints_publisher.publish(prunned_constraints_marker_);
            full_constraints_publisher.publish(full_constraints_marker_);
        }
    }

    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;

    //exit program
    return 0;
}



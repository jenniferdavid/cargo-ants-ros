#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

//std
#include <iostream>
#include <vector>
#include <list>
#include <iomanip>

// Eigen
#include <eigen3/Eigen/Geometry>

//laserscanutils
#include "laser_scan_utils.h"
#include "entities.h"
#include "scan_basics.h"
#include "line_detector.h"
#include "corner_detector.h"




namespace laserscanutils
{
    struct AssociationResult
    {
        // Constructor
        AssociationResult(float _value = 100.00)
        {
            this->corner_ = _value;
            this->angle_ = _value;
            this->line_init_ = _value;
            this->line_end_ = _value;
            this->centroid_ = _value;
        }

        float corner_;
        float angle_;
        float line_init_;
        float line_end_;
        float centroid_;

    };

    struct LaserScanProcessed        // Stores all the needed information from a scan
    {
        // Constructor
        LaserScanProcessed(unsigned int _num_points=10)
        {
            std::cout << " DEBUG INIT. LASERSCANPROCESSED: NumPoints = " << _num_points << std::endl;
            assert(_num_points < 5000 && "Error: saving scan points in laser Scan Processed. Check initialization." );
            this->scan_points_matrix_.conservativeResize(3,_num_points);
//            this->scan_points_matrix_.resize(3,_num_points);
            this->odom_pos_ = {0.0, 0.0, 0.0};
            this->odom_ori_ = {0.0, 0.0, 0.0, 0.0};
            this->num_points_in_clusters_ = 0;
        }

        unsigned int id_;       // Processed scan identifier.

        // Atributes
        //TBR?// std::vector<float> scan_raw_ranges_;           // [TBC] Raw scan ranges information
        Eigen::MatrixXs scan_points_matrix_;           // Homogeneous Coordinates for the valid scan points.

        std::vector< std::pair<int,int> > cluster_indxs_;   // Indexs of the initial and final position (of the scan_points_matrix_) for the clusters in scan.
        std::list<laserscanutils::Object> object_list_;     // Contains the different objects detected (clusters with associate information) in a scan.
        //std::list<laserscanutils::Line> line_list_;         //[TBR] Contains all the lines found in the scan (GLOBAL). -> TBR: use object lines!

        unsigned int scan_total_points_;            //Total number of points in the raw scan
        unsigned int filtered_total_points_;        //Total number of valid points after pre filter
        unsigned int number_of_clusters_;         //Total number of clusters found in the scan (valid and not valid)
        unsigned int num_points_in_clusters_;     // Total number of points in the clusters
        unsigned int number_of_valid_clusters_;   //Total number of valid clusters found in the scan
        unsigned int number_of_global_scan_;      //Total number scan till now


        std::vector<float> odom_pos_;              // [X,Y,Z] position of the global odometry "nav_msg: pose.pose.position"
        std::vector<float> odom_ori_;              // [X,Y,Z,W] orientation of the global odometry. "nav_msg: pose.pose.orientation"
        Eigen::Matrix3s odom_eigen_tf_;                // Eigen Matrix with the corresponding TF to base


       // std::vector<int> object_asso_values_;       // Stores the strength of the associated objects

    };



    struct LaserObjectDetAlgParams
    {
        //members
        float cluster_jump_thr_;                 // max distance between points
        float cluster_min_num_points_;   // minimum number of points for a cluster
        float cluster_max_num_points_;   // maximum number of points for a cluster

        // Line extraction
        unsigned int fit_line_window_size_;     // number of points to fit lines in the first pass
        ScalarT fit_line_theta_max_parallel_;   // maximum theta between consecutive segments to fuse them in a single line.
        ScalarT fit_line_k_sigmas_;                      // How many std_dev are tolerated to count that a point is supporting a line

        // Corner extraction
        ScalarT corner_theta_min_;                     // minimum theta between consecutive segments to detect corner. PI/6=0.52

        // TF Scan association
        unsigned int tf_scan_window_size_;         // Number of scanners on the buffer (comparison of movement btw thos step)
        float asso_corner_dist_;                   // max distance between corners of two scans to be associated
        float asso_angle_diff_;                    // max angle difference between lines of two scans to be associated
        float asso_lines_ext_dist_;                // max distance between initial OR final line points of two scans to be associated

        //ExtractLineParams extract_line_params_;     // parameters for extracting lines


        void print() const;
    };


//    template <class T, size_t ROW, size_t COL, size_t DEPTH>
//    using MultiDimArray = std::array<std::array<std::array<T, DEPTH>, COL>, ROW>;
//    MultiDimArray<float, > floats;


    /** \brief Test Porpuses to create a Matlab script with all the variables.
    **/
    void exportStaticScanProcessed2Matlab(const char * _file_name,
                                          const LaserObjectDetAlgParams & _alg_params,
                                          const LaserScanProcessed & _scan_processed);


    /** \brief Test Porpuses to create a Matlab script with all the variables.
    **/
    void exportDataScanProcessed2Matlab(std::string &_file_name,
                                        LaserScanProcessed & _scan_processed);


    /** \brief Test Porpuses print Matrix Association Results.
    **/
    void printAssociationResults(std::vector<std::vector<laserscanutils::AssociationResult> > & _asso_results);




    /** \brief Filters the valid ranges on a scan and converts them to homogeneous coordinates.
    *
    * Valid ranges are filtered according to scan parameters.
    * Converts ranges from a laserScan to homogeneous coordinates and save it to a matrix of same size than ranges size, returning index till valid values.
    *
    * \param _params is the laser_scan_utils structure of the laser parameters.
    * \param _ranges is the raw scan measures. Vector of float ranges.
    * \param _points is the returned points from the conversion of ranges to homogeneous coordinates
    * \param _stats is the statistics structure where number of valid points is saved under _stats.filteredTotalPoints
    *
    **/
    void preFilterScan2homog(const laserscanutils::ScanParams & _params,
                             const std::vector<float> & _ranges,
                             Eigen::MatrixXs & _points,
                             laserscanutils::LaserScanProcessed &_scan_processed);



    /** \brief Find the clusters in the valid points from the scan.
    *
    * Check distance between valid points in homogeneous coordinates. If it is more than the threshold, considers it as a different cluster.
    *
    * \param _alg_params is the structure of algorithm params for the Laser Object Detection Algorithm.
    * \param _points is the returned points from the conversion of ranges to homogeneous coordinates
    * \param _global_scan_number is the global number of scan processed (to be used as header id)
    * \param _scan_processed is the structure where all the information of elements in a scan is stored
    *
    **/
    void extractClustersInScan(const laserscanutils::LaserObjectDetAlgParams & _alg_params,
                               const Eigen::MatrixXs & _points,
                               unsigned int _global_scan_number,
                               laserscanutils::LaserScanProcessed & _scan_processed);




    /** \brief Extract lines in the given cluster.
    *
    * From a cluster of points (EigenMatrix) extract the existing lines according to the alg parameters
    *
    * \param _alg_params is the structure of algorithm params for the Laser Object Detection Algorithm.
    * \param _points is the returned points from the conversion of ranges to homogeneous coordinates
    * \param _global_scan_number is the global number of scan processed (to be used as header id)
    * \param _scan_processed is the structure where all the information of elements in a scan is stored
    *
    **/
    void findLinesInCluster(const laserscanutils::ScanParams & _scan_params,
                             const laserscanutils::LaserObjectDetAlgParams &_alg_params,
                             const Eigen::MatrixXs &_cluster,
                             std::list<Line> &_line_list);




    /** \brief Extract corners in the lines of a given cluster.
    *
    * From a list of lines belonging to the same cluster, extracts the existing corners according to the alg parameters
    *
    * \param _alg_params is the structure of algorithm params for the Laser Object Detection Algorithm.
    * \param _line_list is the list of lines from which the corners will be extracted.
    * \param _corner_list is the list of found corners in the cluster.
    *
    **/
    unsigned int findCornersInClusterLines(const laserscanutils::LaserObjectDetAlgParams & _obj_det_alg_params,
                                           const std::list<laserscanutils::Line> & _line_list,
                                           std::list<laserscanutils::Corner> & _corner_list);



    /** \brief Calculate Features
    *
    * Calculate middle point, centroid, length, with, area...
    *
    * \param _alg_params is the structure of algorithm params for the Laser Object Detection Algorithm.
    * \param _points is the returned points from the conversion of ranges to homogeneous coordinates
    * \param _global_scan_number is the global number of scan processed (to be used as header id)
    * \param _scan_processed is the structure where all the information of elements in a scan is stored
    *
    **/
    void extractFeaturesInObject(const Eigen::MatrixXs & _points,
                                  Object &_object);




    /** \brief Performs the association from two objects list
    *
    * From a list of objects of both previous and actual scans, computes the distances and find the
    * closest (euclidean dist threshold) ones as associated.
    *
    *
    **/
    void associateObjects(std::list<laserscanutils::Object> & _prev_objects_list,
                          std::list<laserscanutils::Object> & _act_objects_list,
                          float _corner_threshold, float _angle_thr, float _line_ext_threshold);



//    /** \brief Fullfils the bounding boxes and rest of parameters on the detections.
//    *
//    **/
//    void objectsBB(laserscanutils::LaserScanProcessed & _act_scan);

}




#endif // OBJECT_DETECTOR_H

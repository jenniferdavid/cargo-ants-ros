/******* DEPRECATED *******/

#ifndef CLUSTERING_H
#define CLUSTERING_H

//laserscanutils
#include "laser_scan_utils.h"
#include "entities.h"
#include "scan_basics.h"
#include "corner_detector.h"


#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
//#include <algorithm>    // std::min

//#include <fstream>  // File Printing for tests

//#include "segment.h"




namespace laserscanutils
{

    /** \brief Set of tunning parameters
     *
     * Set of tunning parameters for corner extraction
     *
     */
    struct LaserDetectionParams
    {
        //members
        float jump_thr;                 //max distance between points
        float min_num_cluster_points;   //minimum number of points for a cluster
        float max_num_cluster_points;   //maximum number of points for a cluster

        unsigned int segment_window_size; //number of points to fit lines in the first pass
        ScalarT theta_max_parallel; //maximum theta between consecutive segments to fuse them in a single line.
        ScalarT k_sigmas;//How many std_dev are tolerated to count that a point is supporting a line



        void print() const;
    };

    struct LaserDetectionStats // TO BE REMOVED [TBR] -> Substitute = ObjectDetector.h: LaserScanProcessed
    {
        // Mid variables for the algorithm, storing information needed for the correct working

        unsigned int scanTotalPoints;            //Total number of points in the raw scan
        unsigned int filteredTotalPoints;        //Total number of valid points after pre filter
        unsigned int numberOfClusters;           //Total number of clusters found in the scan
        unsigned int numberOfValidClusters;      //Total number of valid clusters found in the scan
        std::vector< std::pair<int,int> > cluster_indxs_;   // Contains initial and final columns (of the filtered homogeneous matrix) for clusters in scan.
        std::list<laserscanutils::Line> line_list_;         //[TBR] Contains all the lines found in the scan (GLOBAL).
        std::list<laserscanutils::Object> object_list_;     // Contains the different objects detected (clusters with associate information) in a scan.

        std::vector<float> odom_pos_;              // [X,Y,Z] position of the global odometry "nav_msg: pose.pose.position"
        std::vector<float> odom_ori_;             // [X,Y,Z,W] orientation of the global odometry. "nav_msg: pose.pose.orientation"


    };



    // TEST
    Eigen::MatrixXs loadScanMatrix();


    void exportStaticData2Matlab(const char * _file_name,
                                 const LaserDetectionParams & _alg_params);

    /** \brief Test Porpuses to create a Matlab script with all the variables.
    **/
    void exportData2Matlab(std::string &_file_name, const Eigen::MatrixXs & _points,
                           LaserDetectionStats &_stats, unsigned int _scanNumber);



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
    void preFilterScan(const laserscanutils::ScanParams & _params,
                       const std::vector<float> & _ranges,
                       Eigen::MatrixXs & _points,
                       laserscanutils::LaserDetectionStats &_stats);



//    /** \brief Find the clusters in the valid points from the scan.
//    *
//    * Check distance between valid points in homogeneous coordinates. If it is more than the threshold, considers it as a different cluster.
//    *
//    * \param _alg_params is the structure of algorithm params for the Laser Detection Algorithm.
//    * \param _points is the returned points from the conversion of ranges to homogeneous coordinates
//    * \param _stats is the statistics structure (TODO: What is used here? - Number of Clusters)
//    **/
//    void findClusters(const LaserDetectionParams & _alg_params,
//                              const Eigen::MatrixXs & _points,
//                              LaserDetectionStats & _stats);






//    void findLinesInCluster(const laserscanutils::ScanParams & _params,
//                             const LaserDetectionParams &_alg_params,
//                             const Eigen::MatrixXs &_cluster,
//                             const unsigned int initIndex, //
//                             std::list<laserscanutils::Line> &_line_list);




    unsigned int findCornersInClusterLines(const laserscanutils::ScanParams & _params,
                                           const ExtractCornerParams & _alg_params,
                                           const std::list<laserscanutils::Line> & _line_list,
                                           std::list<laserscanutils::Corner> & _corner_list);






    /** \brief Converts ranges from a laserScan to homogeneous coordinates. Check validity of the ranges and look cluster jumps.
    *
    * Converts ranges from a laserScan to homogeneous coordinates and save it to a matrix of same size than ranges size, returning index till valid values.
    * Valid ranges are filtered and at the same time cluster jumps are checked.
    * TODO (jumpThreshold)
    *
    * \param _params is the laser_scan_utils structure of the laser parameters.
    * \param _ranges is the raw scan measures. Vector os ranges.
    * \param _jump_threshold is the jump to diferenciate between clusters -> TODO in algorithm parameters
    * \param _points is the returned points from the conversion of ranges to homogeneous coordinates
    * \param _jumps_id_ is the vector containing in which column of the _points matrix exist a cluster jump.
    **/// NOT USED!!
    unsigned int scanProcess(const ScanParams & _params,
                             const std::vector<float> & _ranges,
                             const float _jump_threshold,
                             Eigen::MatrixXs & _points,
                             std::vector<unsigned int> & _jumps_id_);




    void printMatrixAndClusters2File(std::string &_file_name,
                                    const unsigned int _num_points,
                                    Eigen::MatrixXs & _points,
                                    std::vector<std::pair<int, int> > _cluster_indxs_);



    void printLinesToFile(std::string &_file_name,
                          std::list<laserscanutils::Line> & _line_list);



    // VV - Old, needs jumps array.
    void printMatrix2FileJumpsVersion(char *_file_name, const unsigned int _num_points,
                   Eigen::MatrixXs & _points,
                   std::vector<unsigned int> & _jumps_id_);





  /** \brief Analyse clusters and extract the geometry primitives - line, corner...
   *
   * TODO
   *
   **/
//  unsigned int extractGeomPrimitives(const laserscanutils::ScanParams & _params, const ExtractCornerParams & _alg_params, const std::vector<float> & _ranges, std::list<laserscanutils::Corner> & _corner_list);

}
#endif// CLUSTERING_H

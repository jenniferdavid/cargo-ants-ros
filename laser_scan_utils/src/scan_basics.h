
#ifndef SCAN_BASICS_H_
#define SCAN_BASICS_H_

//std
#include <iostream>
#include <vector>
#include <queue>
#include <list>

//laserscanutils
#include "laser_scan_utils.h"

namespace laserscanutils
{
    struct ScanParams
    {
        //members
        double angle_min_; //radians
        double angle_max_; //radians
        double angle_step_; //radians
        double scan_time_; //time from the measuring the first ray up to measuring the last one, seconds
        double range_min_; //meters 
        double range_max_; //meters
        double range_std_dev_; //standard deviation measurement noise in range, meters 
        
        //just a print method
        void print() const;
    };
    
    /** \brief Transforms from ranges (polar) to euclidean (xy)
     * 
     * Transforms from polar (ranges) to euclidean (xy), while checking correctness of raw data.
     * Invalid values (inf's and nan's) and out of range measurements are not transformed. 
     * Valid measurements are returned, by reference, in homogeneous coordinates.
     * Ranges vector is returned with all not correct elements removed.
     * A list of jumps indices is also returned by reference.
     * \param _params laser_scan_utils structure of the laser scan parameters.
     * \param _ranges raw range measurements from the laser scanner device. Size Nr. Type float to match ROS LaserScan message 
     * \param _jump_theshold distance threshold to classify a jump in the range.
     * \param _points matrix of homogeneous coordinates of hit points. Size 3xNp, where Np can be <= than Nr (due to possible inf's and nan's from raw data)
     * \param jumps_id_ list of indexes of jumps in range measurements. Indexes correpond to _points vector.
     * 
    **/
    void ranges2xy(const ScanParams & _params, std::vector<float> & _ranges, const ScalarT& _jump_threshold, Eigen::MatrixXs & _points, std::list<unsigned int> & _jumps);

    /** \brief Transforms from ranges (polar) to euclidean (xy) detecting jumps not in absolute range threshold but in angle w.r.t the beams
     *
     * Transforms from polar (ranges) to euclidean (xy), while checking correctness of raw data.
     * Invalid values (inf's and nan's) and out of range measurements are not transformed.
     * Valid measurements are returned, by reference, in homogeneous coordinates.
     * Ranges vector is returned with all not correct elements removed.
     * A list of jumps indices is also returned by reference.
     * \param _params laser_scan_utils structure of the laser scan parameters.
     * \param _ranges raw range measurements from the laser scanner device. Size Nr. Type float to match ROS LaserScan message
     * \param _jump_theshold distance threshold to classify a jump in the range.
     * \param _jump_angle_theshold angle threshold to classify a jump in the range.
     * \param _points matrix of homogeneous coordinates of hit points. Size 3xNp, where Np can be <= than Nr (due to possible inf's and nan's from raw data)
     * \param jumps_id_ list of indexes of jumps in range measurements. Indexes correpond to _points vector.
     *
    **/
    void ranges2xy(const ScanParams & _params, std::vector<float> & _ranges, const ScalarT& _jump_threshold, const ScalarT& _jump_angle_threshold, Eigen::MatrixXs & _points, std::list<unsigned int> & _jumps);

    /** \brief Transforms from ranges (polar) to euclidean (xy)
     *
     * Transforms from polar (ranges) to euclidean (xy), while checking correctness of raw data.
     * Invalid values (inf's and nan's) and out of range measurements are not transformed.
     * Valid measurements are returned, by reference, in homogeneous coordinates.
     * Ranges vector is returned with all not correct elements removed.
     * Requires: 
     *      \param _params laser_scan_utils structure of the laser scan parameters.
     *      \param _ranges raw range measurements from the laser scanner device. Size Nr. Type float to match ROS LaserScan message
     * Outputs: 
     *      \param _points matrix of homogeneous coordinates of hit points. Size 3xNp, where Np can be <= than Nr (due to possible inf's and nan's from raw data)
    **/
    void ranges2xy(const ScanParams & _params, std::vector<float> & _ranges, Eigen::MatrixXs & _points);


    /** \brief Fits angle in (-pi, pi]
     *
     * Fits angle in (-pi, pi]
    **/
    ScalarT pi2pi(const ScalarT& angle);
}
#endif

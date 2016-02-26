
#ifndef CORNERS_H_
#define CORNERS_H_

//std
#include <iostream>
#include <list>
#include <queue>
#include <random>

// Eigen ingludes
#include <eigen3/Eigen/Geometry>

//laserscanutils
#include "laser_scan_utils.h"
#include "scan_basics.h"
#include "entities.h"
#include "line_detector.h"

namespace laserscanutils
{
    /** \brief Set of tunning parameters
     * 
     * Set of tunning parameters for corner extraction
     * 
     */
    struct ExtractCornerParams
    {
        //members
//         unsigned int segment_window_size; //number of points to fit lines in the first pass
//         ScalarT theta_min; //minimum theta between consecutive segments to detect corner. PI/6=0.52
//         ScalarT theta_max_parallel; //maximum theta between consecutive segments to fuse them in a single line.
//         ScalarT k_sigmas;//How many std_dev are tolerated to count that a point is supporting a line
//         unsigned int max_beam_distance;//max number of beams of distance between lines to consider corner or concatenation
//         ScalarT max_distance;//max distance between line ends to consider corner or concatenation
        
        ScalarT theta_min_; //minimum theta between consecutive segments to detect corner. PI/6=0.52
        ScalarT max_distance_;//max distance between line extremes to consider a corner
        ExtractLineParams line_params_; //algorithm parameters for line extraction, whcih is executed previously to corner extraction

        //just a print method
        void print() const;
    };

    /** \brief Extract corners from a given lines list. Result as a list of Corners
     *
     * Extract corners from a given lines list.
     * Returns corners as a list of laserscanutils::Corners
     *
     */
    unsigned int extractCorners(const laserscanutils::ScanParams & _params, 
                                const laserscanutils::ExtractCornerParams & _alg_params, 
                                const std::list<laserscanutils::Line> & _line_list, 
                                std::list<laserscanutils::Corner> & _corner_list);

    /** \brief Extract corners from a given scan. Result as a list of Corners
     * 
     * Extract corners from a given scan. 
     * Returns corners as a list of laserscanutils::Corners
     * 
     */    
    unsigned int extractCorners(const laserscanutils::ScanParams & _params, 
                                const laserscanutils::ExtractCornerParams & _alg_params, 
                                const std::vector<float> & _ranges, 
                                std::list<laserscanutils::Corner> & _corner_list);
    
    /** \brief Returns corner aperture 
     * 
     * Computes corner aperture in 2D, given two extreme points and the corner location
     * REQUIRES: three points in homogeneous coordinates (last component set to 1): 
     *     _p1: first point of the first line supporting a corner
     *     _c: corner point
     *     _p2: last point of the second line supporting a corner
     * ASSUMES: 
     *     - _p1, _c and _p2 are not collinear since they probably come from a corner detector
     *     - _p1, _c and _p2 are ordered as a scan can view them, scanning clockwise from the top view point
     * ENSURES:
     *     A return angle in [0,2pi]
     **/
    ScalarT cornerAperture(const Eigen::Vector3s & _p1, 
                           const Eigen::Vector3s & _c, 
                           const Eigen::Vector3s & _p2);
    
}
#endif


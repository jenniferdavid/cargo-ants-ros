#ifndef LASER_SCAN_H_
#define LASER_SCAN_H_

//laserscanutils
#include "laser_scan_utils.h"
#include "scan_segment.h"

//std
#include <vector>
#include <list>
#include <iostream>

namespace laserscanutils
{

/** \brief Laser scan parameters
 * 
 * Laser scan parameters
 * 
 **/
struct LaserScanParams
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
    
/** \brief Class for raw laser scan data and basic methods
* 
* Class for raw laser scan data and basic methods
* 
*/
class LaserScan
{
    protected:
        LaserScanParams params_;
        
    public: 
        //Ordered raw range data
        std::vector<float> ranges_; 
        
        //ordered 2D points, each one expressed in homogeneous coordinates (x,y,1)^T. NaN and inf's are filtered out. 
        Eigen::MatrixXs points_;
        
        //list of indexes where a scan jump is found. Indexes incicate start of a scan segment
        std::list<unsigned int> jumps_; 
            
    public: 
        /** \brief Constructor
         * 
         * Constructor
         * 
         **/
        LaserScan(); 
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        ~LaserScan(); 
        
        /** \brief Set scan params
         * 
         * Set scan params. 
         * 
         **/
        void setLaserScanParams(const laserscanutils::LaserScanParams & _params);
        
        /** \brief Transforms from ranges (polar) to euclidean (xy)
        *
        * Transforms from polar (ranges) to euclidean (xy), while checking correctness of raw data.
        * Invalid values (inf's and nan's) and out of range measurements are not transformed.
        * Valid measurements are set to points_ in homogeneous coordinates.
        * 
        * _device_T is the homogeneous transformation, moving points from laser device frame to the reference one:
        *       p_ref = _device_T*p_laser;  
        * If _device_T is provided, points are transformed to a new reference frame. 
        * 
        * Set also the jumps_ vector, which after the call holds the indexes to points_ where a scan segment starts
        *
        **/
        void ranges2xy(Eigen::Matrix4s _device_T = Eigen::Matrix4s::Identity());        

        /** \brief Find segments based on jumps of consecutive scan points
        *
        * Find segments based on jumps of consecutive scan points
        * Do not compute segment parameters, just fill ScanSegment.points_
        *
        **/
        void findSegments(std::list<laserscanutils::ScanSegment> & _segment_list);        
      
};

}//namespace
#endif



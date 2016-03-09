#ifndef LINE_SEGMENT_H_
#define LINE_SEGMENT_H_

//laserscanutils
#include "laser_scan_utils.h"

//std
#include <iostream>

//open namespace
namespace laserscanutils
{
/** \brief Line segment
* 
* Line segment
* 
*/
class LineSegment
{
    public:
        /** \brief Homogeneous parameterization of the line to which the segment lies on. 
         * 
         * Homogeneous parameterization of the line to which the segment lies on: (a,b,c)^T -> ax+by+c=0. 
         * Vector (a/c,b/c)^T is the normal vector, pointing to a free surface
         * 
         */
        Eigen::Vector3s abc_; 
        unsigned int idx_first_; //corresponding index to the scan ranges of the first point used
        unsigned int idx_last_; //corresponding index to the scan ranges of the last point used
        Eigen::Vector3s point_first_; //homogeneous coordinates of the starting 2D point
        Eigen::Vector3s point_last_; //homogeneous coordinates of the ending 2D point
        ScalarT fit_error_; //sum of all distances from used points to line when fitting
        unsigned int np_; // number of scan points supporting this line (through fitting, Hough voting, ...)
        ScalarT range_; //range component in polar parameterization
        ScalarT theta_; //theta component in polar parameterization.
        ScalarT length_; //theta component in polar parameterization.

    public:
        //constructor
        LineSegment();
        
        //Destructor
        ~LineSegment();
        
        /** \brief Computes projection of point to this line
         * 
         * Computes the projection of _in_pt to this LineSegment. Result at _out_pt
         * Everything in homogeneous coordinates
         * 
        **/
        void pointProjectionToLine(const Eigen::Vector3s & _in_pt, Eigen::Vector3s & _out_pt) const;
        
        //merges this LineSegment with the argument. 
        void merge(const LineSegment & _segment); 
      
        //print
        void print() const;    
};
}//namespace
#endif
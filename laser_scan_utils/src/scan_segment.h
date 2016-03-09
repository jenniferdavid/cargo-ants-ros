#ifndef SCAN_SEGMENT_H_
#define SCAN_SEGMENT_H_

//laserscanutils
#include "laser_scan_utils.h"

//std
#include <iostream>

//open namespace
namespace laserscanutils
{
/** \brief Scan segment
* 
* A segment is a set of connected scan points.
* "Connected" means that neigbors are close enough (below a threshold).
* "Neigbors" implies scan ordering is required 
* 
*/
class ScanSegment
{
    protected:
        unsigned int segment_id_; //Segment id. 
        static unsigned int segment_id_count_; //Segment counter (acts as simple ID factory)        
        Eigen::Vector3s centroid_; //homogeneous coordinates of the segment centroid 
        Eigen::Vector3s bbox_corners_[4]; //4 corners of the bounding box [m]
        ScalarT eval_1_, eval_2_; //eigenvalues. eval_1_ is the biggest one
    
    public:
        /** \brief Set of points belonging to this segment
         * 
         * Set of points belonging to this segment, in homogeneous coordinates. 
         * Each column is a point (x,y,1)^T
         * 
         */
        Eigen::MatrixXs points_; 

    public:
        //constructor
        ScanSegment();
        
        //Destructor
        ~ScanSegment();
        
        //merges this ScanSegment with the argument. 
        void merge(const ScanSegment & _segment); 
        
        /** \brief Return the number of points supporting this segment
         * 
         * Return the number of points supporting this segment
         * 
         **/
        unsigned int numPoints() const;  
        
        /** \brief Return const ref to centroid
         * 
         * Return const ref to centroid
         * 
         **/
        const Eigen::Vector3s & getCentroid() const;  
        
        /** \brief Return const ref to one corner of the bounding box
         * 
         * Return const ref to one corner of the bounding box
         * 
         **/
        const Eigen::Vector3s & getBBox(unsigned int _corner_idx) const;  
        
        /** \brief Return the ratio eval_2_/eval_1_
         * 
         * Return the ratio eval_2_/eval_1_
         * 
         **/        
        ScalarT getEvalRatio() const; 

        /** \brief Compute centroid coordinates
         *
         * Compute centroid coordinate.
         *      
         **/        
        void computeCentroid(); 
        
        /** \brief Compute the oriented bounding box
         * 
         * Compute the oriented bounding box
         * Clearance is an extra distance added to the bb limits. Typically half od the grid cell size
         * 
         **/
        void computeBoundingBox(const ScalarT & _clearance = 0.1);
        
        /** \brief Compute both centroids and bounding box
         * 
         * Compute both centroids and bounding box
         * 
         **/
        void computeAll(); 
        
        
              
        //print
        void print() const;    
};
}//namespace
#endif
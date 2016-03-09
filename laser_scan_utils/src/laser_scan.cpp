#include "laser_scan.h"

namespace laserscanutils
{
    
LaserScan::LaserScan()
{

}

LaserScan::~LaserScan()
{

}

void LaserScan::setLaserScanParams(const LaserScanParams & _params)
{
    params_ = _params; 
}

void LaserScan::ranges2xy(Eigen::Matrix4s _device_T)
{
    ScalarT azimuth = params_.angle_min_;
    ScalarT prev_range = 0;
    unsigned int ii = 0;
    unsigned int ii_ok = 0;
    ScalarT kr = 10; //TODO: as a parameters somewhere. 
    Eigen::Vector4s point_laser, point_ref;

    //resize to all points case
    points_.resize(3,ranges_.size());

    //clear jumps_ vector
    jumps_.clear();

    //for each range, check correctness of value and translate from polar to xy coordinates
    for ( ii = 0; ii < ranges_.size(); ii++ )
    {
        //check raw range integrity
        if ( ( ranges_[ii] > params_.range_min_ ) && 
             ( ranges_[ii] < params_.range_max_ ) && 
             ( !std::isnan(ranges_[ii]) ) && 
             ( !std::isinf(ranges_[ii]) ) )
        {
            //transform the laser hit from polar to 3D euclidean homogeneous
            point_laser << ranges_[ii]*cos(azimuth), ranges_[ii]*sin(azimuth),0,1;
            
            //apply device mounting point calibration (p_r = T * p_l)
            point_ref = _device_T*point_laser; 

            //set to points_ as a 2D homogeneous
            points_.block<3,1>(0,ii_ok) << point_ref(0),point_ref(1),1;
                        
//             //transform from polar to euclidean
//             points_(0,ii_ok) = ranges_[ii] * cos(azimuth);
//             points_(1,ii_ok) = ranges_[ii] * sin(azimuth);
//             points_(2,ii_ok) = 1;
            
            //check jump. Min dist between consecutive points is r*sin(angle_step_). A jump occurs when this min dist is overpassed by kr times
            if ( fabs(ranges_[ii]-prev_range) > fabs(kr*ranges_[ii]*params_.angle_step_)) //jump condition (kr*r*sin(a) ~ kr*r*a)
            //if ( fabs(ranges_[ii]-prev_range) > 0.5) //jump condition >0.5m
            {
                jumps_.push_back(ii_ok);
            }
           
            //increment ok counter
            ii_ok ++; 
            
            //keep current range as previous for the next iteration
            prev_range = ranges_[ii];            
        }
        else
            prev_range = 0; 
        
        //increment azimuth with angle step
        azimuth += params_.angle_step_;
    }
    
    //push back the last index to jumps_, to properly close the jumps_ vector. This will be used by findSegments()
    jumps_.push_back(ii_ok);

    //resize the output matrix to the number of correct points, while keeping values
    points_.conservativeResize(3, ii_ok);
}

void LaserScan::findSegments(std::list<laserscanutils::ScanSegment> & _segment_list)
{
    std::list<unsigned int>::iterator jumps_it, next_jumps_it, jumps_last; 
    unsigned int num_points; 
    
    //set jumps_last to the last valid element of jumps
    jumps_last = std::prev(jumps_.end()); 
    
    //run over all jumps (except the last, which indicates the closing index) 
    for (jumps_it = jumps_.begin(); jumps_it != jumps_last; jumps_it ++)
    {
        //new segment in the list
        _segment_list.push_back(ScanSegment()); 
        
        //check how many points 
        next_jumps_it = jumps_it;
        next_jumps_it ++; 
        num_points = (*next_jumps_it) - (*jumps_it);
        
        //fill points
        _segment_list.back().points_.resize(3,num_points);
        for ( unsigned int ii=0; ii < num_points; ii++)
        {
            _segment_list.back().points_.block<3,1>(0,ii) << this->points_.block<3,1>(0,(*jumps_it)+ii);
        }
    }
}

}//namespace


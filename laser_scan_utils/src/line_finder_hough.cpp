#include "line_finder_hough.h"

namespace laserscanutils
{

LineFinderHough::LineFinderHough() : LineFinder()
{

}

LineFinderHough::~LineFinderHough()
{

}

void LineFinderHough::setHoughParams(const LineFinderHoughParams & _params)
{
    //copy provided params
    this->hough_params_ = _params; 
    
    //resize hough grid accordingly
    hough_grid_rows = (unsigned int)ceil(M_PI/hough_params_.theta_step_);//theta in [0,PI]
    hough_grid_rows_half = (unsigned int)ceil(0.5*M_PI/hough_params_.theta_step_);//middle row index
    hough_grid_cols = (unsigned int)ceil(2*hough_params_.range_max_/hough_params_.range_step_);//[-rmax,+rmax]
    hough_grid_cols_half = (unsigned int)ceil(hough_params_.range_max_/hough_params_.range_step_);//middle col index
    hough_grid_.resize(hough_grid_rows);
    for (unsigned int ii = 0; ii < hough_grid_rows; ii++)
    {
        hough_grid_[ii].resize(hough_grid_cols);
    }
}

unsigned int LineFinderHough::findLines( const Eigen::MatrixXs & _points, 
                                         std::list<laserscanutils::LineSegment> & _line_list)
{
    ScalarT theta, range, max_value; 
    unsigned int ii_max_value, jj_max_value; 
    int kr; 
    LineSegment line;
    ScalarT xmax, xmin, ymax, ymin;
    Eigen::Vector3s q_pt, p_pt; //auxiliary points
    Eigen::MatrixXs pts; //auxiliar array of points
    std::list<Eigen::Vector3s>::iterator pts_it; 
    Eigen::SparseMatrix<unsigned int> sum_grid(hough_grid_rows,hough_grid_cols); 
    std::list<Eigen::SparseMatrix<unsigned int> >::iterator subgrid_it; 
    Eigen::MatrixXs support_points;
    std::vector<Eigen::Triplet<unsigned int> > triplet_vector;
        
    //clear the Houhg Grid
    hough_grid_x_.clear(); 
    
    
    //STEP 1. For each scan point, accumulate hits in the Hough Grid
    for (unsigned int ipt = 0; ipt < _points.cols(); ipt++) //loop over all points
    {
        //push_back a new Sparse Matrix to hough_grid_. Each SparseMatrix represents the grid for one point.
        hough_grid_x_.push_back(Eigen::SparseMatrix<unsigned int>(hough_grid_rows,hough_grid_cols));
        
        //clear the triplet_vector
        triplet_vector.clear();
        
        //reserve space for triplet vector
        triplet_vector.reserve(hough_grid_rows*2);
        
        //loop over all theta values in the grid
        for (unsigned int jth = 0; jth < hough_grid_rows; jth++) 
        {
            //compute Real values of theta and range
            theta = jth*hough_params_.theta_step_;
            range = _points(0,ipt)*cos(theta) + _points(1,ipt)*sin(theta); //r=xcos(th)+ysin(th)
            
            //discretize range
            kr = (int)floor(range/hough_params_.range_step_) + (int)hough_grid_cols_half ;
            
            //check validity of the discretized values
            if( ( kr >= 0 ) && ( kr < hough_grid_cols ) )
            {
                //Anotate the support of point ipt to cell (jth,kr)
                triplet_vector.push_back(Eigen::Triplet<unsigned int>(jth,kr,1));
            }
        }
        hough_grid_x_.back().setFromTriplets(triplet_vector.begin(), triplet_vector.end());
    }

    ////STEP 2 .Find cells having a peak of at least min_supports_ points supporting them
    while ( hough_grid_x_.size() > hough_params_.min_supports_ )
    {
        //sum all subgrids
        sum_grid.setZero();
        for ( subgrid_it = hough_grid_x_.begin(); subgrid_it != hough_grid_x_.end(); subgrid_it++ )
        {
            sum_grid = sum_grid + (*subgrid_it);
        }
        
        //Find the max_value at sum_grid -> r,th . Run over the Sparse Matrix sum_grid
        max_value = 0; 
        for (int ii=0; ii<sum_grid.outerSize(); ii++)
        {
            for (Eigen::SparseMatrix<unsigned int>::InnerIterator it(sum_grid,ii); it; ++it)
            {
                if ( it.value() > max_value )
                {
                    max_value = it.value();
                    ii_max_value = it.row(); 
                    jj_max_value = it.col(); 
                }
            }
        }        
        
        //max_value is the number of supporters for the most supported cell. Ceck if it has enough supporters
        if ( max_value < hough_params_.min_supports_ )
        {
           break; //Exit while(), since the max_value does not get the min_supports_
        }
        else
        {
            //resize supporter set
            support_points.resize(3,max_value);
            
            //select points supporting the cell of max value 
            unsigned int ipt = 0; //index over _points
            unsigned int jpt = 0; //index over support_points
            for (ipt = 0, subgrid_it = hough_grid_x_.begin(); ipt < _points.cols(), subgrid_it != hough_grid_x_.end(); ipt++ )
            {
                if ( subgrid_it->coeff(ii_max_value,jj_max_value) != 0 ) //point ipt hit this cell
                {
                    //copy ipt point to support_points
                    support_points.block<3,1>(0,jpt) << _points.block<3,1>(0,ipt);
                    jpt++; 
                    
                    //remove this subgrid
                    subgrid_it = hough_grid_x_.erase(subgrid_it); 
                }
                else //point ipt didn't hit the cell, just carry on
                {
                    subgrid_it ++;
                }
            }
            
            //fit a line with the supporter points (support_points)
            this->fitLine(support_points, line); 
            
            //set the line hough params TODO: Compute them from fitLine result, not from the Grid !!
            line.np_ = max_value; //supporters
            line.theta_ = ii_max_value*hough_params_.theta_step_; //theta
            line.range_ = jj_max_value*hough_params_.range_step_; //range

            //set starting and ending points of the line 
            //TODO: apply convention: (point_first_ - point_last_ , 0)^T x (a/c, b/c, 0)^T = UP ! (z>0)
            xmax=-100; xmin=100; ymax=-100; ymin=100; 
            for (jpt = 0; jpt<support_points.cols(); jpt++ )
            {
                //find xmax, xmin, ymax, ymin
                if (support_points(0,jpt) > xmax) xmax = support_points(0,jpt); 
                if (support_points(1,jpt) > ymax) ymax = support_points(1,jpt); 
                if (support_points(0,jpt) < xmin) xmin = support_points(0,jpt); 
                if (support_points(1,jpt) < ymin) ymin = support_points(1,jpt); 
            }
            if (ii_max_value < hough_grid_rows_half) //theta in [0,PI/2]  
            {
                q_pt << xmin,ymax,1;
                line.pointProjectionToLine(q_pt, p_pt); 
                line.point_first_ << p_pt; 
                q_pt << xmax,ymin,1;
                line.pointProjectionToLine(q_pt, p_pt); 
                line.point_last_ << p_pt; 
            }
            else //theta in [PI/2,PI]
            {
                q_pt << xmin,ymin,1;
                line.pointProjectionToLine(q_pt, p_pt); 
                line.point_first_ << p_pt; 
                q_pt << xmax,ymax,1;
                line.pointProjectionToLine(q_pt, p_pt); 
                line.point_last_ << p_pt; 
            }
            
            //push back the line to the list
            _line_list.push_back(line);            
        }
    }//closes while    
    
    //return the number of lines detected
    return _line_list.size(); 
}

void LineFinderHough::print() const
{
    //TODO
}

}//namespace



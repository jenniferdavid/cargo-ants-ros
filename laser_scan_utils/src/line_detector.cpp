
#include "line_detector.h"

void laserscanutils::ExtractLineParams::print() const
{
    std::cout << "Extract Line Algorithm Parameters : " << std::endl 
                << "   jump_dist_ut_: " << jump_dist_ut_ << std::endl
                << "   jump_angle_ut_: " << jump_angle_ut_ << std::endl
                << "   window_length: " << window_length_ << std::endl
                << "   k_sigmas_ut_: " << k_sigmas_ut_ << std::endl
                << "   concatenate_ii_ut_: " << concatenate_ii_ut_ << std::endl
                << "   concatenate_angle_ut_: " << concatenate_angle_ut_ << std::endl;
}

void laserscanutils::fitLine(const Eigen::MatrixXs & _points, Line & _line)
{
    //build the system
    Eigen::Matrix3s AA = _points * _points.transpose();
    AA.row(2) << 0,0,1;

    //solve for line
    _line.vector_ = AA.inverse().col(2);

    // normalize the line
    //std::cout << "line squaredNorm (before) " << _line.vector_.head(2).squaredNorm() << std::endl;
    _line.vector_ /= _line.vector_.head(2).norm();
    //std::cout << "line squaredNorm " << _line.vector_.head(2).squaredNorm() << std::endl;

    // compute fitting error
    _line.error_ = (_points.transpose() * _line.vector_).squaredNorm() / _points.cols(); //_line.error_ = (_points.transpose() * _line.vector_).array().abs().sum() / (_line.vector_.head(2).norm()*_points.cols());
}

unsigned int laserscanutils::extractLines(const laserscanutils::ScanParams & _params, 
                                          const ExtractLineParams & _alg_params, 
                                          const std::vector<float> & _ranges, 
                                          std::list<laserscanutils::Line> & _line_list)
{
    //local variables
    Eigen::MatrixXs points;
    ScalarT cos_theta, theta;
    Line line;
    std::list<Line>::iterator line_it1, line_it2, last_it;
    std::list<unsigned int> jumps;
    std::vector<float> ranges(_ranges);
    std::list<unsigned int>::iterator jj_jump;
    bool update_iterators;
    unsigned int ii, window_points;

    //STEP 1: transform to euclidean coordinates and detect jumps
    ranges2xy(_params, ranges, _alg_params.jump_dist_ut_, _alg_params.jump_angle_ut_, points, jumps);
//    std::cout << "jumps: " << jumps.size() << std::endl;
//    std::cout << "points: " << points.rows() << "x" << points.cols()<< std::endl;
    
    // STEP 2: find line segments running over the scan
    ii = 0;
    //ii2=_alg_params.window_sz_;
    window_points = windowPoints(_params, _alg_params, _ranges, ii);
    jj_jump = jumps.begin();


    while ( ii+window_points < points.cols() )
    {
        //check if a jump exists between ii and ii+window_points
        if ( jj_jump != jumps.end()  &&  ii < (*jj_jump) && (ii+window_points >= (*jj_jump)) )
        {
            ii = *jj_jump;
            jj_jump++;
        }
        else
        {
            //Found the best fitting line over points within the window [ii,ii+window_points]
            fitLine(points.block(0,ii,3,window_points), line);

            //if error below stdev, add line to ScalarT cornerAperture(const Eigen::Vector3s & _p1, const Eigen::Vector3s & _c, const Eigen::Vector3s & _p2);result set
            if ( line.error_ < _params.range_std_dev_*_params.range_std_dev_*_alg_params.k_sigmas_ut_*_alg_params.k_sigmas_ut_ )
            {
                line.first_ = ii;
                line.last_ = ii + window_points;
                line.point_first_ = points.col(line.first_);
                line.point_last_ = points.col(line.last_);
                line.np_ = line.last_ - line.first_ + 1;
                _line_list.push_back(line);
            }

            //increment window iterators
            ii++;
        }
        window_points = windowPoints(_params, _alg_params, _ranges, ii);
        //std::cout << "min window points: " << _alg_params.min_window_points_ << std::endl;
        //std::cout << "range: " << ranges[ii] << std::endl;
        //std::cout << "l_step: " << _params.angle_step_ * ranges[ii] << std::endl;
        //std::cout << "window_sz: " << _alg_params.window_sz_ << std::endl;
        //std::cout << "window size / l_step: " << (_alg_params.window_sz_ / (_params.angle_step_ * ranges[ii])) << std::endl;
        //std::cout << "(unsigned int) window size / l_step: " << (unsigned int)(_alg_params.window_sz_ / (_params.angle_step_ * ranges[ii])) << std::endl;
        //std::cout << "window_points: " << window_points << std::endl << std::endl;
    }
    //std::cout << "Lines fitted: " << _line_list.size() << std::endl;

    //STEP 3: concatenate lines
    if ( _line_list.size() < 2 ) return _line_list.size(); //In case just less than two lines found, return
    line_it1 = _line_list.begin();
    line_it2 = line_it1;
    line_it2++;
    last_it = _line_list.end();
    last_it--;
    jj_jump = jumps.begin();
    while (line_it1 != last_it && line_it1 != _line_list.end())
    {
    	while (jj_jump != jumps.end() && line_it1->first_ > (*jj_jump))
    		jj_jump++;

        // check number of beams and jumps between lines
        if ( ( (int)line_it2->first_ - (int)line_it1->last_ ) <=  (int)_alg_params.concatenate_ii_ut_  && //not too many points between lines
        	  !(jj_jump != jumps.end()  &&  line_it1->first_ < (*jj_jump) && line_it2->last_ >= (*jj_jump))	) //not jumps between lines
        {
			//compute angle between lines 1 and 2
			theta = angleBetweenLines(*line_it1, *line_it2);

			//Check angle threshold
			if ( theta < _alg_params.concatenate_angle_ut_ ) //fabs(theta) not required since theta>0
			{
				//compute a new line with all involved points
				Line new_line;
				fitLine(points.block(0,line_it1->first_, 3, line_it2->last_-line_it1->first_+1), new_line);

				//check if error below a threshold to effectively concatenate
				if ( new_line.error_ < _params.range_std_dev_*_alg_params.k_sigmas_ut_ )
				{
					//update line1 as the concatenation of l1 and l2
					new_line.first_ = line_it1->first_;
					new_line.last_ = line_it2->last_;
					new_line.point_first_ = line_it1->point_first_;
					new_line.point_last_ = line_it2->point_last_;
					new_line.np_ = new_line.last_ - new_line.first_ + 1;

					//assign new values to line 1. Remove line 2
					*line_it1 = new_line;
					line_it2 = _line_list.erase(line_it2);//iterator line_it2 points to the next line of the list
					//Update iterator to last element
					last_it = _line_list.end();
					last_it--;
					//std::cout << "lines concatenated " << _line_list.size() << std::endl;
				}
				else
					line_it2++;
			}
			else
				line_it2++;

			if (line_it2 == _line_list.end()) //no more lines to be checked with line 1, next line 1 (and 2 is the next one)
			{
				//std::cout << "no more lines to be checked with line 1, next line 1 (and 2 is the next one) " << _line_list.size() << std::endl;
				line_it1++;
				line_it2 = line_it1;
				line_it2++;
			}
        }
        else // lines not close enought or jumps between, next line 1 (and 2 is the next one)
        {
            //std::cout << "lines not close enought, next line 1 (and 2 is the next one) " << _line_list.size() << std::endl;
            line_it1++;
            line_it2 = line_it1;
            line_it2++;
        }
    }
    //std::cout << "Lines after concatenation: " << _line_list.size() << std::endl;

    //STEP 4: removing outliers
    for (line_it1 = _line_list.begin(); line_it1 != _line_list.end(); line_it1++)
    {
        // Remove front and back points with high error
        while (line_it1->last_ - line_it1->first_ > 1 && pow(points.col(line_it1->first_).transpose() * line_it1->vector_,2) > 2 * line_it1->error_)
            line_it1->first_++;

        while (line_it1->last_ - line_it1->first_ > 1 && pow(points.col(line_it1->last_).transpose() * line_it1->vector_,2) > 2 * line_it1->error_)
            line_it1->last_--;

        if (line_it1->last_ - line_it1->first_ +1 < line_it1->np_) // Only if any point removed
        {
            if (line_it1->last_ - line_it1->first_ < _alg_params.min_window_points_)
            {
                //std::cout << "line too short after removing outliers!" << std::endl;
                line_it1 = _line_list.erase(line_it1);//iterator line_it points to the next line of the list
                line_it1--;
            }
            else
            {
                //std::cout << "updating line after removing " << line_it1->np_ - (line_it1->last_ - line_it1->first_ +1) << " outliers!" << std::endl;
                //update line
                Line new_line;
                new_line.first_ = line_it1->first_;
                new_line.last_ = line_it1->last_;
                new_line.point_first_ = points.col(line_it1->first_);
                new_line.point_last_ = points.col(line_it1->last_);
                new_line.np_ = new_line.last_ - new_line.first_ + 1;
                fitLine(points.block(0,new_line.first_, 3, new_line.np_), new_line);
                *line_it1 = new_line;
            }
        }
    }


    return _line_list.size();
}


unsigned int laserscanutils::extractLinesHough( const Eigen::MatrixXd & _points,
                                                const ExtractLinesHoughParams & _alg_params, 
                                                std::list<laserscanutils::Line> & _line_list )
{
    double theta, range; 
    int kr; 
    Line line;
    double xmax, xmin, ymax, ymin;
    
    //A 2D array of lists. Each cell is a (r,theta) discretization in the line parameter space.
    //Each list keeps the laser point coordinates that support that cell
    std::vector<std::vector<std::list<std::pair<double,double> > > > hough_grid; //TODO: to be created/sized outside and required as a param
    std::list<std::pair<double,double> >::iterator pt_it; //iterator over the points of a given cell list
    
    //resize hough_grid according range and theta steps and bounds
    unsigned int hough_grid_rows = (unsigned int)ceil(M_PI/_alg_params.theta_step_);//[0,PI]
    unsigned int hough_grid_rows_half = (unsigned int)ceil(0.5*M_PI/_alg_params.theta_step_);//middle row index
    unsigned int hough_grid_cols = (unsigned int)ceil(2*_alg_params.range_max_/_alg_params.range_step_);//[-rmax,+rmax]
    unsigned int hough_grid_cols_half = (unsigned int)ceil(_alg_params.range_max_/_alg_params.range_step_);//middle col index
    
    //clear line list 
    //_line_list.clear(); //TODO: TO BE DONE OUTSIDE!!
    
    //Set grid dimensions
    hough_grid.resize(hough_grid_rows);
    for (unsigned int ii = 0; ii < hough_grid_rows; ii++)
    {
        hough_grid[ii].resize(hough_grid_cols);
    }
    
    //For each input point, register all the supports to the hough_grid
//     for (unsigned int laser_id = 0; laser_id < _points.size(); laser_id++) //loop on the laser devices
//     {
    //for (unsigned int ipt = 0; ipt < _points.at(laser_id).cols(); ipt++) //loop over all points of laser_id
    for (unsigned int ipt = 0; ipt < _points.cols(); ipt++) //loop over all points
    {
        for (unsigned int jth = 0; jth < hough_grid_rows; jth++) //loop over all theta values in the grid
        {
            //compute Real values of theta and range
            theta = jth*_alg_params.theta_step_;
            range = _points(0,ipt)*cos(theta) + _points(1,ipt)*sin(theta); //r=xcos(th)+ysin(th)
            
            //discretize range
            kr = (int)floor(range/_alg_params.range_step_) + (int)hough_grid_cols_half ;
            
            //check validity of the discretized values
            if( ( kr >= 0 ) && ( kr < hough_grid_cols ) )
            {
                //Add support to cell(jth,kr), by pushing back the point coordinates
                //hough_grid.at(jth).at(kr).push_back( std::pair<double,double>(_points.at(laser_id)(0,ipt),_points.at(laser_id)(1,ipt)) );
                hough_grid.at(jth).at(kr).push_back( std::pair<double,double>(_points(0,ipt),_points(1,ipt)) );
            }
        }
    }
//     }
    
    //Find cells having a peak of at least min_supports_ points supporting them
    for (unsigned int ii = 1; ii < hough_grid_rows-1; ii++)
    {
        for (unsigned int jj = 1; jj < hough_grid_cols-1; jj++)
        {
            //check min supports
            unsigned int cell_supports = hough_grid.at(ii).at(jj).size();
            if( cell_supports >= _alg_params.min_supports_ )
            {
                //check if cell ii,jj is a peak ( 8 neighboring cells should be below in number of supports)
                if ( ( cell_supports >= hough_grid.at(ii-1).at(jj-1).size() ) &&
                     ( cell_supports >= hough_grid.at(ii-1).at(jj).size() ) &&  
                     ( cell_supports >= hough_grid.at(ii-1).at(jj+1).size() ) &&
                     ( cell_supports >= hough_grid.at(ii).at(jj-1).size() ) && 
                     ( cell_supports >= hough_grid.at(ii).at(jj+1).size() ) &&
                     ( cell_supports >= hough_grid.at(ii+1).at(jj-1).size() ) &&
                     ( cell_supports >= hough_grid.at(ii+1).at(jj).size() ) &&
                     ( cell_supports >= hough_grid.at(ii+1).at(jj+1).size() ) )
                {                    
                    //find best fitting line with the supporting points
                    //TODO //run over supporters indexes of the ii,jj cell, and build a Eigen::Matrix3s
                    //Eigen::Matrix3s supporters; 
                    //supporters.resize(3,...)
                    //supporters << fill with point data
                    //call fitLine(supporters, line); 
                 
                    //set the line hough params TODO: Compute them from fitLine result, not from the Grid !!
                    line.np_ = hough_grid.at(ii).at(jj).size(); //supporters
                    line.theta_ = ii*_alg_params.theta_step_; //theta
                    line.range_ = jj*_alg_params.range_step_; //range

                    //set starting and ending points of the line TODO: Set the projection of min max points to the fitLine line                    
                    xmax=-100; xmin=100; ymax=-100; ymin=100; 
                    for (pt_it = hough_grid.at(ii).at(jj).begin(); pt_it != hough_grid.at(ii).at(jj).end(); pt_it++)
                    {
                        //find xmax, xmin, ymax, ymin
                        if (pt_it->first > xmax) xmax = pt_it->first; 
                        if (pt_it->second > ymax) ymax = pt_it->second; 
                        if (pt_it->first < xmin) xmin = pt_it->first; 
                        if (pt_it->second < ymin) ymin = pt_it->second; 
                    }
                    if (ii < hough_grid_rows_half) //first and third quartile of r-theta plane
                    {
                        line.point_first_ << xmin,ymax,1;
                        line.point_last_ << xmax,ymin,1;
                        
                    }
                    else //second and fourth quartile of r-theta plane
                    {
                        line.point_first_ << xmin,ymin,1;
                        line.point_last_ << xmax,ymax,1;                    
                    }
                    
                    //push back the line to the list
                    _line_list.push_back(line);
                }
            }
        }
    }

    //return the number of lines detected
    return _line_list.size(); 
}
    

laserscanutils::ScalarT laserscanutils::angleBetweenLines(const laserscanutils::Line& line1, const laserscanutils::Line& line2)
{
    // lines should be normalized
    assert(abs(line1.vector_.head(2).squaredNorm() - 1) < 1e-9 && "line 1 is not normalized!");
    assert(abs(line2.vector_.head(2).squaredNorm() - 1) < 1e-9 && "line 2 is not normalized!");

    //compute angle, result in [0,PI]
    ScalarT theta = acos( (line1.vector_.head(2)).dot(line2.vector_.head(2)) );

    //returns the angle lower PI/2 of the crossing lines
    if (theta > M_PI/2) theta = M_PI - theta;

    //theta is in [0,PI/2]
    return theta;
}

unsigned int laserscanutils::windowPoints(const laserscanutils::ScanParams & _params,
										  const ExtractLineParams & _alg_params,
										  const std::vector<float> & _ranges,
										  unsigned int& ii)
{
	if (ii < _ranges.size() && _ranges[ii] !=0)
		return std::max(_alg_params.min_window_points_, (unsigned int)(_alg_params.window_length_ / (_params.angle_step_ * _ranges[ii])));
	else
		return _alg_params.min_window_points_;
}

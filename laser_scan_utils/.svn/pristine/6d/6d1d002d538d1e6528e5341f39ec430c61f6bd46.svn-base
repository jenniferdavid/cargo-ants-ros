
#include "corner_detector.h"

void laserscanutils::ExtractCornerParams::print() const
{
    line_params_.print(); 
    std::cout << "Extract Corner Algorithm Parameters : " << std::endl 
                << "   theta_min_: " << theta_min_ << std::endl
                << "   max_distance_: " << max_distance_ << std::endl;
}

unsigned int laserscanutils::extractCorners(const laserscanutils::ScanParams & _params, 
                                            const laserscanutils::ExtractCornerParams & _alg_params, 
                                            const std::list<laserscanutils::Line> & _line_list, 
                                            std::list<laserscanutils::Corner> & _corner_list)
{
  //local variables
  ScalarT cos_theta, theta;
  Corner corner;
  std::list<Line>::const_iterator line_it1, line_it2, last_line;
  std::list<laserscanutils::Corner>::iterator corner_it1, corner_it2;
  bool update_iterators;
  ScalarT max_distance_sq(_alg_params.max_distance_*_alg_params.max_distance_); //init max distance as the squared value to avoid sqroot on comparisons

    //STEP 1:  find corners over the line list
    line_it1 = _line_list.begin();
    while ( line_it1 != _line_list.end() )
    {
        line_it2 = line_it1;
        line_it2 ++;
        while (line_it2 != _line_list.end())
        {

            // check if last of line1 and first of line2 are within the threshold
            if ( ( (int)line_it2->first_ - (int)line_it1->last_ ) <= (int)_alg_params.line_params_.concatenate_ii_ut_ )
            {
                //compute angle between lines 1 and 2
                theta = angleBetweenLines(*line_it1, *line_it2);

                //Check angle threshold
                if ( theta > _alg_params.theta_min_ )
                {
                    // Find the corner as the cross product between lines (intersection)
                    corner.pt_ = (line_it1->vector_).cross(line_it2->vector_);

                    // normalize corner point to set last component to 1
                    corner.pt_ = corner.pt_ / corner.pt_(2);

                    // Check if the corner is close to both lines ends.
                    if ( (line_it1->point_last_ - corner.pt_).head(2).squaredNorm() < max_distance_sq
                            &&
                        (line_it2->point_first_ - corner.pt_).head(2).squaredNorm() < max_distance_sq )
                    {

                        //vector from corner to first point of l1
                        Eigen::Vector2s v1 = line_it1->point_first_.head(2) - corner.pt_.head(2);
                        Eigen::Vector2s v2 = line_it2->point_last_.head(2) - corner.pt_.head(2);

                        //compute corner orientation as the angle of the bisector w.r.t local frame
                        corner.orientation_ = ( atan2(v1(1),v1(0)) + atan2(v2(1),v2(0)) ) / 2;

                        //Compute corner aperture with line_it1->first, corner and line_it2->last
                        corner.aperture_ = cornerAperture(line_it1->point_first_, corner.pt_, line_it2->point_last_);

                        //set other descriptors
                        corner.line_1_ = *line_it1;
                        corner.line_2_ = *line_it2;

                        //add the corner to the list
                        _corner_list.push_back(corner);
                    }
                }
            }
            else
            {
                break;
            }

            line_it2 ++;
        }
        
        line_it1 ++;
    }
    // std::cout << "Corners extracted: " << _corner_list.size() << std::endl;

    // Erase duplicated corners
    corner_it1 = _corner_list.begin();
    while ( corner_it1 != _corner_list.end() )
    {
        corner_it2 = corner_it1;
        corner_it2 ++;
        while (corner_it2 != _corner_list.end())
        {
        // Check if two corners are close enough with similar orientation and aperture
        if ( ( corner_it1->pt_ - corner_it2->pt_ ).head(2).squaredNorm() < max_distance_sq &&
             abs( pi2pi( corner_it1->aperture_ - corner_it2->aperture_) ) < _alg_params.theta_min_ &&
             abs( pi2pi( corner_it1->orientation_ - corner_it2->orientation_) ) < _alg_params.theta_min_)
        {
            // Keep the one with bigger product of number of points of each line
            (*corner_it1) = (corner_it1->line_1_.np_*corner_it1->line_2_.np_ > corner_it2->line_1_.np_*corner_it2->line_2_.np_? *corner_it1 : *corner_it2);
            corner_it2 = _corner_list.erase(corner_it2);
        }
        else
            corner_it2 ++;
        }
        corner_it1 ++;
    }
    // std::cout << "Corners after removing duplicates: " << _corner_list.size() << std::endl;

    return _corner_list.size();
}

unsigned int laserscanutils::extractCorners(const laserscanutils::ScanParams & _params, 
                                            const laserscanutils::ExtractCornerParams & _alg_params, 
                                            const std::vector<float> & _ranges, 
                                            std::list<laserscanutils::Corner> & _corner_list)
{
    std::list<laserscanutils::Line> line_list;
    unsigned int nl; 
    
    //extract lines and fill the list
    nl = extractLines(_params, _alg_params.line_params_, _ranges, line_list);
//     std::cout << "line_list.size(): " << line_list.size() << std::endl;

    //If more than one line, call to extract corners with the line list as input parameter
    if ( nl > 1) 
    {
        return extractCorners(_params, _alg_params, line_list, _corner_list); //return detected corners and fill _corner_list
    }
    else 
        return 0;
    
}


laserscanutils::ScalarT laserscanutils::cornerAperture(const Eigen::Vector3s & _p1, 
                                                       const Eigen::Vector3s & _c, 
                                                       const Eigen::Vector3s & _p2)
{
    //TODO: use lines as inputs instead of _p1 and _p2 -> better estimation of the aperture value
    
    //buid lines from c to p1 and p2
    Eigen::Vector3s l1 = _c.cross(_p1);
    Eigen::Vector3s l2 = _c.cross(_p2);
    
    //normalize lines
    l1 = l1 / ( sqrt( l1(0)*l1(0)+l1(1)*l1(1) ) );
    l2 = l2 / ( sqrt( l2(0)*l2(0)+l2(1)*l2(1) ) );
    
    //compute angle
    ScalarT aux = l1(0)*l2(0) + l1(1)*l2(1); 
    if (fabs(aux) > 1.) aux = aux / fabs(aux); //limit here to +-1 to avoid complex numbers due to numerical precision when acos(>1). Anyway, corners will be far from this situation
    ScalarT alpha = acos(aux); //alpha in [0,pi]
    
    //build matrix M and compute its determinant
    Eigen::Matrix3s M;
    M << _p1,_c, _p2;
    ScalarT detM = M.determinant(); 
    
    //if det<0, return the complementary angle
    if ( detM < 0 ) return ( 2*M_PI - alpha );
    else return alpha;    
}

/*
laserscanutils::ScalarT laserscanutils::angleBetweenLines(const laserscanutils::Line& line1, const laserscanutils::Line& line2)
{
  ScalarT theta = acos( (line1.vector_.head(2)).dot(line2.vector_.head(2)) / ( line1.vector_.head(2).norm()*line2.vector_.head(2).norm() ) );

  //theta is in [0,PI]
  if (theta > M_PI/2)
    theta = M_PI - theta;//assures theta>0 AND either theta close to 0 or close to PI will always map to theta close to 0.

  return theta;
}*/


/*
unsigned int laserscanutils::extractLines(const laserscanutils::ScanParams & _params, const ExtractCornerParams & _alg_params, const std::vector<ScalarT> & _ranges, std::list<laserscanutils::Line> & _line_list)
{
    //local variables
    Eigen::MatrixXs points
    ScalarT cos_theta, theta;
    Line line;
    std::list<Line>::iterator line_it1, line_it2;
    std::list<unsigned int> jumps;
    bool update_iterators;
    unsigned int ii_from; 
    ScalarT max_distance_sq(_alg_params.max_distance*_alg_params.max_distance); //init to squared value to avoid root() at comparisons

  //init random generator
//   std::default_random_engine generator(1);
//   std::uniform_int_distribution<int> rand_window_overlapping(1,_alg_params.segment_window_size);

    //laserPolar2xy(_params, _ranges, _alg_params.max_distance, points, jumps);
    ranges2xy(_params, _ranges, _alg_params.max_distance, points, jumps);
//  std::cout << "jumps: " << jumps.size() << std::endl;
//  std::cout << "points: " << points.rows() << "x" << points.cols()<< std::endl;
    
    
    // STEP 1: find line segments running over the scan
    //for (unsigned int ii = _alg_params.segment_window_size-1; ii<points.cols(); ii++)//ii=ii+rand_window_overlapping(generator) )
    for (unsigned int ii=0, ii_from=0; ii<points.cols(); ii++)//ii=ii+rand_window_overlapping(generator) )
    {
        //unsigned int i_from = ii - _alg_params.segment_window_size + 1;
        
        //if current point (ii) still close to ii_from, jump to next ii
        if ( (points.col(ii_from)-points.col(ii)).squaredNorm() < max_distance_sq ) continue; //jump to for loop again
        
        //advance ii_from up to being closer than max_distance_sq
        while ( (points.col(ii_from+1)-points.col(ii)).squaredNorm() > max_distance_sq ) ii_from ++;
                    
        // update the jump to be checked
        while (!jumps.empty() && ii_from > jumps.front()) jumps.pop_front();

        // check if there is a jump inside the window (not fitting if it is the case)
        if ( (jumps.front() > ii_from) && (jumps.front() <= ii) ) continue;

        //Found the best fitting line over points within the window [ii - segment_window_size + 1, ii]
        fitLine(points.block(0,ii_from, 3, _alg_params.segment_window_size), line);

        //if error below stdev, add line to ScalarT cornerAperture(const Eigen::Vector3s & _p1, const Eigen::Vector3s & _c, const Eigen::Vector3s & _p2);result set
        if ( line.error_ < _params.range_std_dev_*_alg_params.k_sigmas )
        {
            line.first_ = i_from;
            line.last_ = ii;
            line.point_first_ = points.col(line.first_);
            line.point_last_ = points.col(line.last_);
            line.np_ = line.last_ - line.first_ + 1;
            _line_list.push_back(line);
        }
    }
//  std::cout << "Lines fitted: " << _line_list.size() << std::endl;

  //In case just less than two lines found, return with 0 corners
  if ( _line_list.size() < 2 ) return 0;

  //STEP 2: concatenate lines
  line_it1 = _line_list.begin();
  while ( line_it1 != _line_list.end() )
  {
    line_it2 = line_it1;
    line_it2 ++;
    while (line_it2 != _line_list.end())
    {
//      std::cout << std::endl << "line 1: first idx: " << (int)line_it1->last_ << std::endl;
//      std::cout << "line 2: first idx: " << (int)line_it2->first_ << std::endl;
//      std::cout << "substraction " << (int)line_it2->first_ - (int)line_it1->last_ << std::endl;

      // check if last of line1 and first of line2 are within the threshold
      if ( ( (int)line_it2->first_ - (int)line_it1->last_ ) <=  (int)_alg_params.max_beam_distance )
      {
//        std::cout << "line 1: point first: " << points.col(line_it1->first_).transpose() << std::endl;
//        std::cout << "line 1: point last: " << points.col(line_it1->last_).transpose() << std::endl;
//        std::cout << "line 2: point first: " << points.col(line_it2->first_).transpose() << std::endl;
//        std::cout << "line 2: point last: " << points.col(line_it2->last_).transpose() << std::endl;

        //compute angle between lines 1 and 2
        theta = angleBetweenLines(*line_it1, *line_it2);

        //Check angle threshold
        if ( theta < _alg_params.theta_max_parallel ) //fabs(theta) not required since theta>0
        {
            //compute a new line with all involved points
            Line new_line;
            fitLine(points.block(0,line_it1->first_, 3, line_it2->last_-line_it1->first_+1), new_line);

            //std::cout << "new_line.error: " << new_line.error_ << std::endl;

            //check if error below a threshold to effectively concatenate
            if ( new_line.error_ < _params.range_std_dev_*_alg_params.k_sigmas )
            {
                //update line1 as the concatenation of l1 and l2
                new_line.first_ = line_it1->first_;
                new_line.last_ = line_it2->last_;
                new_line.point_first_ = line_it1->point_first_;
                new_line.point_last_ = line_it2->point_last_;
                new_line.np_ = new_line.last_ - new_line.first_ + 1;
                *line_it1 = new_line;

                //remove l2 from the list
                line_it2 = _line_list.erase(line_it2);

//                std::cout << "lines concatenated" << std::endl;
//                std::cout << "line 1: " << std::endl << line_it1->first << std::endl <<
//                     line_it1->last << std::endl << line_it1->vector_.transpose() << std::endl << line_it1->error << std::endl;
//                std::cout << "line 2: " << std::endl << line_it2->first << std::endl <<
//                     line_it2->last << std::endl << line_it2->vector_.transpose() << std::endl << line_it2->error << std::endl;
//                std::cout << "line resultant: "<< std::endl << new_line.first << std::endl <<
//                    new_line.last << std::endl << new_line.vector_.transpose() << std::endl << new_line.error << std::endl;
            }
            else
              line_it2 ++;
        }
        else
          line_it2 ++;
      }
      else
        break;
    }
    line_it1 ++;
  }
  //std::cout << "Lines after concatenation: " << _line_list.size() << std::endl;

  return _line_list.size();
}
*/


/*
//unsigned int laserscanutils::extractCorners(const laserscanutils::ScanParams & _params, const ExtractCornerParams & _alg_params, const Eigen::VectorXs & _ranges, std::list<Eigen::Vector4s> & _corner_list)
//unsigned int laserscanutils::extractCorners(const laserscanutils::ScanParams & _params, const ExtractCornerParams & _alg_params, const Eigen::VectorXs & _ranges, std::list<laserscanutils::Corner> & _corner_list)
unsigned int laserscanutils::extractCorners(const laserscanutils::ScanParams & _params, const ExtractCornerParams & _alg_params, const std::vector<float> & _ranges, std::list<laserscanutils::Corner> & _corner_list)
{
    //local variables
    //Eigen::MatrixXs points(3,_ranges.size());
    ScalarT azimuth, cos_theta, theta;
    //Eigen::Vector3s corner;
    Corner corner;
    Line line;
    std::list<Line> line_list;
    std::list<Line>::iterator line_it1, line_it2;
    std::queue<unsigned int> jumps;
    bool update_iterators; 
    ScalarT max_distance_sq(_alg_params.max_distance_*_alg_params.max_distance_); //init max distance as the squared value to avoid sqroot on comparisons 

    std::cout << "CORNER DETECTION FROM RANGES: " << std::endl;
    //_alg_params.print();

    //init random generator
    std::default_random_engine generator(1);
    std::uniform_int_distribution<int> rand_window_overlapping(1,_alg_params.segment_window_size);

    //STEP 0: convert range polar data to cartesian points, and keep range jumps
//    for (unsigned int ii = 0; ii<_ranges.size(); ii++)
//    {
//        //range 2 polar
//        azimuth = _params.angle_max_ - _params.angle_step_ * ii;
//        points.col(ii) << _ranges(ii)*cos(azimuth), _ranges(ii)*sin(azimuth), 1; //points.row0-> x coordinate, points.row1->y coordinate
//
//        //check range jump and keep it
//        if (ii > 0 && fabs(_ranges(ii)-_ranges(ii-1)) > _alg_params.max_distance_) jumps.push(ii);
//    }
    Eigen::MatrixXs points = laserPolar2xy(_params, _ranges, _alg_params.max_distance_, jumps);
    std::cout << "jumps: " << jumps.size() << std::endl;
    std::cout << "points: " << points.rows() << "x" << points.cols()<< std::endl;

    // STEP 1: find line segments running over the scan
    for (unsigned int ii = _alg_params.segment_window_size-1; ii<points.cols(); ii=ii+rand_window_overlapping(generator) )
    {
        unsigned int i_from = ii - _alg_params.segment_window_size + 1;

        // update the jump to be checked
        while (!jumps.empty() && i_from > jumps.front())
            jumps.pop();

        // check if there is a jump inside the window (not fitting if it is the case)
        if (jumps.front() > i_from && jumps.front() <= ii)
            continue;

        //Found the best fitting line over points within the window [ii - segment_window_size + 1, ii]
        //fitLine(i_from, ii, points, line);
        fitLine(points.block(0,i_from, 3, ii-i_from+1), line);

        //if error below stdev, add line to ScalarT cornerAperture(const Eigen::Vector3s & _p1, const Eigen::Vector3s & _c, const Eigen::Vector3s & _p2);result set
        if ( line.error_ < _params.range_std_dev_*_alg_params.k_sigmas )
        {
            line.first_ = i_from; 
            line.last_ = ii;
            line.point_first_ = points.col(line.first_);
            line.point_last_ = points.col(line.last_);
            line.np_ = line.last_ - line.first_ + 1;
            line_list.push_back(line);
        }
    }
    std::cout << "Lines fitted: " << line_list.size() << std::endl;

    //In case just less than two lines found, return with 0 corners
    if ( line_list.size() < 2 ) return 0; 
    
    //STEP 2: concatenate lines
    line_it1 = line_list.begin();
    line_it2 = line_it1;
    line_it2 ++;
    while ( line_it1 != line_list.end() )
    {
        //if no concatenation found, iterators will run forward
        update_iterators = false;
        
        // check if last of line1 and first of line2 are within the threshold
        if ( ( line_it2->first_ - line_it1->last_ ) <=  _alg_params.line_params_.concatenation_ii_threshold_ )
        {
            //compute angle between lines 1 and 2
            theta = angleBetweenLines(*line_it1, *line_it2);
//            std::cout << std::endl << "cos_theta: " << cos_theta << std::endl << "theta: " << theta << std::endl <<
//                                      "*index_it1: " << *index_it1 << std::endl << "*index_it2: " << *index_it2 << std::endl;
//                                      "   (*line_it1).dot(*line_it2): " << (*line_it1).dot(*line_it2) << std::endl <<
//                                      "   (*line_it1).norm()*(*line_it2).norm(): " << (*line_it1).norm()*(*line_it2).norm() << std::endl;

            //Check angle threshold
            if ( theta < _alg_params.theta_max_parallel ) //fabs(theta) not required since theta>0
            {
                //compute a new line with all involved points
                Line new_line;
                fitLine(points.block(0,line_it1->first_, 3, line_it2->last_-line_it1->first_+1), new_line);
                
                //check if error below a threshold to effectively concatenate
                if ( new_line.error_ < _params.range_std_dev_*_alg_params.k_sigmas )
                {
                    //update line1 as the concatenation of l1 and l2
                    new_line.first_ = line_it1->first_;
                    new_line.last_ = line_it2->last_;
                    new_line.point_first_ = points.col(new_line.first_);
                    new_line.point_last_ = points.col(new_line.last_);
                    new_line.np_ = new_line.last_ - new_line.first_ + 1;
                    *line_it1 = new_line; 
                    
                    //remove l2 from the list
                    line_it2 = line_list.erase(line_it2);
                    
                    //in that case do not update iterators to allow concatenation of the new l1 with the next line in the list, if any
                   // update_iterators = false;
                    
//                      std::cout << "lines concatenated" << std::endl;
//                      std::cout << "line 1: " << std::endl << line_it1->first_ << std::endl <<
//                           line_it1->last_ << std::endl << line_it1->vector_.transpose() << std::endl << line_it1->error_ << std::endl;
//                      std::cout << "line 2: " << std::endl << line_it2->first_ << std::endl <<
//                           line_it2->last_ << std::endl << line_it2->vector_.transpose() << std::endl << line_it2->error_ << std::endl;
//                      std::cout << "line resultant: "<< std::endl << new_line.first_ << std::endl <<
//                          new_line.last_ << std::endl << new_line.vector_.transpose() << std::endl << new_line.error_ << std::endl;
                }
            }
        }
        else
          update_iterators = false;
   
        //update iterators
        if (update_iterators)
        {
            line_it1 ++;
            line_it2 = line_it1;
            line_it2 ++;                    
        }            
    }
    std::cout << "Lines after concatenation: " << line_list.size() << std::endl;

    //STEP 3:  find corners over the line list
    line_it1 = line_list.begin();
    line_it2 = line_it1;
    line_it2 ++;
    while ( line_it1 != line_list.end() )
    {   
        //if no concatenation found, iterators will run forward
        update_iterators = true; 

        // check if last of line1 and first of line2 are within the threshold
        if ( ( line_it2->first_ - line_it1->last_ ) <=  _alg_params.line_params_.concatenation_ii_threshold_ )            
        {
            //compute angle between lines 1 and 2
            theta = angleBetweenLines(*line_it1, *line_it2);
//            cos_theta = (line_it1->vector_.head(2)).dot(line_it2->vector_.head(2)) / ( line_it1->vector_.head(2).norm()*line_it2->vector_.head(2).norm() );
//            theta = acos (cos_theta); //theta is in [0,PI]
//            if (theta > M_PI/2) theta = M_PI - theta;//assures theta>0 AND either theta close to 0 or close to PI will always map to theta close to 0.

//             std::cout << std::endl << "cos_theta: " << cos_theta << std::endl << "theta: " << theta << std::endl <<
//                     "line 1: " << line_it1->first << "-" << line_it1->last << std::endl <<
//                     "line 2: " << line_it2->first << "-" << line_it2->last << std::endl <<
//                     "   (*line_it1).dot(*line_it2): " << (line_it1->vector_).dot(line_it2->vector_) << std::endl <<
//                     "   (*line_it1).norm()*(*line_it2).norm(): " << (line_it1->vector_).norm()*(line_it2->vector_).norm() << std::endl;

            //Check angle threshold
            if ( theta > _alg_params.theta_min_ ) //fabs(theta) not required since theta>0
            {
                // Find the corner ad the cross product between lines (intersection)
                //corner = (line_it1->vector_).cross(line_it2->vector_);
                corner.pt_ = (line_it1->vector_).cross(line_it2->vector_);
                
                // normalize corner point to set last component to 1
                corner.pt_ = corner.pt_ / corner.pt_(2); 
                
                // Check if the corner is close to both lines ends. TODO: Move this check before computing intersection 
                if ( ( (points.col(line_it1->last_) - corner.pt_).head(2).squaredNorm() < max_distance_sq )
                        && 
                   ( (points.col(line_it2->first_) - corner.pt_).head(2).squaredNorm() < max_distance_sq ) )
                {
                    
                    //vector from corner to first point of l1
                    Eigen::Vector2s v1 = points.col(line_it1->first_).head(2) - corner.pt_.head(2);
                    
                    //compute corner orientation as the angle between v1 and local X, in [-pi,pi]
                    corner.orientation_ = atan2(v1(1),v1(0));
                    
                    //Compute corner aperture with line_it1->first, corner and line_it2->last
                    corner.aperture_ = cornerAperture(points.col(line_it1->first_), corner.pt_, points.col(line_it2->last_));
                    
                    //set other descriptors
                    corner.line_1_ = *line_it1;
                    corner.line_2_ = *line_it2;
//                    corner.np_l1_ = line_it1->last_ - line_it1->first_;//>0 assured due to scanning order
//                    corner.np_l2_ = line_it2->last_ - line_it2->first_;//>0 assured due to scanning order
//                    corner.error_l1_ = line_it1->error_;
//                    corner.error_l2_ = line_it2->error_;

                    //add the corner to the list
                    _corner_list.push_back(corner);
                }
            }

            //update iterators //TODO: QUESTION: Are we checking each line against all ? Why if they are find consecutively?
            if (line_it2 != line_list.end() )
            {
                line_it2 ++;
                update_iterators = false; 
            }   
        }
        
        //update iterators
        if (update_iterators)
        {
            line_it1 ++;
            line_it2 = line_it1;
            line_it2 ++;
        }
            
    }
    std::cout << "Corners extracted: " << _corner_list.size() << std::endl;

    // Erase duplicated corners
    if ( _corner_list.size() > 1 )
    {
        // concatenate lines
        std::list<laserscanutils::Corner>::iterator corner_it1 = _corner_list.begin();
        std::list<laserscanutils::Corner>::iterator corner_it2 = corner_it1;
        corner_it2 ++;
        while ( corner_it2 != _corner_list.end() ) 
        {
            // Check if two corners are close enough. TODO: Check othe attributes also (orientation and aperture) !
            if ( ( corner_it1->pt_ - corner_it2->pt_ ).head(2).squaredNorm() < max_distance_sq )
            {
                // Keep the one with bigger product of number of points of each line
                (*corner_it1) = (corner_it1->line_1_.np_*corner_it1->line_2_.np_ > corner_it2->line_1_.np_*corner_it2->line_2_.np_? *corner_it1 : *corner_it2);
                corner_it2 = _corner_list.erase(corner_it2);
            }
            else
            {
                corner_it1 ++;
                corner_it2 = corner_it1;
                corner_it2 ++;
            }
        }
    }
    std::cout << "Corners after removing duplicates: " << _corner_list.size() << std::endl;
    for(std::list<laserscanutils::Corner>::iterator corner_it1 = _corner_list.begin(); corner_it1 != _corner_list.end(); corner_it1++) corner_it1->print(); 

    return _corner_list.size();
}
*/


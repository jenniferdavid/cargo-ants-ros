#include "line_finder.h"

namespace laserscanutils
{
    
LineFinder::LineFinder()
{

}

LineFinder::~LineFinder()
{

}

void LineFinder::fitLine(const Eigen::MatrixXs & _points, LineSegment & _line) const
{
    //build the system
    Eigen::Matrix3s AA = _points * _points.transpose();
    AA.row(2) << 0,0,1;

    //solve for line
    _line.abc_ = AA.inverse().col(2);

    // normalize the line
    _line.abc_ /= _line.abc_.head(2).norm();

    // compute fitting error
    _line.fit_error_ = (_points.transpose() * _line.abc_).squaredNorm() / _points.cols(); 
    //_line.fit_error_ = (_points.transpose() * _line.abc_).array().abs().sum() / (_line.abc_.head(2).norm()*_points.cols());
}

void LineFinder::fitLineRansac(const Eigen::MatrixXs & _points, LineSegment & _line, ScalarT _error) const
{
    std::vector<bool> used_points(_points.cols(),false); //mask, true means used
    std::vector<unsigned int> available_points(_points.cols());
    unsigned int support_counter, best_support_counter; 
    LineSegment line; 
        
    //init best_support_counter
    best_support_counter = 0; 
    
    //  while (attempts < max_attempts)
    //  {
            //init available_points vector with all indexes
            for ( unsigned int ii=0; ii<available_points.size(); ii++ )
            {
                available_points[ii] = ii; 
            }
    //      pick randomly two indexes from available_points
    //      Set the selecte points to MatrixXs P
    //      fitLine(P,line)
    //      support_counter = 0; 
    //      while (available_points.size() > 0 )
    //      {
    //          choose a point q randomly from the index vector available_points
    //          remove that index from available_points
    //          Compute dist between q and line
    //          if (dist < max_error) support_counter ++;
    //      }
    //      
    //      if ( support_counter > best_support_counter )
    //      {
    //          _line = line;
    //          best_support_counter = support_counter; 
    //      }
    //      attempts ++;
    //  }
    
}

// unsigned int laserscanutils::LineFinder::mergeLines(std::list<LineSegment> & _line_list)
// {
//     //TODO
// }

    
}//namespace


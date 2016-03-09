#include "line_finder_jump_fit.h"

namespace laserscanutils
{

LineFinderJumpFit::LineFinderJumpFit()
{

}

LineFinderJumpFit::~LineFinderJumpFit()
{

}

void LineFinderJumpFit::setJumpFitParams(const LineFinderJumpFitParams & _params)
{
    this->jump_fit_params_ = _params; 
}

unsigned int LineFinderJumpFit::findLines( const Eigen::MatrixXs & _points, 
                                           std::list<laserscanutils::LineSegment> & _line_list)
{
    //TODO 
    //Import code from line_detector.h -> laserscanutils::extractLines(), 
    //but remove things considering two main assumptions of this method: 
    // - ordering
    // - points belong to the same segment (no jumps in _points)
    
}

void LineFinderJumpFit::print() const
{
    //TODO
}

}//namespace

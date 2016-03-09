#include "line_segment.h"

namespace laserscanutils
{

LineSegment::LineSegment() :
    abc_(0,0,0),
    idx_first_(0),
    idx_last_(0),
    point_first_(0,0,0),
    point_last_(0,0,0),
    fit_error_(-1.),
    np_(0),
    range_(0.),
    theta_(0.), 
    length_(0.)
{

}

LineSegment::~LineSegment()
{

}

void LineSegment::pointProjectionToLine(const Eigen::Vector3s & _in_pt, Eigen::Vector3s & _out_pt) const
{
    ScalarT a,b,c,qx,qy,dd;
    
    //inits
    a = abc_(0); 
    b = abc_(1); 
    c = abc_(2); 
    qx = _in_pt(0)/_in_pt(2);
    qy = _in_pt(1)/_in_pt(2); 
    
    //compute point projection to line. The formula comes from frocing: 
    //      1) _out_pt to lie on this line
    //      2) lines l2 through _in_pt and _out_pt perpendicular to this -> normal vectors perpendicular
    
    if (a == 0. ) //case a=0 -> vertical line (parallel to y axis)
    {
        _out_pt(0) = _in_pt(0);
        _out_pt(1) = -c/b;
        return;
    }
    
    if ( b == 0. ) //case b=0 -> vertical line (parallel to y axis)
    {
        _out_pt(0) = -c/a;
        _out_pt(1) = _in_pt(1);
        return; 
    }
    
    //general case
    dd = -(a*a + b*b); //denominator. Can't be 0 because either a or b have to be non-zero
    _out_pt(1) = (b*c + a*b*qx - a*a*qy) / dd;
    _out_pt(0) = -( c + b*_out_pt(1) ) / a; 
}


void LineSegment::merge(const LineSegment & _segment)
{
    //TODO
}

void LineSegment::print() const
{
     std::cout << "Line Parameters : " << std::endl 
              << "\t(a,b,c): " << abc_.transpose() << std::endl
              << "\tidx_first_,idx_last_: " << idx_first_ << " , " << idx_last_ << std::endl
              << "\tfirst point: " << point_first_.transpose() << std::endl
              << "\tlast point: " << point_last_.transpose() << std::endl
              << "\terror: " << fit_error_ << std::endl
              << "\tnumber of points: " << np_ << std::endl
              << "\trange: " << range_ << std::endl
              << "\ttheta: " << theta_ << std::endl
              << "\tlength: " << length_ << std::endl;
}

}//namespace

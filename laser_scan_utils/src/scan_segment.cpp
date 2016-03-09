#include "scan_segment.h"

namespace laserscanutils
{

//init static segment counter
unsigned int ScanSegment::segment_id_count_ = 0;
    
ScanSegment::ScanSegment():
    segment_id_(++segment_id_count_)
{

}
        
ScanSegment::~ScanSegment()
{

}
        
void ScanSegment::merge(const ScanSegment & _segment)
{
    //TODO
}

unsigned int ScanSegment::numPoints() const
{
    return points_.cols(); 
}

const Eigen::Vector3s & ScanSegment::getCentroid() const
{
    return centroid_; 
}

const Eigen::Vector3s & ScanSegment::getBBox(unsigned int _corner_idx) const
{
    if (_corner_idx < 4)
        return bbox_corners_[_corner_idx]; 
    else
        return bbox_corners_[0]; 
}

ScalarT ScanSegment::getEvalRatio() const
{
    if ( eval_1_ != 0 )
        return eval_2_/eval_1_; 
    else
        return 1e10;
}

void ScanSegment::computeCentroid()
{
    ScalarT mx=0, my=0; 
    for (unsigned int ii=0; ii<points_.cols(); ii++)
    {
        mx += points_(0,ii); 
        my += points_(1,ii); 
    }

    centroid_ << mx/points_.cols(), my/points_.cols(), 1; 
}

void ScanSegment::computeBoundingBox(const double & _clearance)
{
    double cxx, cyy, cxy; //variance and covariance terms
    Eigen::MatrixXd points_o, points_c; //points wrt origin, points wrt cov eigen vectors
    Eigen::Matrix2d c_mat; //covariance matrix of cluster points
    Eigen::EigenSolver<Eigen::Matrix2d>::EigenvectorsType e_mat; //matrix of eigenvectors (could be complex or real)
    Eigen::EigenSolver<Eigen::Matrix2d>::EigenvalueType evals; //matrix of eigenvectors (could be complex or real)
    Eigen::Matrix2d r_mat; //real velued rotation matrix
    Eigen::Matrix<double, 2,4> corners_c; //corners wrt cov eigen vectors
    Eigen::Matrix<double, 2,4> corners_o; //Final box corners. wrt global axis and translated to cluster centroid
    
    //copy cluster point x,y coordinates to an Eigen Matrix, subtracting centroid coordinates
    points_o.resize(2,points_.cols());
    for (unsigned int ii=0; ii<points_.cols(); ii++) 
    {
        points_o.block<2,1>(0,ii) << (points_(0,ii)-centroid_(0)) , (points_(1,ii)-centroid_(1)); 
    }

    //compute covariance matrix (c_mat)
    cxx = points_o.row(0).dot(points_o.row(0))/points_.cols();
    cyy = points_o.row(1).dot(points_o.row(1))/points_.cols(); 
    cxy = points_o.row(0).dot(points_o.row(1))/points_.cols(); 
    c_mat << cxx,cxy,cxy,cyy; 
    
    //get eigen vectors of c_mat
    Eigen::EigenSolver<Eigen::Matrix2d> e_solver(c_mat);
    e_mat = e_solver.eigenvectors();
    evals = e_solver.eigenvalues();
    
    //keep eigen values. eval_1_ is the largest
    if ( evals(0).real() > evals(1).real() )
    {
        eval_1_ = evals(0).real();
        eval_2_ = evals(1).real();
    }
    else
    {
        eval_1_ = evals(1).real();
        eval_2_ = evals(0).real();
    }
    
    //mount a Real rotation matrix. e_mat is real since c_mat is positive symetric
    r_mat << e_mat(0,0).real(), e_mat(0,1).real(), e_mat(1,0).real(), e_mat(1,1).real();
    
    //rotate all points_o to points_c
    points_c.resize(2,points_.cols());
    points_c = r_mat.inverse()*points_o; 
    
    //get min and max values
    double min_x = points_c.row(0).minCoeff();
    double max_x = points_c.row(0).maxCoeff();
    double min_y = points_c.row(1).minCoeff();
    double max_y = points_c.row(1).maxCoeff();
    
    //set 4 corners of the box wrt c. One corner per column. Order is: top-left, top-right, bottom-left, bottom-right
    corners_c.row(0) << min_x-_clearance,max_x+_clearance,max_x+_clearance,min_x-_clearance; //x coordinates
    corners_c.row(1) << max_y+_clearance,max_y+_clearance,min_y-_clearance,min_y-_clearance; //y coordinates

    //rotate corners to origin frame
    corners_o = r_mat*corners_c; 
    
    //set class member bbox_corners_, adding the translation (centroids) substracted at the beggining of this function
    for (unsigned int ii=0; ii<4; ii++)
    {
        bbox_corners_[ii] << corners_o(0,ii)+centroid_(0), corners_o(1,ii)+centroid_(1), 1;
    }
}

void ScanSegment::computeAll()
{
    computeCentroid();  
    computeBoundingBox();
}
      
void ScanSegment::print() const
{
    //print segment data
    std::cout 
            << "\tsegment_id_: " << segment_id_ << std::endl 
            << "\tcentroid_x_: " << centroid_(0) << std::endl 
            << "\tcentroid_y_: " << centroid_(1)<< std::endl;
}

}//namespace



#include "cluster.h"

//init static cluster counter
unsigned int Cluster::cluster_id_count_ = 0;

Cluster::Cluster() : 
    cluster_id_(++cluster_id_count_), 
    centroid_x_(0),
    centroid_y_(0),
    long_side_(0),
    short_side_(0),
    orientation_(0)
{
	//
}

Cluster::~Cluster()
{
	//
}

void Cluster::computeCentroid()
{
    double mx=0, my=0; 
    for (unsigned int ii=0; ii<points_.size(); ii++)
    {
        mx += points_.at(ii).first; 
        my += points_.at(ii).second; 
    }

    centroid_x_ = mx/points_.size();
    centroid_y_ = my/points_.size();
}

void computeBoundingBox()
{
    
    //eigenvalues(); 
    //eigenvectors(); 
}

void Cluster::print() const
{
    //print cluster params
    std::cout 
            << "\tcluster_id_: " << cluster_id_ << std::endl 
            << "\tcentroid_x_: " << centroid_x_ << std::endl 
            << "\tcentroid_y_: " << centroid_y_ << std::endl 
            << "\tlong_side_: " << long_side_ << std::endl 
            << "\tshort_side_: " << short_side_ << std::endl 
            << "\torientation_: " << orientation_ << std::endl;

    //print cluster cell index pairs
    std::cout << "\tcells_: ";
    for (unsigned int jj = 0; jj < cells_.size(); jj++)
	{
		std::cout << cells_.at(jj).first << "," << cells_.at(jj).second << " - "; 
	}
	std::cout << std::endl; 
    
    //print cluster point pairs
    std::cout << "\tpoints_: ";
    for (unsigned int jj = 0; jj < points_.size(); jj++)
    {
        std::cout << points_.at(jj).first << "," << points_.at(jj).second << " - "; 
    }
    std::cout << std::endl; 
    
}

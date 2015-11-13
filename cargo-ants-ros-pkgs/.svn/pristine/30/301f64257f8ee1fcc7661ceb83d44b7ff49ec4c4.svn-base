#ifndef CLUSTER_H
#define CLUSTER_H

//Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues> 

//std includes
#include <math.h>
#include <iostream>
#include <vector>

//local includes
// #include "grid_2d.h"

/** \brief Cluster class
 * 
 * Cluster of neighboring occupied cells in a grid
 * 
 **/
class Cluster
{
	public: 
        unsigned int cluster_id_; //Cluster id. 
        static unsigned int cluster_id_count_; //cluster counter (acts as simple ID factory)
		std::vector<std::pair<unsigned int, unsigned int> > cells_; //i,j index of the grid cells of this cluster
        std::vector<std::pair<double, double> > points_; //x,y coordinates of the points of this cluster (occuppied points)
		double centroid_x_; //x coordinate of the cluster centroid [m]
		double centroid_y_; //y coordinate of the cluster centroid [m]
		double long_side_; //dimension of the longest side of the bounding box [m]
		double short_side_; //dimension of the shortest side of the bounding box [m]
		double orientation_; //angle beween X grid axis (forward) and the axis aligned with the longest side of the bounding box [rad]
		
	public:	 
		/** \brief Constructor
		 *
		 * Constructor
		 *
		 **/ 
		Cluster(); 

		/** \brief Destructor
		 *
		 * Destructor
		 *
		 **/		
		~Cluster();
		
		/** \brief Compute centroid coordinates
		 *
		 * Compute centroid coordinate.
		 * 		
		 **/		
		void computeCentroid(); 
		
        /** \brief Compute the oriented bounding box
         * 
         * Compute the oriented bounding box
         * 
         **/
		void computeBoundingBox();
        
// 		computeAll(); 
		
		void print() const; 
};
#endif


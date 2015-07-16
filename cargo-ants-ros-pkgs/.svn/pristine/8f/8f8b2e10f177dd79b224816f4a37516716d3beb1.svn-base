#ifndef GRID2D_H
#define GRID2D_H

//Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//std includes
#include <math.h>
#include <iostream>

/** \brief A 2D grid class
 *
 * A 2D grid class (yet another 2d grid class ...)
 * 
 * The x axis points to the vehicle forward, and the y axis points to the vehicle leftwards. 
 * Cell ij=00 is the upper leftmost one (xmax, ymax). 
 * 
 */
class Grid2D
{
    protected:
        //sizes in x and y axis respectively
        double size_x_, size_y_; 

        //cell size in x and y axis respectively        
        double step_x_, step_y_; 

        //number of cells in x and y axis respectively
        unsigned int nx_, ny_; 
        
        //grid position of the upper left corner, (ij=00)
        double xmax_, ymax_; 
        
        //number of consecutive iterations that are taken into account to build the grid.
        unsigned int holding_iters_; 
        
        //The grid array. There is always two allcated grids, which interchange the role of current grid. 
        //In each grid, each cell holds the probability of being occupied. 
        Eigen::MatrixXd grid_[2]; 
        
        //index to the current grid in the above array of grids
        unsigned int current_;
        
        //index to the next grid in the above array of grids
        unsigned int next_;

        
    public:
        /** \brief Constructor with arguments
        * 
        * Constructor with arguments
        *   \param _sz_x size of x [m]
        *   \param _sz_y size of y [m]
        *   \param _dx size of the cell in x direction [m]
        *   \param _dy size of the cell in y direction [m]
        *   \param _xmax max x value (upper left corner) [m]
        *   \param _ymax max y value (upper left corner) [m]
        *   \param _hold number of consecutive iterations holding in the grid
        * 
        */
        Grid2D(const double & _sz_x, const double & _sz_y, const double & _dx, const double & _dy, const double & _xmax, const double & _ymax, const unsigned int & _hold);

        /** \brief Destructor
        * 
        * This destructor frees all necessary dynamic memory allocated within this class.
        */
        ~Grid2D();
        
        /** \brief returns grid resolution in m/cell
         * 
         * returns grid resolution in m/cell
         * 
         **/
        double getGridStep() const;

        /** \brief returns grid height in meters
         * 
         * returns grid height in meters
         * 
         **/
        double getSizeX() const;

        /** \brief returns grid width in meters
         * 
         * returns grid width in meters
         * 
         **/        
        double getSizeY() const;

        /** \brief returns num of vertical cells
         * 
         * returns num of vertical cells
         * 
         **/                
        unsigned int getNx() const;        
        
        /** \brief returns num of horizontal cells
         * 
         * returns num of horizontal cells
         * 
         **/                
        unsigned int getNy() const;
        
        /** \brief returns xmax_
         * 
         * returns xmax_
         * 
         **/
        double getXmax() const;

        /** \brief returns ymax_
         * 
         * returns ymax_
         * 
         **/
        double getYmax() const;

        /** \brief Get grid indexes given a xy position
         * 
         * Get ij grid indexes given a xy position
         * 
         **/
        bool xy2ij(const double & _x, const double & _y, unsigned int & _i, unsigned int & _j) const;
        
        /** \brief Get xy position given ij grid indexes
         * 
         * Get xy position given ij grid indexes
         * Returns true if xy within grid limits, otherwise returns false 
         * 
         **/
        void ij2xy(const unsigned int & _i, const unsigned int & _j, double & _x, double & _y) const;

        /** \brief Odometry update
         * 
         * Updates all the grid cells given a robot motion, expressed as a 2D twist
         *      \param _vx linear forward velocity 
         *      \param _vy linear lateral veolcity 
         *      \param _w rotational velocity around z axis (vertical axis)
         *      \param _dt integration time
         * 
         **/
        void odometryUpdate(const double & _vx, const double & _vy, const double & _w, const double & _dt);
        
        /** \brief Lidar update
         * 
         * Updates all the grid cells given a new lidar scan
         *      \param _scan_points xy coordinates of the scanned ranges, with respect to the vehicle (grid) frame. Each column is a coordinate pair xy.
         * 
         **/
        void lidarUpdate(const Eigen::MatrixXd & _scan_points);

        /** \brief Radar update
         * 
         * Updates all the grid cells given a new radar set of clusters
         *      \param _radar_clusters xy coordinates of the detected radar clusters, with respect to the vehicle (grid) frame
         * 
         **/
        void radarUpdate(const Eigen::MatrixXd & _radar_clusters);
        
        /** \brief Returns grid in ROS frienfly fromat
         * 
         * Returns grid in ROS frienfly fromat
         * 
         **/
        void getGrid(std::vector<int8_t> & _data) const; 
        
        /** \brief Display in ascii format
         * 
         * Display in ascii format
         * 
         **/
        void display() const;
        
        /** \brief Print grid parameters 
         * 
         * Print grid parameters 
         * 
         **/
        void print() const;

        /** \brief a debug function
         * 
         * a debug function
         * 
         **/
        void debugFill(); 
        
    protected:
        /** \brief Update index over the grid array
         * 
         * Update index over the grid array
         * 
         **/
        void updateGridArrayIndexes();
        
        
};
#endif



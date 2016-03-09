#ifndef LINE_FINDER_HOUGH_H_
#define LINE_FINDER_HOUGH_H_

//laserscanutils
#include "line_finder.h"

namespace laserscanutils
{
/** \brief set of tunning parameters for the Hough transform line detection
* 
* set of tunning parameters for the Hough transform line detection
* 
*/
struct LineFinderHoughParams
{
    double range_max_; //maximum allowed range for lines
    double range_step_; //range step in the voting grid
    double theta_step_; //theta step in the voting grid
    unsigned int min_supports_; //Min supports at the hough grid to consider a cell as a line
    void print() const; //just a print method
};

    
/** \brief Class implementing methods to extract striaght segments from a scan
* 
* Class implementing methods to extract striaght segments from a scan
* 
*/
class LineFinderHough : public LineFinder
{
    protected:
        
        //Tunning params for Hough approach
        LineFinderHoughParams hough_params_; 
        
        //Hough grid in theta-range space. Each cell holds a list of the point coordinates supporting the cell
        std::vector<std::vector<std::list<Eigen::Vector3s > > > hough_grid_; 
        std::list<Eigen::SparseMatrix<unsigned int> > hough_grid_x_; //hough grid is a list of grids. Each one corresponds to the votes of a single point
        
        //dereived Hough params
        unsigned int hough_grid_rows;
        unsigned int hough_grid_rows_half;
        unsigned int hough_grid_cols;
        unsigned int hough_grid_cols_half;
    
    public: 
        /** \brief Constructor
         * 
         * Constructor
         * 
         **/
        LineFinderHough(); 
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        ~LineFinderHough(); 
                
        /** \brief Set Hough tunning params
         * 
         * Set Hough tunning params. Resizes Hough grid accordingly.
         * 
         **/
        void setHoughParams(const LineFinderHoughParams & _params);
                
        /** \brief Find lines using Hough transform. Result as a list of Line's
        *
        * Find lines from a set of points.
        * Returns Lines as a std::list<laserscanutils::StraightSegment>
        * 
        * \Requires: 
        *    \param _points: 3xN matrix, set of points. Each column is a 2D point in homogeneous (x,y,1). Ordering is not required.
        * 
        * \Provides: 
        *    \param _line_list set of lines extracted from _points
        *    \return Number of lines extracted.
        * 
        *
        */
        unsigned int findLines( const Eigen::MatrixXs & _points, 
                                std::list<laserscanutils::LineSegment> & _line_list);
        
        /** \brief Print things
         * 
         * Print things about this class
         * 
         **/
        void print() const;
};
}//namespace
#endif
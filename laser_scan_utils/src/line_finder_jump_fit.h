#ifndef LINE_FINDER_JUMP_FIT_H_
#define LINE_FINDER_JUMP_FIT_H_

//laserscanutils
#include "line_finder.h"

namespace laserscanutils
{
/** \brief Set of tunning parameters for the Jump&Fit approach
* 
* Set of tunning parameters for line extraction using the Jump&Fit approach
* 
*/
struct LineFinderJumpFitParams 
{
    //members
    ScalarT jump_dist_ut_; //Upper threshold in consecutive ranges to consider a jump
    ScalarT jump_angle_ut_; //Upper threshold in angle of two consecutive ranges to consider a jump
    ScalarT window_length_; // length (m) of the window of points to fit lines in the first pass
    unsigned int min_window_points_; // minimum number of points to fit lines in the first pass
    ScalarT k_sigmas_ut_;//Uppet threshold of how many std_dev are tolerated to count that a point is supporting a line
    unsigned int concatenate_ii_ut_;//Upper threshold for ray index difference between consecutive lines to consider concatenation
    ScalarT concatenate_angle_ut_; //Upper threshold for angle between consecutive lines to consider concatenation
    void print() const; //just a print method
};

    
/** \brief Class implementing methods to extract straight segments from a scan, following a "jump & fit approach"
* 
* Class implementing methods to extract straight segments from a scan, following a "jump & fit approach"
* 
*/
class LineFinderJumpFit : public LineFinder
{
    protected:
        //Tunning params for Jump & Fit approach
        LineFinderJumpFitParams jump_fit_params_;
        
    public: 
        /** \brief Constructor
         * 
         * Constructor
         * 
         **/
        LineFinderJumpFit(); 
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        ~LineFinderJumpFit(); 
                
        /** \brief Set Jump and Fit tunning params
         * 
         * Set Jump and Fit tunning params
         * 
         **/
        void setJumpFitParams(const LineFinderJumpFitParams & _params);
                
        /** \brief Find lines using Jump & Fit. Result as a list of Line's
        *
        * Find lines from a set of ordered points.
        * Returns Lines as a std::list<laserscanutils::StraightSegment>
        * 
        * \Requires: 
        *    \param _points: 3xN matrix, set of points. Each column is a 2D point in homogeneous (x,y,1). 
        *                    Ordering is required.
        *                    Points should belong to a single scan segment
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
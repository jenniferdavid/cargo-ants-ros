
#ifndef LINE_DETECTOR_H_
#define LINE_DETECTOR_H_

//laserscanutils
#include "laser_scan_utils.h"
#include "entities.h"
#include "scan_basics.h"

namespace laserscanutils
{
    /** \brief Set of tunning parameters
     * 
     * Set of tunning parameters for line extraction
     * 
     */
    struct ExtractLineParams //TODO rename to ExtractLinesParams
    {
        //members
        ScalarT jump_dist_ut_; //Upper threshold in consecutive ranges to consider a jump
        ScalarT jump_angle_ut_; //Upper threshold in angle of two consecutive ranges to consider a jump
        ScalarT window_length_; // length (m) of the window of points to fit lines in the first pass
        unsigned int min_window_points_; // minimum number of points to fit lines in the first pass
        ScalarT k_sigmas_ut_;//Uppet threshold of how many std_dev are tolerated to count that a point is supporting a line
        unsigned int concatenate_ii_ut_;//Upper threshold for ray index difference between consecutive lines to consider concatenation
        ScalarT concatenate_angle_ut_; //Upper threshold for angle between consecutive lines to consider concatenation

        //just a print method
        void print() const;
    };
    
    /** \brief set of tunning parameters for the Hough transform line detection
     * 
     * set of tunning parameters for the Hough transform line detection
     * 
     **/
    struct ExtractLinesHoughParams
    {
        double range_max_; //maximum allowed range for lines
        double range_step_; //range step in the voting grid
        double theta_step_; //theta step in the voting grid
        unsigned int min_supports_; //Min supports at the hough grid to consider a cell as a line
    };
    
    /** \brief Find the best fittig line given a set of points
     * 
     * Find the best fittig line given a set of points
     * 
     * \Requires:
     * \param _points: 3xN matrix, set of points. Each column is a 2D point in homogeneous (x,y,1). Ordering is not required.
     * 
     * \Provides:
     * \param _line: a laserscanutils::Line object of the best fitting line in the Least Squares sense
     * 
     **/
    void fitLine(const Eigen::MatrixXs & _points, Line & _line);
        
    /** \brief Extract lines from a given scan. Result as a list of Line
     *
     * Extract lines from a given scan.
     * Returns Lines as a list of laserscanutils::Line
     * \param _params are the laser scan parameters
     * 
     * \param _ranges raw ranges data
     * \param _line_list output lines extracted from the scan
     * \return Number of lines extracted.
     *
     */
    unsigned int extractLines(const laserscanutils::ScanParams & _params, 
                              const ExtractLineParams & _alg_params, 
                              const std::vector<float> & _ranges, 
                              std::list<laserscanutils::Line> & _line_list);

    /** \brief Extract lines using Hough transform. Result as a list of Line's
     *
     * Extract lines from a set of scans.
     * Returns Lines as a std::list<laserscanutils::Line>
     * 
     * \Requires: 
     *    \param _points: 3xN matrix, set of points. Each column is a 2D point in homogeneous (x,y,1). Ordering is not required.
     *    \param _alg_params Hough transform parameters
     * 
     * \Provides: 
     *    \param _line_list set of lines extracted from _points
     *    \return Number of lines extracted.
     * 
     *
     */
    unsigned int extractLinesHough( const Eigen::MatrixXd & _points,
                                    const ExtractLinesHoughParams & _alg_params, 
                                    std::list<laserscanutils::Line> & _line_list);

    
    /** \brief Returns the angle between two lines
     *
     * Returns the angle between two lines
     *
     */
    ScalarT angleBetweenLines(const laserscanutils::Line& line1, const laserscanutils::Line& line2);

    /** \brief Returns the window points
     *
	 * Returns the number of points in window depending on the ranges
	 *
	 */
    unsigned int windowPoints(const laserscanutils::ScanParams & _params,
    						  const ExtractLineParams & _alg_params,
    						  const std::vector<float> & _ranges,
    						  unsigned int& ii);
}
#endif

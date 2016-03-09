#ifndef LINE_FINDER_H_
#define LINE_FINDER_H_

//laserscanutils
#include "laser_scan_utils.h"
#include "line_segment.h"

//std
#include <list>

namespace laserscanutils
{
/** \brief Base Class for methods to extract striaght segments from a scan
* 
* Base Class for methods to extract striaght segments from a scan
* 
*/
class LineFinder
{
    protected:
            
    public: 
        /** \brief Constructor
         * 
         * Constructor
         * 
         **/
        LineFinder(); 
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        ~LineFinder(); 
        
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
        void fitLine(const Eigen::MatrixXs & _points, LineSegment & _line) const;    
        
        /** \brief Find the best fittig line given a set of points, applying RANSAC
        * 
        * Find the best fittig line given a set of points, applying RANSAC
        * 
        * \Requires:
        * \param _points: 3xN matrix, set of points. Each column is a 2D point in homogeneous (x,y,1). Ordering is not required.
        * \param _error: maximum fitting error to accept a point in a line
        * 
        * \Provides:
        * \param _line: a laserscanutils::Line object of the best fitting line in the Least Squares sense
        * 
        * RANSAC is applied as outlier rejection method. 
        * 
        **/
        void fitLineRansac(const Eigen::MatrixXs & _points, LineSegment & _line, ScalarT _error = 0.1) const;    

        /** \brief Merges lines of a list
         * 
         * Updates the input list with the merged lines
         * Lines are merged if all below conditions fulfill:
         *      - abc line homogeneous params close 
         *      - Overlapping points
         */
        //unsigned int mergeLines(std::list<LineSegment> & _line_list);

        /** \brief Find lines. Pure virtual. To be implemented by each inherited class
        *
        * Find lines from a set of scans.
        * Returns Lines as a std::list<LineSegment>
        * 
        * \Requires: 
        *    \param _points: 3xN matrix, set of points. Each column is a 2D point in homogeneous (x,y,1). Ordering is not required.
        * 
        * \Provides: 
        *    \param _line_list set of lines extracted from _points
        *    \return Number of lines extracted.
        *
        */
        virtual unsigned int findLines( const Eigen::MatrixXs & _points, 
                                        std::list<laserscanutils::LineSegment> & _line_list) = 0;
        
        /** \brief Print things
         * 
         * Print things about this class
         * 
         **/
        virtual void print() const = 0;
};
}//namespace
#endif
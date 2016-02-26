#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

//#include <graphics.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

//laserscanutils
#include "entities.h"
#include "clustering.h"
#include "object_detector.h"



void readOneScanRanges(const char * _file_name, std::vector<float> & ranges_read)
{
    // "/home/vvaquero/iri-lab/matlab/laser_detection_mat/TestOneScanRanges.txt"
    std::ifstream inputFile(_file_name);   // input file stream with name

    ranges_read.clear();

    std::string lineTest;
    std::getline( inputFile, lineTest );

    // use a stringstream to separate the fields out of the line
    std::stringstream ss( lineTest);
    std::string field;
    while (getline( ss, field, ',' ))
    {
      // for each field convert it to a double
      std::stringstream fs( field );
      double f = 0.0;  // (default value is 0.0)
      fs >> f;

      ranges_read.push_back( f );
    }
    inputFile.close();


    // printing
//    std::cout << " Vector Ranges [ " ;
//    for (unsigned int k = 0; k < 10; k++)
//    {
//        std::cout << ranges_read[k] << ", ";
//    }
//    std::cout << " ] " << std::endl;
}


void initParameters(laserscanutils::LaserDetectionParams & _parameters)
{
    _parameters.jump_thr                = 0.9;
    _parameters.max_num_cluster_points  = 200;
    _parameters.min_num_cluster_points  = 5;
    _parameters.segment_window_size = 6;
    _parameters.theta_max_parallel = 0.1;
    _parameters.k_sigmas = 3;

    //_parameters.print();
}



int main()
{
/*    //TEST For graphical output
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(800,600);
    glutCreateWindow("Hello World");

    setup();
    glutDisplayFunc(display);
    glutMainLoop();
    */

    // *****  INITIALIZATION  ***** //

    laserscanutils::LaserDetectionParams alg_parameters;    // structure with the parameters to tune the algorithm
//    laserscanutils::LaserDetectionStats alg_stats;          // structure with the internal variables of the algorithm
    laserscanutils::LaserScanProcessed alg_stats;          // structure with the internal variables of the algorithm
    initParameters(alg_parameters);
    alg_parameters.print();


    // get laser Scan Parameters from real laser rosbag
    //    Laser 2D Scan Parameters :
    //       Angle min: -1.0472
    //       Angle max: 0.610865
    //       Angle step: 0.00436332
    //       Scan time: 6.95332e-310
    //       Range min: 0.1
    //       Range max: 200
    //       Range std dev: 6.91555e-310
    laserscanutils::ScanParams scan_params;
    scan_params.angle_min_ = -1.0472;
    scan_params.angle_max_ = 0.610865;
    scan_params.angle_step_ = 0.00436332;
    scan_params.scan_time_ = 0.01;//not relevant
    scan_params.range_min_ = 0.1;
    scan_params.range_max_ = 200;
    scan_params.range_std_dev_ = 0.01;
    scan_params.print();

    laserscanutils::ExtractCornerParams algExtractCornerParams;
    algExtractCornerParams.line_params_.jump_dist_ut_ = 500; 
    algExtractCornerParams.line_params_.jump_angle_ut_ = 3;
    algExtractCornerParams.line_params_.window_sz_ = 5;
    algExtractCornerParams.line_params_.k_sigmas_ut_ = 3;
    algExtractCornerParams.line_params_.concatenate_ii_ut_ = 5; 
    algExtractCornerParams.line_params_.concatenate_angle_ut_ = 0.1; 
    algExtractCornerParams.theta_min_ = 0.01;
    algExtractCornerParams.max_distance_ = 0.9;
    algExtractCornerParams.print(); 

    // *****  READING LASER POINTS FOR TEST  ***** //
    std::vector<std::vector<float> > values;                // temp for reading the file values
    std::ifstream fin("/home/vvaquero/iri-lab/matlab/laser_detection_mat/testOneScanMatrix.txt");   // input file stream with name

    for (std::string line; std::getline(fin, line); )
    {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream in(line);
        values.push_back(
            std::vector<float>(std::istream_iterator<float>(in),
                                std::istream_iterator<float>()));
    }

    unsigned int rows = values.size();
    unsigned int columns = 0;

    for (unsigned int k = 0; k < rows; k++)
    {
        if (values[k].size() > columns)
            columns = values[k].size();
    }

    std::cout << "Matrix Size = " << rows << "x" << columns  <<std::endl;
    Eigen::MatrixXs result(rows,columns);

    for (unsigned int i = 0; i < rows; i++)
        for (unsigned int j = 0; j < columns; j++)
        {
            result(i,j) = values[i][j];
        }

    alg_stats.scanTotalPoints_ = 380;
    alg_stats.filteredTotalPoints_ = columns;

    //std::cout << "Matrix = " << std::endl << result <<std::endl;


    //  |
    //  ->
    // Till here, everything is like a scan has been received and prefiltered



    // Finding clusters on the scan -> fulfills "stats.cluster_idxs", cleared at the begining.
    //laserscanutils::findClusters(alg_parameters, result,alg_stats);


    laserscanutils::extractClustersInScan(alg_parameters, result,alg_stats);

    // DEBUG: Printing Cluster Information.
    std::cout << "Clusters: " << std::endl <<
                 " TotalFound = " << alg_stats.number_of_clusters_ << std::endl <<
                 " Valid Ones = " << alg_stats.number_of_valid_clusters_ << std::endl <<
                 " Indexes = " ;
                    for (unsigned int j = 0; j < alg_stats.cluster_indxs_.size(); j++)
                    {
                        std::pair<int,int> tempSeg = alg_stats.cluster_indxs_[j];
                        std::cout << "(" << tempSeg.first << "," << tempSeg.second << ") | " ;
                    }
    std::cout << std::endl;



    std::list<laserscanutils::Line>::iterator list_it1;

    alg_stats.line_list_.clear();



    //for (auto pair_it = alg_stats.cluster_indxs_.begin(); pair_it != alg_stats.cluster_indxs_.end(); pair_it++)
    for (auto object_it = alg_stats.object_list_.begin(); object_it != alg_stats.object_list_.end(); object_it++)
    {
        // temporal object
        laserscanutils::Object temp_obj = *object_it;
        //std::pair<int,int> tempSeg = *pair_it;
        //std::list<laserscanutils::Line> line_list_temp;  // temporal //TODO: put on alg_stats structure.

        //std::cout << std::endl << "-------" << std::endl << " SEGMENT FROM : " << tempSeg.first << " to " << tempSeg.second     << std::endl;
        std::cout << std::endl << "-------" << std::endl << " ObjectCluster FROM : " << object_it->first_ << " to " << object_it->last_  << std::endl;

        Eigen::MatrixXs tempCluster = result.block(0,object_it->first_, 3, object_it->last_-object_it->first_+1);

        //std::cout << "tempCluster = " << std::endl << tempCluster << std::endl;


        // LINES
        laserscanutils::findLinesInCluster(scan_params,alg_parameters, tempCluster, object_it->first_ , object_it->line_list_);

        // concat line_list_temp with to the general line_list
        std::cout << " Temp Lines in cluster= " << object_it->line_list_.size() << std::endl;

        //alg_stats.object_list_.push_back(temp_obj);

        //alg_stats.line_list_.splice(alg_stats.line_list_.end(),object_it->line_list_,object_it->line_list_.begin(),object_it->line_list_.end());
        alg_stats.line_list_.insert(alg_stats.line_list_.end(),object_it->line_list_.begin(), object_it->line_list_.end());
        std::cout << " Final Lines in total = " << alg_stats.line_list_.size() << std::endl;


        // CORNERS
        std::cout << "Extracting the Corners of each Cluster/Object" << std::endl;
        laserscanutils::findCornersInClusterLines(scan_params, algExtractCornerParams, object_it->line_list_, object_it->corner_list_);

    }






    // DEBUG: Printing global lines information
    std::cout << std::endl << std::endl << "GLOBAL LINE LIST" << std::endl;
    for (list_it1 = alg_stats.line_list_.begin(); list_it1 != alg_stats.line_list_.end(); list_it1++)
    {
        std::cout << "  #." << std::distance(alg_stats.line_list_.begin(),list_it1) << std::endl;
        list_it1->print();
        std::cout << std::endl << std::endl;
    }
    // DEBUG: END Printing global lines information



    // DEBUG: Printing ALL objects information
    for (auto object_it1 = alg_stats.object_list_.begin(); object_it1 != alg_stats.object_list_.end(); object_it1++)
    {
        std::cout << "  #." << std::distance(alg_stats.object_list_.begin(),object_it1) << std::endl;
        object_it1->print();
        std::cout << std::endl << std::endl;
    }
    // DEBUG: END Printing ALL objects information


    unsigned int numberOfCorners;
    std::list<laserscanutils::Corner> corner_list;
    //numberOfCorners = laserscanutils::findCornersInClusterLines(scan_params,algExtractCornerParams,line_list,corner_list);





} // end main

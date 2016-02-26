
#ifndef ENTITIES_H_
#define ENTITIES_H_

//laserscanutils
#include "laser_scan_utils.h"

#include <iostream>
#include <list>

//TODO: .cpp with print() functions per each entity
namespace laserscanutils
{
    struct Line
    {
        Eigen::Vector3s vector_; //homogeneous parameterization of the line: (a,b,c)^T -> ax+by+c=0
        ScalarT error_; //sum of all distances from used points to line
        unsigned int first_; //index of the range vector of the first point used
        unsigned int last_; //index of the range vector of the last point used
        Eigen::Vector3s point_first_; //homogeneous coordinates of the starting 2D point
        Eigen::Vector3s point_last_; //homogeneous coordinates of the ending 2D point
        unsigned int np_; // number of points of the line
        double range_; //range component in polar coordinates
        double theta_; //theta component in polar coordinates
        //TODO: add an Eigen::Map to the supporting points ... but who ensures memory allocation of such points ???

        //just a print method
        void print() const;
        
        //returns length
        float length();
    };
    
    struct Corner 
    {
        Corner()
        {
            this->pt_ = {0,0,0};
            this->aperture_ = 0;
            this->orientation_ = 0;
            from_object_ = -1;

        }

        Eigen::Vector3s pt_; //homogeneous coordinates of the 2D point, [meters]
        ScalarT orientation_; //orientation angle from the line1 to x axis, in [-pi,pi]
        ScalarT aperture_; //angle aperture of the corner in [0,2pi]. Aperture angle defined passing thorugh the shadow area of the scan (not crossing rays)
        Line line_1_; // line of the corner
        Line line_2_; // line of the corner

        // test
        ScalarT from_object_;   // ID of the object that belongs to (-1 if not associated).
        
        //just a print method
        void print() const;        
    };


    struct Object
    {
        // Constructor
        Object()
        {
            this->num_points_ = 0;
            this->first_ = 0;
            this->last_ = 0;
            this->associated_ = false;
            this->structured_ = false;

            this->id_ = -1;

            this->ref_centroid_point_ = {0.0, 0.0, 0.0};
            this->ref_act_point_ = {0.0, 0.0, 0.0};
            this->ref_old_point_ = {0.0, 0.0, 0.0};

            this->min_x_ = 1000.0;
            this->min_y_ = 1000.0;
            this->max_x_ = -1000.0;
            this->max_y_ = -1000.0;
            this->obj_orientation_ = 0.0;
            this->object_center_ << 0.0, 0.0;
            this->size_x_ = 0.0;
            this->size_y_ = 0.0;


            this->ref_type_ = -1;
            this->ref_position_ = -1;       // TBR??? Position of the corner/line in the respective list...
            this->asso_to_prev_obj_in_pos_ = -1;       // TBR??? Position of the corner/line in the respective list...

        }

        bool associated_;
        bool structured_;
        unsigned int num_points_;   // number of points of the cluster
        unsigned int first_;        //index of the Homogeneous Matrix column for the first point used (GLOBAL)
        unsigned int last_;         //index of the Homogeneous Matrix column for the last point used (GLOBAL)
        std::list<laserscanutils::Line> line_list_;        // list of lines contained in the cluster (Refs Points are LOCAL)
        std::list<laserscanutils::Corner> corner_list_;    // list of corners contained in the cluster (Refs Points are LOCAL)

        ScalarT id_;        // Id of the object. -1 if unknown or MOBILE; other number if pre-associated.

        unsigned int asso_to_prev_obj_in_pos_;       // For tracking help, this object has been associated to the object in position
                                                // "asso_to_obj_in_pos_" from the previous scan. (-1 if not)

        // Description
        Eigen::Vector3s ref_centroid_point_;
        Eigen::Vector3s ref_act_point_;
        Eigen::Vector3s ref_old_point_;

        float min_x_;
        float min_y_;
        float max_x_;
        float max_y_;
        float obj_orientation_;
        float size_x_;
        float size_y_;
        Eigen::Vector2s object_center_;
        Eigen::MatrixXs polyline_points_;  //<float,2,5>

        int ref_type_;      // Type of the reference. 8 = corner;   4 = init_line ; 2 = end_line;   1 = centroid. -1 = unkonwn;
        int ref_position_;  // [TODO] The position of the referenced corner (or line) in its corresponding list;


        void print() const;
        float getArea();
        float getPerimeter();

    };
}
#endif

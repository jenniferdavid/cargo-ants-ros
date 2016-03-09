//std
#include "iostream"

//laserscanutils
#include "entities.h"

void laserscanutils::Line::print() const
{
    std::cout << "Line Parameters : " << std::endl 
//               << "   (a,b,c): " << vector_.transpose() << std::endl
//               << "   error: " << error_ << std::endl
//               << "   first,last: " << first_ << " , " << last_ << std::endl
//               << "   first point: " << point_first_.transpose() << std::endl
//               << "   last point: " << point_last_.transpose() << std::endl
              << "\tnumber of points: " << np_ << std::endl
              << "\trange: " << range_ << std::endl
              << "\ttheta: " << theta_ << std::endl;
}

float laserscanutils::Line::length()
{
    return(std::sqrt( std::pow( (this->point_last_(0)-this->point_first_(0)), 2 ) + std::pow( (this->point_last_(1)-this->point_first_(1)), 2)   )  );
}



void laserscanutils::Corner::print() const
{
    std::cout << "Corner Parameters : " << std::endl 
              << "   corner point: " << pt_.transpose() << std::endl
              << "   orientation: " << orientation_ << std::endl
              << "   aperture: " << aperture_ << std::endl;
//    std::cout << "Line 1: ";
//    line_1_.print();
//    std::cout << "Line 2: ";
//    line_2_.print();
}




void laserscanutils::Object::print() const
{
    std::cout <<
    std::endl     << "---------------------------------------------" << std::endl
                  << "---      Object Detected Information      ---" << std::endl
                  << "---------------------------------------------" << std::endl
                  << "      * Initial Global Point : \t"        << this->first_       << std::endl
                  << "      * Final Global Point : \t"          << this->last_        << std::endl
                  << "      * Number of Points: \t"             << this->num_points_  << std::endl
                  << "      * Associated?: \t\t"                << this->associated_<< std::endl
                  << "      * Structured?: \t\t"                << this->structured_ << std::endl
                  << "      * Bounding Box Center : \t"         << this->object_center_.transpose()       << std::endl
                  << "      * Bounding Box Size : \t\t"           << this->size_x_ << " x " << this->size_y_ << std::endl
                  << "      * Bounding Box Orientation :"     << this->obj_orientation_ << std::endl
                  << "      * Polyline: "                       << std::endl << "\t\t" << this->polyline_points_ << std::endl
                  << "      * Lines: " << std::endl;

            if (this->line_list_.size() >= 1)
            {
                    for (auto it1 = this->line_list_.begin(); it1 != this->line_list_.end(); it1++)
                    {
                        std::cout << "  - Line #." << std::distance(this->line_list_.begin(),it1) << std::endl;
                        it1->print();
                        std::cout << std::endl;
                    }
            }
            else
                std::cout << "      --> No Lines in this Object. " << std::endl;

            std::cout << "      * Corners: " << std::endl;
            if (this->corner_list_.size() >= 1)
            {
                std::cout << "object Corner line SIZE: " << this->corner_list_.size() << std::endl;
                    for (auto it1 = this->corner_list_.begin(); it1 != this->corner_list_.end(); it1++)
                    {
                        std::cout << "  - Corner #." << std::distance(this->corner_list_.begin(),it1) << std::endl;
                        it1->print();
                        std::cout << std::endl;
                    }
            }
            else
                std::cout << "      --> No corners in this Object. " << std::endl;


}

float laserscanutils::Object::getArea()
{
    return this->size_x_*this->size_y_;
}

float laserscanutils::Object::getPerimeter()
{
    return ( (this->size_x_*2) + (this->size_y_*2) );
}

/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */


#ifndef SUNFLOWER_BUBBLE_HPP
#define SUNFLOWER_BUBBLE_HPP


#include <sfl/api/Scan.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {


  class Bubble
  {
  public:
    typedef enum { 
      NOOVERLAP = 0, 
      FULLOVERLAP, 
      PARTIALOVERLAP, 
      SAMECENTER
    } overlap_t;
    
    static const double DEFAULTALPHAINT;// = 0.1;
    static const double DEFAULTALPHAEXT;// = 0.5;
    
    
    static bool CheckOverlap(const Bubble & bubble1,
			     const Bubble & bubble2,
			     double min_normal);
    static bool InformativeCheckOverlap(const Bubble & bubble1,
					const Bubble & bubble2,
					double min_normal,
					double & dx,
					double & dy,
					double & distance,
					double & radial,
					double & normal,
					int & overlap_type);
    static double Distance(const Bubble & bubble1,
			   const Bubble & bubble2);

    void Configure(double cutoff_distance,
		   std::pair<double, double> position);
    void CopyConstruct(const Bubble & original);
    
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value). */
    void
    UpdateExternalParameters(boost::shared_ptr<const Scan> scan,
			     double ignore_radius);
    
    void ResetInternalParameters();
    void UpdateInternalParameters();
    void ApplyForces();
    double IgnoreDistance() const { return _ignore_distance; }
    void SetMinIgnoreDistance(double d);
    void SetIgnoreDistance(double d);
    double X() const { return _position.first; }
    double Y() const { return _position.second; }
    double Radius() const { return _radius; }
    Bubble * Next() const { return _next; }
  

  private:
    friend class BubbleFactory;
    friend class BubbleList;
    friend class BubbleBand;
    friend class ReplanHandler;


    Bubble * _previous;
    Bubble * _next;


    std::pair<double, double> _position;
    std::pair<double, double> _closest;
    double _radius, _radius2;
    double _dprevious, _dnext, _path_length;
    double _cutoff_distance;
    double _ignore_distance, _ignore_distance2;
    /** \todo unused? */
    double _min_ignore_distance;
    /** \todo use a 2d-vector class */
    std::pair<double, double> _fext, _fint;
    double _alpha_int, _alpha_ext;


    void Construct();

    static overlap_t InformativeOverlap(const Bubble & bubble1,
					const Bubble & bubble2,
					double & dx,
					double & dy,
					double & distance,
					double & radial,
					double & normal);
  };

}

#endif // SUNFLOWER_BUBBLE_HPP

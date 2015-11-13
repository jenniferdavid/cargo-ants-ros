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


#ifndef SUNFLOWER_SCAN_HPP
#define SUNFLOWER_SCAN_HPP


#include <sfl/api/Timestamp.hpp>
#include <sfl/util/Frame.hpp>
#include <vector>


namespace sfl {

  struct scan_data {
    /** ray angles [rad] */      
    double phi;
    /** measured distances [m] */      
    double rho;
    /** x-coordinates [m] in the robot frame */      
    double locx;
    /** y-coordinates [m] in the robot frame */      
    double locy;
    /** x-coordinates [m] in the global frame */      
    double globx;
    /** y-coordinates [m] in the global frame */      
    double globy;
    /** true if the point is an actual point in the environment */
    bool in_range;
  };
  
  /**
     Encapsulates the whole dataset of a scan, useful for algorithms
     that are formulated for using arrays of data. Contains timestamp
     information in the form of an upper and a lower bound, useful for
     determining if data is still fresh, and in multisensor data
     fusion.
  */
  class Scan
  {
  public:
    /** \note that Scan allows everyone to access its fields */
    typedef std::vector<scan_data> array_t;
    
    /**
       Constructor for empty (zeroed arrays) data sets.
    */
    Scan(/** number of data points */
	 size_t nscans,
	 /** lower bound on the acquisition timestamp */
	 const Timestamp & tlower,
	 /** upper bound on the acquisition timestamp */
	 const Timestamp & tupper,
	 const Frame & scanner_pose);
    
    Scan(const Scan & original);
    
    /** lower bound of the estimated acquisition time */
    Timestamp tlower;
    
    /** upper bound of the estimated acquisition time */
    Timestamp tupper;
    
    /** estimated scanner position at "most probable" acquisition time */
    Frame scanner_pose;
    
    /** array of scan data */
    array_t data;
  };
  
}

#endif // SUNFLOWER_SCAN_HPP

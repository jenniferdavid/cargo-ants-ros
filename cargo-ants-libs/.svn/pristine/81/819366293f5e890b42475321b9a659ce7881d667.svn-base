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


#ifndef SUNFLOWER_MULTISCANNER_HPP
#define SUNFLOWER_MULTISCANNER_HPP


#include <boost/shared_ptr.hpp>
#include <vector>


namespace sfl {
  
  class LocalizationInterface;
  class Scanner;
  class Scan;

  /**
     Manages several scanners that can be mounted on the robot,
     provides a little unifying functionality.

     To use Multiscanner, first create instances for all the scanners
     on your robot. Then add them to a Multiscanner instance (in the
     order with which you'd like the data to appear in collected
     scans).
  */
  class Multiscanner
  {
  public:
    typedef std::vector<boost::shared_ptr<Scan> > raw_scan_collection_t;
    
    explicit Multiscanner(boost::shared_ptr<LocalizationInterface> localization);
    
    
    /** Appends a Scanner instance to the list of registered devices. */
    void Add(boost::shared_ptr<Scanner> scanner);

    /** \return The number of registered Scanners. */
    size_t Nscanners() const;

    /** \return A pointer to a registered Scanner, or 0 if no such index. */
    boost::shared_ptr<Scanner> GetScanner(size_t i) const;

    /**
       Concatenates data from all Scanners into a single Scan
       object. Only successfully retrieved data is added to the
       returned Scan instance, so Scan::Nscans() might return a number
       smaller than what you would expect from the sum of
       Scanner::Nscans().
       
       Timestamps of the returned Scan correspond to the minimum and
       maximum Timestamp of all registered Scanners. Even if a Scanner
       provides no data (e.g. all values are out of range), its
       Timestamp is still taken into account.  The robot pose will be
       taken from sfl::LocalizationInterface or sfl::Odometry (cannot
       remember right now which one), not from the information in the
       scanners.
       
       \note For polar coordinates, the robot origin is used. The
       ordering of the data is inherited from the order of calls to
       Multiscanner::Add(). Thus it is not guaranteed that the angles
       are monotonically increasing! Also, scans are collected via
       individual calls to Scanner accessors, it is thus possible that
       the result mixes data from more than one acquisition cycle.
       
       \note Scan::robot_pose and Scan::scanner_pose are both set to
       the robot pose. Use CollectRawScans() if you need to have more
       sensible value the scanner pose.
    */
    boost::shared_ptr<Scan> CollectScans() const;
    
    /**
       Similar to CollectScans(), but it contains all scan data (also
       OUT_OF_RANGE readings) and keeps the (phi, rho) values relative
       to each sensor's frame. This is more appropriate e.g. for
       "sweeped map updates".
    */
    boost::shared_ptr<raw_scan_collection_t> CollectRawScans() const;
    
    /** Calls Scanner::Update() on all registered instances and
	returns true if all of these calls succeeded. Does NOT take
	the shortcut of forfeiting updates after a failure. */
    bool UpdateAll(std::ostream * erros = 0);
    
  protected:
    typedef std::vector<boost::shared_ptr<Scanner> > vector_t;
    
    boost::shared_ptr<LocalizationInterface> m_localization;
    size_t m_total_nscans;
    vector_t m_scanner;
  };
  
}

#endif // SUNFLOWER_MULTISCANNER_HPP

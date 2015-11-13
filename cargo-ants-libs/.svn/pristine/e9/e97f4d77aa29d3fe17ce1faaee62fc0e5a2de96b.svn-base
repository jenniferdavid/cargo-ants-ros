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


#ifndef SUNFLOWER_SCANNER_HPP
#define SUNFLOWER_SCANNER_HPP


#include <sfl/api/LocalizationInterface.hpp>
#include <sfl/api/LidarChannel.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>


namespace sfl {
  
  
  class Scan;
  struct scan_data;
  class Timestamp;
  class Frame;
  
  
  /**
     Encapsulates a distance scanner such as the SICK-LMS. This class
     is generic in the sense that you have to provide it with a number
     of parameters to make it work correctly with the actual
     scanner(s) mounted on your robot:

     <ul>

       <li> Where the sensor is mounted on the robot. Scanner works in
            two coordinate frames: Polar coordinates (distance and ray
            angle) are represented in the sensor frame; Cartesian
            points are converted to the robot frame (using the
            provided <code>mount</code> information). </li>

       <li> Number of readings per scan (181 or 361 for SICK-LMS,
            depending on its configuration) </li>

       <li> Offset and sweep angles. This effectively defines the
            sensor frame: The first ray is considered to lie on
            <code>phi0</code> in the sensor frame, and the scanner
            sweeps a sector of <code>phirange</code> radians in the
            usual geometric sense (counterclockwise). If your sensor
            sweeps clockwise, just specify a negative
            <code>phirange</code>. </li>

       <li> Maximum valid distance (readings above this value are
            handled as <code>OUT_OF_RANGE</code>). </li>

     </ul>
     
     For example, the mount points, start and sweep angles on Robox are:
     
     <ul>
       <li> Front: <ul>
         <li> <code>mount</code>: (0.093, 0, 0) </li>
	 <li> <code>phi0</code>: - <code>M_PI</code> / 2 </li>
	 <li> <code>phirange</code>: <code>M_PI</code> </li> </ul> </li>
       <li> Rear: <ul>
         <li> <code>mount</code>: ( - 0.093, 0, <code>M_PI</code>) </li>
	 <li> <code>phi0</code>: - <code>M_PI</code> / 2 </li>
	 <li> <code>phirange</code>: <code>M_PI</code> </li> </ul> </li>
     </ul>
  */
  class Scanner
  {
  private:
    Scanner(const Scanner &);
    
  public:
    /**
       Return type of several accessors.
       
       Instead of using explicit <code>status_t</code> return values
       from the access methods, an exception mechanism might do the
       job "more elegantly". But <code>OUT_OF_RANGE</code> readings
       are actually quite common, not really an exception. It seemed
       inappropriate to write a <code>try ... catch</code> around all
       places where you access Scanner data.
       
       In most cases, you'll access scan data through a Scan instance
       anyways, which provides STL <code>vector</code> based semantics
       and is much more practical for handling a whole scan as a
       single object.
    */
    typedef enum {
      /** Successful call. */
      SUCCESS,
      /** You provided an invalid index. */
      INDEX_ERROR,
      /** The value is out of range (Scanner::Rhomax()). */
      OUT_OF_RANGE
    }
    status_t;
    
    /**
       The <code>name</code> parameter is useful for distinguishing
       between multiple scanners mounted on your robot. The meaning of
       the other parameters is explained in more detail in the general
       section (Scanner class documentation, scroll your browser
       upwards).
    */
    Scanner(boost::shared_ptr<LocalizationInterface> localization,
	    /** proxy object used to retrieve actual data */
	    boost::shared_ptr<LidarChannel> channel,
	    /** sensor origin wrt robot frame, copied over */
	    const Frame & mount,
	    /** number of scans per measurement */
	    size_t nscans,
	    /** maximum range [m] */
	    double rhomax,
	    /** angle of first ray wrt sensor frame [rad] */
	    double phi0,
	    /** angular range swept by measurement [rad] */
	    double phirange);
    
    /**
       \return copy of the most recently acquired scan
       
       \note The returned Scan object uses (phi, rho) relative to the
       sensor origin and can still contain readings that are out of
       range. Some consumers of Scan objects expect (phi, rho)
       relative to the robot origin and / or do not handle
       out-of-range readings very well. Consider using
       Multiscanner::CollectScans() in such cases.
    */
    boost::shared_ptr<Scan> GetScanCopy() const;
    
    /**
       Refresh the scan data.
       
       This method calls LidarChannel::GetData(). If something goes
       wrong, then the last valid data is returned by
       accessors. You can check the validity of the last Update() by
       checking AcquisitionOk(), which returns true if the previous
       attempt was successful. However, it is better to rely on Scan
       timestamps to check for valid data, because it is conceivable
       (depending on your application) that the acquisition status
       change between your call to AcquisitionOk() and an accessor.
       
       \return 0 on success.
    */
    int Update();
    
    /**
       Get a data point
       \return SUCCESS, OUT_OF_RANGE, or INDEX_ERROR
    */
    status_t GetData(/** index: [0, Scanner::Nscans() - 1] */
		     size_t index, scan_data & data) const;
    
    /**
       Get the measured distance [m] of a ray in sensor frame.
       \return SUCCESS, OUT_OF_RANGE, or INDEX_ERROR
    */
    status_t Rho(/** index: [0, Scanner::Nscans() - 1] */
		 size_t index,
		 /** (return) angle [phi] */
		 double & rho) const;
    
    /**
       Get the angle [rad] of a ray in sensor frame.
       \return SUCCESS or INDEX_ERROR
    */
    status_t Phi(/** index: [0, Scanner::Nscans() - 1] */
		 size_t index,
		 /** (return) angle [phi] */
		 double & phi) const;
    
    /**
       Get the cosine of a ray angle in sensor frame.
       \return SUCCESS or INDEX_ERROR
    */
    status_t CosPhi(/** index: [0, Scanner::Nscans() - 1] */
		    size_t index,
		    /** (return) angle [phi] */
		    double & cosphi) const;
    
    /**
       Get the sine of a ray angle in sensor frame.
       \return SUCCESS or INDEX_ERROR
    */
    status_t SinPhi(/** index: [0, Scanner::Nscans() - 1] */
		    size_t index,
		    /** (return) angle [phi] */
		    double & sinphi) const;
    
    /** \return upper timestamp of the last successfully acquired scan */
    const Timestamp & Tupper() const;
    
    /** \return lower timestamp of the last successfully acquired scan */
    const Timestamp & Tlower() const;
    
    bool AcquisitionOk() const;
    
    const boost::shared_ptr<const Frame> mount;
    const size_t nscans;
    const double rhomax;
    const double phi0;
    const double phirange;
    const double dphi;
    
  protected:
    typedef std::vector<double> vector_t;
    
    boost::shared_ptr<LocalizationInterface> m_localization;
    boost::shared_ptr<LidarChannel> m_channel;
    std::vector<boost::shared_ptr<Scan> > m_buffer;
    
    /** \todo The dirty/clean scheme makes little sense anymore... */
    boost::shared_ptr<Scan> m_dirty, m_clean; // would need to be mutexed for multithreading
    bool m_acquisition_ok;		      // would need to be mutexed for multithreading
    vector_t m_cosphi;
    vector_t m_sinphi;
  };
  
}

#endif // SUNFLOWER_SCANNER_HPP

/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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

#include "wrap_carmen.hpp"
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Scan.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>


#ifdef HAVE_CARMEN
# error Include the actual Carmen headers for Elrob here.
#else
# warning Using copy-pasted declarations.

extern "C" {
  
  //////////////////////////////////////////////////
  // copy-pasted from mapping/3dmapping/src/global/point.h
  // in revision 1734 of https://lsa1pc12.ethz.ch/svn/elrob/trunk
  
  typedef struct elrob_point_6d_t {
    double x;         /**< x cooordinate **/
    double y;         /**< y cooordinate **/
    double z;         /**< z cooordinate **/
    double phi;       /**< roll **/
    double theta;     /**< pitch **/
    double psi;       /**< yaw **/
  } elrob_point_6d_t;
  
  //////////////////////////////////////////////////
  // copy-pasted from mapping/3dmapping/src/interfaces/smart_messages.h
  // in revision 1734 of https://lsa1pc12.ethz.ch/svn/elrob/trunk
  
  typedef struct {
    elrob_point_6d_t pose;
    double velocity;
    double steering_angle;
    double timestamp;
    char* host;
  } elrob_smart_pos_message;
  
  typedef struct {
    double velocity;
    double steeringangle;
    double timestamp;
    char* host;
  } elrob_smart_steering_message;
  
  //////////////////////////////////////////////////
  // copy-pasted from carmen-elrob/src/laser/laser_messages.h
  // in revision 1734 of https://lsa1pc12.ethz.ch/svn/elrob/trunk
  
  typedef enum {
    SICK_LMS                  = 0, 
    SICK_PLS                  = 1, 
    HOKUYO_URG                = 2, 
    SIMULATED_LASER           = 3, 
    SICK_S300                 = 4, 
    UMKNOWN_PROXIMITY_SENSOR  = 99
  } carmen_laser_laser_type_t;
  
  typedef enum {
    REMISSION_NONE       = 0, 
    REMISSION_DIRECT     = 1, 
    REMISSION_NORMALIZED = 2
  } carmen_laser_remission_type_t;
  
  typedef struct {
    carmen_laser_laser_type_t  laser_type;
    double start_angle;
    double fov;
    double angular_resolution;
    double maximum_range;
    double accuracy;
    carmen_laser_remission_type_t remission_mode;
  } carmen_laser_laser_config_t;
  
  typedef struct {
    int id;
    carmen_laser_laser_config_t config;
    int num_readings;
    float *range;
    int num_remissions;
    float *remission;
    double timestamp;
    char *host;
  } carmen_laser_laser_message;
  
}

#endif // HAVE_CARMEN


using namespace sfl;
using namespace boost;
using namespace std;


namespace wrap_carmen {
  
#ifdef HAVE_CARMEN
# error Implement the actual Carmen communication here.
#else  
# warning Using dummy implementations of Carmen communication.
  
  
  bool init_receive_steering(ostream * err)
  {
    // The real implementation has to set up the connection and
    // probably return a handle of some kind. In case of an error,
    // print a message on the optional err stream and return false,
    // like this:
    static const bool some_error_happened(false);
    if(some_error_happened){
      if(err)
	*err << "ERROR in init_receive_steering(): blah blah blah\n";
      return false;
    }
    return true;
  }
  
  
  bool receive_steering(double & velocity,
			double & steeringangle,
			ostream * err)
  {
    // In the real implementation, you'll get the actual values from
    // Carmen communication. Here we just fill with fixed values that
    // make the car go round in circles. Also, Nepumuk doesn't care
    // about the timestamp or the host name, so we just ignore it.
    elrob_smart_steering_message msg;
    msg.velocity = 0.5;
    msg.steeringangle = 0.17;
    // msg.timestamp = 42;
    // msg.host = "dummy.wrap_carmen.nepumuk";
    
    velocity = msg.velocity;
    steeringangle = msg.steeringangle;
    return true;
  }
  
  
  bool cleanup_receive_steering(ostream * err)
  {
    // Here you'll have to disconnect and clean up.
    return true;
  }
  
  
  bool init_send_pos(ostream * err)
  {    
    return true;
  }
  
  
  bool send_pos(double x, double y, double theta,
		double velocity, double steering,
		ostream * err)
  {
    elrob_smart_pos_message msg;
    msg.pose.x = x;
    msg.pose.y = y;
    msg.pose.z = 0;
    msg.pose.phi = 0;		/**< roll **/
    msg.pose.theta = 0;		/**< pitch **/
    msg.pose.psi = theta;	/**< yaw **/
    msg.velocity = velocity;
    msg.steering_angle = steering;
    msg.timestamp = 42;		// this should probably change each tick
    msg.host = "dummy.wrap_carmen.nepumuk";
    
    // here you should send it...
    
    return true;
  }
  
  
  bool cleanup_send_pos(ostream * err)
  {
    return true;
  }
  
  
  bool init_send_laser(ostream * err)
  {
    return true;
  }
  
  
  bool send_laser(const Scanner & scanner,
		  ostream * err)
  {
    shared_ptr<Scan> scan(scanner.GetScanCopy());
    carmen_laser_laser_message msg;
    msg.id =                        scanner.hal_channel;
    msg.config.laser_type =         SICK_LMS;
    msg.config.start_angle =        scanner.phi0;
    msg.config.fov =                scanner.phirange;
    msg.config.angular_resolution = scanner.dphi;
    msg.config.maximum_range =      scanner.rhomax;
    msg.config.accuracy =           0.01; // well... hardcoded stuff...
    msg.config.remission_mode =     REMISSION_NONE;
    msg.num_readings =              scan->data.size();
    for(size_t ii(0); ii < scan->data.size(); ++ii)
      msg.range[ii] =               scan->data[ii].rho;
    msg.num_remissions =            0;
    // msg.remission[ii] = 0;
    msg.timestamp = 42;	      // use scan.tlower and/or scan.tupper...
    msg.host = "dummy.wrap_carmen.nepumuk";
    
    // here you should send it...
    
    return true;
  }
  
  
  bool cleanup_send_laser(ostream * err)
  {
    return true;
  }

#endif // HAVE_CARMEN
  
}

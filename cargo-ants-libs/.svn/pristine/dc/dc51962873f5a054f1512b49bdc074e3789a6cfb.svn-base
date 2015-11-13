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

#ifndef NPM_WRAP_CARMEN_HPP
#define NPM_WRAP_CARMEN_HPP


#include <iosfwd>


namespace sfl {
  class Scanner;
}


namespace wrap_carmen {
  
  //////////////////////////////////////////////////
  // You probably need to add some parameters in order to hook these
  // functions into Carmen, but the easiest generic interface that
  // seems possible.
  
  bool init_receive_steering(std::ostream * err);
  
  bool receive_steering(double & velocity,
			double & steeringangle,
			std::ostream * err);
  
  bool cleanup_receive_steering(std::ostream * err);
  
  bool init_send_pos(std::ostream * err);
  
  bool send_pos(double x, double y, double theta,
		double velocity, double steering,
		std::ostream * err);
  
  bool cleanup_send_pos(std::ostream * err);
  
  bool init_send_laser(std::ostream * err);
  
  bool send_laser(const sfl::Scanner & scanner,
		  std::ostream * err);
  
  bool cleanup_send_laser(std::ostream * err);
  
}

#endif // NPM_WRAP_CARMEN_HPP

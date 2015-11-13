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


#include "Drive.hpp"
#include <sfl/util/Frame.hpp>


using namespace sfl;
using namespace boost;


namespace npm {


  Drive::
  Drive()
    : m_cache(new Frame())
  {
  }


  Drive::
  ~Drive()
  {
  }


  shared_ptr<Frame> Drive::
  NextPose(const Frame & current, double timestep)
  {
    shared_ptr<Frame> result(ComputeNextPose(current, timestep));
    m_cache.reset(new Frame( * result));
    return result;
  }


  shared_ptr<const Frame> Drive::
  PoseCache() const
  {
    return m_cache;
  }

}

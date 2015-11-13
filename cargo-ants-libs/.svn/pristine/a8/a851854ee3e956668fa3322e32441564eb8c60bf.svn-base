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


#include "DiffDrive.hpp"
#include <sfl/api/DiffDriveChannel.hpp>
#include <sfl/util/Frame.hpp>
#include <sfl/util/numeric.hpp>


using namespace sfl;
using namespace boost;


namespace {
  
  class Channel
    : public sfl::DiffDriveChannel
  {
  public:
    explicit Channel(npm::DiffDrive * drive): m_drive(drive) {}
    
    virtual bool SetSpeed(double qdl, double qdr)
    {
      m_drive->qdl = qdl;
      m_drive->qdr = qdr;
      return true;
    }
    
    virtual bool GetSpeed(double &qdl, double &qdr) const
    {
      qdl = m_drive->qdl;
      qdr = m_drive->qdr;
      return true;
    }
    
    npm::DiffDrive * m_drive;
  };
  
}


namespace npm {


  DiffDrive::
  DiffDrive(double _wheelbase, double _wheelradius)
    : wheelbase(_wheelbase), wheelradius(_wheelradius),
      qdl(0.0), qdr(0.0)
  {
  }
  
  
  boost::shared_ptr<DiffDriveChannel> DiffDrive::
  CreateChannel()
  {
    boost::shared_ptr<DiffDriveChannel> ch(new Channel(this));
    return ch;
  }
  
  
  ////    boost::shared_ptr<DiffDriveChannel> CreateNoisyChannel();
  
  
  shared_ptr<Frame> DiffDrive::
  ComputeNextPose(const Frame & current, double timestep) const
  {
    shared_ptr<Frame> result(new Frame(current));
  
    // actuator speed -> global speed
    double dl    = qdl  * wheelradius;
    double dr    = qdr * wheelradius;
    double v     = (dl + dr) / 2;
    double omega = (dr - dl) / wheelbase;
  
    // local kinematics
    double dtheta = omega * timestep;
    double dx, dy;
    double R;
    if(absval(dtheta) > epsilon){
      // use circular movement
      R = v / omega;
      dx = R * sin(dtheta);
      dy = R * (1 - cos(dtheta));
    }
    else {
      // approximate with linear movement
      R = v * timestep;
      dx = R * cos(0.5 * dtheta);
      dy = R * sin(0.5 * dtheta);
    }
  
    // rotate and transform to global frame
    current.RotateTo(dx, dy);
    result->Add(dx, dy, dtheta);
  
    return result;
  }
  
}

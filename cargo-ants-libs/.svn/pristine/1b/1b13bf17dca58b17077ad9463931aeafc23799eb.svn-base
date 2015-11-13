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


#ifndef NPM_SENSOR_HPP
#define NPM_SENSOR_HPP


#include <sfl/util/Line.hpp>
#include <boost/shared_ptr.hpp>


namespace npm {
  
  class RobotServer;
  class World;
  
  /**
     Abstract sensor. Also includes some common functionality clients
     shouldn't be concerned about: Each sensor belongs to a Robot (it's
     owner), and the owner's outline is invisible to that sensor.
  */
  class Sensor
  {
  public:
    /**
       Keep a reference to the robot which owns this sensors in order to
       know which lines are invisible.
       
       \note Don't use a boost::shared_ptr for the owner, that would
       create a reference loop because the owner maintains a shared
       pointer to the Sensor instance.
    */
    const RobotServer * owner;
    
    Sensor(const RobotServer * owner);
    virtual ~Sensor() { }
    
    /**
       Template method for updating the sensor. After calling
       InitUpdate(), this method invokes the visitor method
       World::UpdateSensor(), which in turn calls StepUpdate() on all
       lines in the world except those belonging to owner. After the
       visitor invokation, FinalizeUpdate() is called.
       
       The default implementations of InitUpdate() and FinalizeUpdate()
       are no-ops, so that subclasses that don't require special
       initialization or finalization can implement a single method.
    */
    void Update();
    
  protected:
    friend class Object;		// visits StepUpdate()
    
    const World & m_world;
    
    /**
       Template method for resetting the sensor, called before each
       update cycle. Empty by default.
    */
    virtual void InitUpdate();
    
    /**
       Template method for updating the sensor reading to reflect
       knowledge about the given line. Called on all lines in the world
       (through Update()), except those belonging to the owner of the
       sensor.
    */
    virtual void StepUpdate(const sfl::Line & line) = 0;
    
    /**
       Template method for finalizing the update cycle, called as the
       last operation from Update(). Empty by default.
    */
    virtual void FinalizeUpdate();
  };

}

#endif // NPM_SENSOR_HPP

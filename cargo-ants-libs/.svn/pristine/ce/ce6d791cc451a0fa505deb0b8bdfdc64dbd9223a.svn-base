/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#ifndef NPM_WORLD_HPP
#define NPM_WORLD_HPP

#include <fpplib/configurable.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>


namespace sfl {
  class Line;
  class TraversabilityMap;
}


namespace npm {

  class Object;  
  class RobotServer;  
  class WorldDrawing;
  class WorldCamera;
  class BBox;
  class Sensor;
  
  class KeyListener {
  public:
    virtual ~KeyListener() {}
    virtual void KeyPressed(unsigned char key) = 0;
  };
  
  
  /**
     Simulated world. Basically a container for objects and robots.
  */
  class World
    : public fpplib::Configurable
  {
  public:
    typedef std::vector<boost::shared_ptr<Object> > object_t;
    typedef std::vector<RobotServer *> robot_t;
    
    explicit World(const std::string & name);
    
    bool LoadBuiltin(const std::string & name);
    void LoadMini();
    void LoadExpo();
    void LoadStage();
    
    /**
       Adds lines to the world, based on the boundary between obstcale
       and non-obstacle cells in the given traversability map. Cells
       with a value of travmap.obstacle or higher are considered
       obstacles.
       
       \todo XXXX to do: plug this into the fpplib configuration approach.
    */
    void
    ApplyTraversability(const sfl::TraversabilityMap & travmap,
			bool simple = true);
    
    /**
       \todo XXXX always returns true, but needs bool return type for
       fpplib::Configurable::reflectCallback mechanism (at time of
       writing).
    */
    bool AddLine(const sfl::Line & line);
    
    /** Makes a copy of the argument. */
    void AddObject(const Object & object);
    
    void UpdateSensor(Sensor & sensor) const;
    
    /** \note Unlike World::AddObject(), which copy constructs the
	object and deletes it in World::~World(), the robots are kept in
	a list for reference only (Sensor updates) and are not
	deleted. */
    void AddRobot(RobotServer * robot);
    
    /** \note the world's "own" lines are stored in the first element */
    const object_t & GetObjects() const { return m_object; }

    const robot_t & GetRobots() const { return m_robot; }
    
    /** \note Can return 0. */
    boost::shared_ptr<const BBox> GetBounds() const { return m_bbox; }
    
    /** For dumping the map in a textfile. */
    void DumpLines(std::ostream & os, bool use_windows_eol) const;
    
    void AddKeyListener(boost::shared_ptr<KeyListener> listener) const;
    void DispatchKey(unsigned char key) const;
    
    
  private:
    friend class CheatSheet;
    
    robot_t m_robot;
    boost::shared_ptr<WorldDrawing> m_drawing;
    boost::shared_ptr<WorldCamera> m_camera;
    object_t m_object;
    boost::shared_ptr<BBox> m_bbox;
    
    mutable std::vector<boost::shared_ptr<KeyListener> > m_listener;
  };
  
}

#endif // NPM_WORLD_HPP

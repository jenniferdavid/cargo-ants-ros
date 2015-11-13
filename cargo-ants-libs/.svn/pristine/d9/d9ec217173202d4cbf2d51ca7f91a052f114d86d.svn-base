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


#ifndef SUNFLOWER_ODOMETRY_HPP
#define SUNFLOWER_ODOMETRY_HPP


#include <sfl/api/Timestamp.hpp>
#include <sfl/api/LocalizationInterface.hpp>
#include <boost/shared_ptr.hpp>
#include <map>


namespace sfl {
  
  
  class Pose;
  
  
  /**
     \todo nohal: rename to PoseHistory
  */
  class Odometry
  {
  public:
    typedef std::map<Timestamp, boost::shared_ptr<Pose> > history_t;
    
    explicit Odometry(boost::shared_ptr<sfl::LocalizationInterface> localization);
    
    void Clear() { m_history.clear(); }
    
    /**
       Uses LocalizationInterface to get the latest pose.
    */
    int Update();
    
    /**
       Update the pose history.
       
       \todo nohal: rename to Add, void retval
    */
    int Update(Pose const & pose);
    
    /**
       \return Copy of the current (most recent) pose in world
       frame. In the unlikely event that no pose is in the history
       (i.e. during initialisation), returns a default constructed
       instance.
    */
    boost::shared_ptr<const Pose> Get() const;
    
    /** Access to the pose history in case you want to do fancy stuff. */
    const history_t & GetHistory() const { return m_history; }
    
    /**
       Find closest matching pose in the history. If the history is
       empty, then the iterator will have a timestamp of
       Timestamp::first and a null Pose. It's up to the caller to see
       how well the returned iterator matches the wanted timestamp.
    */
    history_t::value_type Get(const Timestamp & t) const;
    
  private:
    history_t m_history;
    boost::shared_ptr<LocalizationInterface> m_localization;
  };
  
}

#endif // SUNFLOWER_ODOMETRY_HPP

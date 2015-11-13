/* 
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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
#include <stdio.h>
#include "Simulator.hpp"
#include "Actuator.hpp"
#include "Sensor.hpp"
#include "Process.hpp"
#include <boost/bind.hpp>
#include <cstdio>


namespace npm2 {
  
  
  Simulator::
  Simulator (string const & name)
    : fpplib::Configurable (name),
      timestep_ (0.1),
      state_ (PAUSE),
      erros_ (cout),
      window_width_ (200),
      window_height_ (200),
      window_posx_ (0),
      window_posy_ (0),
      world_ (new Object("world")),
      clock_ (0.0)
  {
    reflectParameter ("timestep", &timestep_,
		      /** \todo parameter guards have unclear ownership */
		      new fpplib::StrictlyPositiveGuard <double> ());
    reflectCallback <string> ("state", true,
			      boost::bind (&Simulator::setState, this, _1));
    
    fpplib::StrictlyPositiveGuard <int> * guard (new fpplib::StrictlyPositiveGuard <int> ());
    reflectParameter ("window_width", &window_width_, guard);
    reflectParameter ("window_height", &window_height_, guard);
    reflectParameter ("window_posx", &window_posx_, guard);
    reflectParameter ("window_posy", &window_posy_, guard);
  }
  
  
  Simulator::
  ~Simulator ()
  {
    delete world_;
    for (size_t ih (0); ih < hooks_.size(); ++ih) {
      if (hooks_[ih].own) {
	delete hooks_[ih].hook;
      }
    }
  }
  
  
  Simulator * Simulator::
  instance ()
  {
    static Simulator * instance (0);
    if ( ! instance) {
      instance = new Simulator ("simulator");
    }
    return instance;
  }
  
  
  Object * Simulator::
  world ()
  {
    return instance()->world_;
  }
  

  double Simulator::
  clock ()
  {
    return instance()->clock_;
  }
  
  
  void Simulator::
  addHook (bool own, SimulatorHook * hook)
  {
    hooks_.push_back (hook_entry_t (own, hook));
  }
  
  
  bool Simulator::
  setState (string const & value)
  {
    if ("pause" == value || "PAUSE" == value) {
      state_ = PAUSE;
    }
    else if ("step" == value || "STEP" == value) {
      state_ = STEP;
    }
    else if ("run" == value || "RUN" == value) {
      state_ = RUN;
    }
    else {
      return false;
    }
    return true;
  }
  
  
  static void recurse_integrate (Object * obj, double dt)
  {
    Actuator * act (dynamic_cast <Actuator*> (obj));
    if (act) {
      act->integrate (dt);
    }
    for (Object::child_iterator_t ic(obj->childBegin()); ic != obj->childEnd(); ++ic) {
      recurse_integrate (*ic, dt);
    }
  }
  
  
  bool Simulator::
  init (ostream & err)
  {
    world_->updateTransform ();
    
    for (size_t ih (0); ih < hooks_.size(); ++ih) {
      if ( ! hooks_[ih].hook->init (err)) {
	return false;
      }
    }
    
    clock_ = 0.0;
    
    return true;
  }
  
  
  void Simulator::
  tick (ostream & err)
  {
    simulateActuators (cout);
    simulateSensors (cout);
    simulateProcesses (cout);
    clock_ += timestep_;
  }


  void Simulator::
  simulateActuators (ostream & err)
  {
    for (size_t ih (0); ih < hooks_.size(); ++ih) {
      hooks_[ih].hook->preActuation (err);
    }
    recurse_integrate (world_, timestep_);
    world_->updateTransform ();
  }
  
  
  static void recurse_sense (Object const * world, Object * obj)
  {
    Sensor * sensor (dynamic_cast <Sensor*> (obj));
    if (sensor) {
      sensor->sensorReset ();
      world->updateSensor (sensor);
    }
    for (Object::child_iterator_t ic(obj->childBegin()); ic != obj->childEnd(); ++ic) {
      recurse_sense (world, *ic);
    }
  }
  
  
  void Simulator::
  simulateSensors (ostream & err)
  {
    for (size_t ih (0); ih < hooks_.size(); ++ih) {
      hooks_[ih].hook->preSensing (err);
    }
    recurse_sense (world_, world_);
  }
  
  
  void Simulator::
  simulateProcesses (ostream & err)
  {
    for (size_t ih (0); ih < hooks_.size(); ++ih) {
      hooks_[ih].hook->preProcessing (err);
    }
    for (size_t ii(0); ii < Process::registry.size(); ++ii) {
      Process::registry.at(ii)->process (*this, erros_);
    }
  }
  
}

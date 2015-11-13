/* Nepumuk Mobile Robot Simulator v2
 *
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

#include "Process.hpp"
#include "Simulator.hpp"


namespace npm2 {
  
  
  Process::registry_t Process::registry;
  
  
  Process::
  Process (string const & name)
    : fpplib::Configurable (name),
      state_ (READY)
  {
    registry.add (name, this);
  }
  
  
  Process::
  ~Process ()
  {
    registry.remove (name, this);
  }
  
  
  Process::state_t Process::
  process (Simulator const & sim, ostream & erros)
  {
    switch (state_) {
    case READY:
      state_ = init (erros);
      break;
    case RUNNING:
      state_ = run (sim.timestep_, erros);
      break;
    case FAILED:
      state_ = recover (erros);
      break;
    case DONE:
    default:
      //   do nothing
      break;
    }
    return state_;
  }
  
  
  Process::state_t Process::
  init (ostream & erros)
  {
    return RUNNING;
  }
  
  
  Process::state_t Process::
  recover (ostream & erros)
  {
    return FAILED;
  }
  
}

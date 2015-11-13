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

#ifndef NPM2_PROCESS_HPP
#define NPM2_PROCESS_HPP

#include <fpplib/configurable.hpp>
#include <iostream>


namespace npm2 {
  
  using namespace std;

  class Simulator;
  
  
  class Process
    : public fpplib::Configurable
  {
  public:
    typedef enum {
      READY,
      RUNNING,
      FAILED,
      DONE
    } state_t;
    
    typedef fpplib::Registry<Process, false> registry_t;
    static registry_t registry;
    
    explicit Process (string const & name);
    virtual ~Process ();
    
    state_t process (Simulator const & sim, ostream & erros);
    state_t getState () const { return state_; }
    
  protected:
    virtual state_t init (ostream & erros);
    virtual state_t run (double timestep, ostream & erros) = 0;
    virtual state_t recover (ostream & erros);
    
  private:
    state_t state_;
  };
  
}

#endif // NPM2_PROCESS_HPP

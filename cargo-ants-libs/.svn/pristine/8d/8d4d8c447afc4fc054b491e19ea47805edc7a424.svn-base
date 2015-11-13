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

#ifndef NPM2_SIMULATOR_HPP
#define NPM2_SIMULATOR_HPP

#include <fpplib/configurable.hpp>


namespace npm2 {
  
  using namespace std;
  
  class Object;
  
  class SimulatorHook
  {
  public:
    virtual ~SimulatorHook () {}
    
    virtual bool init (ostream & err) = 0;
    virtual void preActuation (ostream & err) = 0;
    virtual void preSensing (ostream & err) = 0;
    virtual void preProcessing (ostream & err) = 0;
  };
  
  
  class Simulator
    : public fpplib::Configurable
  {
  private:
    explicit Simulator (string const & name);
    
  public:
    typedef enum {
      PAUSE,
      STEP,
      RUN
    } state_t;
    
    virtual ~Simulator();
    
    static Simulator * instance ();
    static Object * world ();
    static double clock ();
    
    bool setState (string const & value);
    
    void addHook (bool own, SimulatorHook * hook);
    bool init (ostream & err);
    void tick (ostream & err);
    
    // XXXX to do: some or most of the fields should be protected
    
    double timestep_;
    state_t state_;
    ostream & erros_;
    int window_width_;
    int window_height_;
    int window_posx_;
    int window_posy_;
    
  private:
    void simulateActuators (ostream & err);
    void simulateSensors (ostream & err);
    void simulateProcesses (ostream & err);

    Object * world_;
    double clock_;
    
    struct hook_entry_t {
      hook_entry_t (bool own_, SimulatorHook * hook_)
	: own (own_), hook (hook_) {}
      bool own;
      SimulatorHook * hook;
    };
    typedef vector <hook_entry_t> hooks_t;
    hooks_t hooks_;
  };
  
}

#endif // NPM2_SIMULATOR_HPP

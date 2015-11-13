/* Nepumuk Mobile Robot Simulator v2
 * 
 * Copyright (C) 2014 Roland Philippsen. All rights resevred.
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

#include <npm2/Plugin.hpp>
#include <npm2/Factory.hpp>
#include <npm2/Simulator.hpp>
#include <npm2/Process.hpp>
#include <npm2/KinematicControl.hpp>
#include <npm2/Object.hpp>
#include <sfl/api/Goal.hpp>

using namespace npm2;


class PickPlaceProcess
  : public Process
{
public:
  explicit PickPlaceProcess (string const & name);
  
protected:
  typedef enum {
    PICK,
    PLACE,
    EMPTY
  } mode_t;
  
  virtual state_t init (ostream & erros);
  virtual state_t run (double timestep, ostream & erros);
  
  KinematicControl * control_;
  vector <Goal> goals_;
  size_t current_;
  Goal goal_;
  Object * container_;
  mode_t mode_;
};


//////////////////////////////////////////////////


int npm2_plugin_init ()
{
  Factory::instance().declare <PickPlaceProcess> ("PickPlaceProcess");
  return 0;
}


//////////////////////////////////////////////////


PickPlaceProcess::
PickPlaceProcess (string const & name)
  : Process (name)
{
  reflectVectorParameter ("goals", &goals_);
  reflectSlot ("control", &control_);
  reflectSlot ("container", &container_);
}
  
  
PickPlaceProcess::state_t PickPlaceProcess::
init (ostream & erros)
{
  if ( ! control_) {
    erros << "PickPlaceProcess needs a KinematicControl instance\n";
    return FAILED;
  }
  if (goals_.empty()) {
    erros << "PickPlaceProcess needs some goals to work with\n";
    return FAILED;
  }
  if ( ! container_) {
    erros << "PickPlaceProcess needs a container (Object instance)\n";
    return FAILED;
  }
  
  mode_ = PICK;
  goal_.Set (container_->getGlobal().X(),
	     container_->getGlobal().Y(),
	     container_->getGlobal().Theta(),
	     0.1, 0.05);
  control_->setGoal (goal_);
  current_ = 0;
  
  return RUNNING;
}
  
  
PickPlaceProcess::state_t PickPlaceProcess::
run (double timestep, ostream & erros)
{
  if ( ! goal_.Reached (control_->drive_->getParent()->getGlobal(), true)) {
    return RUNNING;
  }
  
  switch (mode_) {
  case PICK:
    container_->attach (control_->drive_->getParent());
    goal_ = goals_[current_];
    control_->setGoal (goals_[current_]);
    mode_ = PLACE;
    break;
  case PLACE:
    container_->attach (Simulator::world());
    current_ = (current_ + 1) % goals_.size();
    goal_ = goals_[current_];
    control_->setGoal (goals_[current_]);
    mode_ = EMPTY;
    break;
  case EMPTY:
  default:
    goal_.Set (container_->getGlobal().X(),
	       container_->getGlobal().Y(),
	       container_->getGlobal().Theta(),
	       0.1, 0.05);
    control_->setGoal (goal_);
    mode_ = PICK;
  }
  
  return RUNNING;
}

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
#include <stdio.h>
#include <npm2/Plugin.hpp>
#include <npm2/Factory.hpp>
#include <npm2/Process.hpp>
#include <npm2/KinematicControl.hpp>
#include <npm2/DifferentialDrive.hpp>
#include <npm2/RevoluteServo.hpp>
#include <npm2/RayDistanceSensor.hpp>
#include <npm2/Object.hpp>
#include <npm2/DifferentialTrailerDrive.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/api/Goal.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <cstdio>

using namespace npm2;


class AliceProcess
  : public Process
{
public:
  explicit AliceProcess (string const & name);
  
  bool attach (string const & base_name);
  
  virtual state_t init (ostream & erros);
  virtual state_t run (double timestep, ostream & erros);
  
  DifferentialDrive * drive_;
  RevoluteServo * servo_;
  RayDistanceSensor * sensor_;
};


class BobProcess
  : public Process
{
public:
  explicit BobProcess (string const & name);
  
  virtual state_t run (double timestep, ostream & erros);
  
  DifferentialTrailerDrive * drive_;
  Object * foo_;
};


class CharlieProcess
  : public Process
{
public:
  explicit CharlieProcess (string const & name);
  
  static CharlieProcess * create (string const & name, Object * parent, Frame const & mount);
  
  virtual state_t init (ostream & erros);
  virtual state_t run (double timestep, ostream & erros);
  
  DifferentialDrive * drive_;
  RayDistanceSensor * left_;
  RayDistanceSensor * right_;
  double gain_;
  double offset_;
};


class DennisProcess
  : public Process
{
public:
  explicit DennisProcess (string const & name);
  
protected:
  virtual state_t init (ostream & erros);
  virtual state_t run (double timestep, ostream & erros);
  
  KinematicControl * control_;
  vector <Goal> goals_;
  size_t current_;
};


//////////////////////////////////////////////////


int npm2_plugin_init ()
{
  Factory::instance().declare <AliceProcess> ("AliceProcess");
  Factory::instance().declare <BobProcess> ("BobProcess");
  Factory::instance().declare <CharlieProcess> ("CharlieProcess");
  Factory::instance().declare <DennisProcess> ("DennisProcess");
  return 0;
}


//////////////////////////////////////////////////


AliceProcess::
AliceProcess (string const & name)
  : Process (name),
    drive_ (0),
    servo_ (0),
    sensor_ (0)
{
  reflectSlot ("drive", &drive_);
  reflectSlot ("servo", &servo_);
  reflectSlot ("sensor", &sensor_);
  reflectCallback <string> ("attach", true, boost::bind (&AliceProcess::attach, this, _1));
}
  
  
bool AliceProcess::
attach (string const & base_name)
{
  Object * obj (Object::registry.find (base_name));
  if ( ! obj) {
    printf ("* cannot find Object %s\n", base_name.c_str());
    return false;
  }
    
  drive_ = obj->find <DifferentialDrive> (name + "_drive");
  if ( ! drive_) {
    printf ("* cannot find drive %s%s\n", name.c_str(), "_drive");
    return false;
  }
    
  servo_ = obj->find <RevoluteServo> (name + "_servo");
  if ( ! servo_) {
    printf ("* cannot find servo %s%s\n", name.c_str(), "_servo");
    return false;
  }

  sensor_ = obj->find <RayDistanceSensor> (name + "_sensor");
  if ( ! sensor_) {
    printf ("* cannot find sensor %s%s\n", name.c_str(), "_sensor");
    return false;
  }
    
  return true;
}
  
  
AliceProcess::state_t AliceProcess::
init (ostream & erros)
{
  if (( ! drive_) || ( ! servo_) || ( ! sensor_)) {
    erros << "AliceProcess " << name << " needs a drive, servo, and sensor\n";
    return FAILED;
  }
  return RUNNING;
}
  
  
AliceProcess::state_t AliceProcess::
run (double timestep, ostream & erros)
{
  drive_->setSpeed (0.02, 0.04);
    
  static double amp (5.0 * M_PI / 180.0);
  static double omg (2.0 * M_PI / 5.0);
  static size_t count (0);
  servo_->setAngle (amp * cos (omg * (count++) * timestep));
    
  return RUNNING;
}


//////////////////////////////////////////////////


BobProcess::
BobProcess (string const & name)
  : Process (name),
    drive_ (0),
    foo_ (0)
{
  reflectSlot ("drive", &drive_);
  reflectSlot ("foo", &foo_);
}
  
  
BobProcess::state_t BobProcess::
run (double timestep, ostream & erros)
{
  if ( ! drive_) {
    erros << "BobProcess " << name << " is missing a drive\n";
    return FAILED;
  }
    
  double thref;
  if (drive_->getParent()->getGlobal().X() > drive_->getParent()->getGlobal().Y()) {
    if (drive_->getParent()->getGlobal().X() > - drive_->getParent()->getGlobal().Y()) {
      thref = 100.0 * M_PI / 180.0;
    }
    else {
      thref = 10.0 * M_PI / 180.0;
    }
  }
  else {
    if (drive_->getParent()->getGlobal().X() > - drive_->getParent()->getGlobal().Y()) {
      thref = 190.0 * M_PI / 180.0;
    }
    else {
      thref = -80.0 * M_PI / 180.0;
    }
  }
  double const dhead (mod2pi (thref - drive_->getParent()->getGlobal().Theta()));
  static double const dth (5.0 * M_PI / 180.0);
  if (fabs (dhead) <= dth) {
    drive_->setSpeed (0.04, 0.04);
  }
  else if (dhead > 0.0) {
    drive_->setSpeed (0.0, 0.04);
  }
  else {
    drive_->setSpeed (0.04, 0.0);
  }
    
  if (foo_) {
    static Object * world (0);
    static int cnt (100);
    if (0 == --cnt) {
      if (drive_->getParent() != foo_->getParent()) {
	world = foo_->attach (drive_->getParent());
	printf ("drive\n");
      }
      else {
	foo_->attach (world);
	printf ("world\n");
      }
      cnt = 100;
    }

    printf ("%d  %g\n", cnt, timestep);
  }
    
  return RUNNING;
}


//////////////////////////////////////////////////


CharlieProcess::
CharlieProcess (string const & name)
  : Process (name),
    drive_ (0),
    left_ (0),
    right_ (0),
    gain_ (0.3),
    offset_ (0.1)
{
  reflectSlot ("drive", &drive_);
  reflectSlot ("left", &left_);
  reflectSlot ("right", &right_);
  reflectParameter ("gain", &gain_);
  reflectParameter ("offset", &offset_);
}
  

CharlieProcess * CharlieProcess::
create (string const & name, Object * parent, Frame const & mount)
{
  CharlieProcess * charlie (new CharlieProcess (name));
    
  Object * base (new Object (name + "_base"));
  base->attach (parent);
  base->mount_ = mount;
  base->addLine (Line (-0.2, -0.4,  0.4, -0.2));
  base->addLine (Line (-0.2,  0.4,  0.4,  0.2));
  base->addLine (Line ( 0.4, -0.2,  0.4,  0.2));
  base->addLine (Line (-0.2,  0.4, -0.2, -0.4));
    
  charlie->drive_ = new DifferentialDrive (name + "_drive");
  charlie->drive_->attach (base);
  charlie->drive_->wheel_base_ = 0.3;
  charlie->drive_->wheel_radius_ = 0.2;
    
  charlie->left_ = new RayDistanceSensor (name + "_left");
  charlie->left_->attach (base);
  charlie->left_->mount_.Set (0.1, -0.1, -0.35);
  charlie->left_->max_distance_ = 2.5;
    
  charlie->right_ = new RayDistanceSensor (name + "_right");
  charlie->right_->attach (base);
  charlie->right_->mount_.Set (0.1, 0.1, 0.35);
  charlie->right_->max_distance_ = 2.5;
    
  return charlie;
}
  
  
CharlieProcess::state_t CharlieProcess::
init (ostream & erros)
{
  if ( ! drive_) {
    erros << "CharlieProcess " << name << " needs a drive\n";
    return FAILED;
  }
  if ( ! left_) {
    erros << "CharlieProcess " << name << " needs a left sensor\n";
    return FAILED;
  }
  if ( ! right_) {
    erros << "CharlieProcess " << name << " needs a right sensor\n";
    return FAILED;
  }
  return RUNNING;
}
  
  
CharlieProcess::state_t CharlieProcess::
run (double timestep, ostream & erros)
{
  double const wl (pow ((right_->distance_ - offset_) / right_->max_distance_, 2.0));
  double const wr (pow ((left_->distance_ - offset_) / left_->max_distance_, 2.0));
  drive_->setSpeed (gain_ * wl, gain_ * wr);
  return RUNNING;
}


//////////////////////////////////////////////////


DennisProcess::
DennisProcess (string const & name)
  : Process (name)
{
  reflectVectorParameter ("goals", &goals_);
  reflectSlot ("control", &control_);
}
  
  
DennisProcess::state_t DennisProcess::
init (ostream & erros)
{
  if ( ! control_) {
    erros << "DennisProcess needs a KinematicControl instance\n";
    return FAILED;
  }
  if (goals_.empty()) {
    erros << "DennisProcess needs some goals to work with\n";
    return FAILED;
  }
    
  current_ = 0;
  control_->setGoal (goals_[current_]);
    
  return RUNNING;
}
  
  
DennisProcess::state_t DennisProcess::
run (double timestep, ostream & erros)
{
  if ( ! goals_[current_].Reached (control_->drive_->getParent()->getGlobal(), true)) {
    return RUNNING;
  }
  current_ = (current_ + 1) % goals_.size();
  control_->setGoal (goals_[current_]);
  return RUNNING;
}

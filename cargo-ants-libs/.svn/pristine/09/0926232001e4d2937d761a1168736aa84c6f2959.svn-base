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
#include <npm2/Object.hpp>
#include <npm2/Simulator.hpp>
#include <npm2/Factory.hpp>
#include <sfl/util/Line.hpp>
#include <limits>
#include <cmath>
#include <stdlib.h>
#include <sys/time.h>

using namespace std;


class ContainerTeleport
  : public npm2::SimulatorHook,
    public fpplib::Configurable
{
public:
  ContainerTeleport ();
  
  virtual bool init (ostream & err);
  virtual void preActuation (ostream & err);
  virtual void preSensing (ostream & err);
  virtual void preProcessing (ostream & err);
  
  npm2::Object * container_;
  sfl::Line bounds_;
  bool container_attached_;
};

static ContainerTeleport ct;


//////////////////////////////////////////////////


int npm2_plugin_init ()
{
  npm2::Factory::instance().declareSingleton <ContainerTeleport> ("ContainerTeleport", &ct);
  npm2::Simulator::instance()->addHook (false, &ct);
  
  struct timeval tt;
  gettimeofday (&tt, NULL);
  srand (tt.tv_usec);
  
  return 0;
}


//////////////////////////////////////////////////


ContainerTeleport::
ContainerTeleport ()
  : fpplib::Configurable ("ContainerTeleport"),
    container_ (0),
    bounds_ (0.0, 0.0, 0.0, 0.0),
    container_attached_ (false)
{
  reflectSlot ("container", &container_);
  reflectParameter ("bounds", &bounds_);
}


bool ContainerTeleport::
init (ostream & err)
{
  return true;
}


void ContainerTeleport::
preActuation (ostream & err)
{
  if ( ! container_) {
    err << "ContainerTeleport: undefined container\n";
    return;
  }
  if (bounds_.X1() - bounds_.X0() < 1e-3) {
    err << "ContainerTeleport: undefined or invalid bounds\n";
    return;
  }
  
  if (container_attached_) {
    if (container_->getParent() == npm2::Simulator::world()) {
      container_attached_ = false;
      static double const nn (1.0 / std::numeric_limits <unsigned int> ::max());
      double const px (bounds_.X0() + (bounds_.X1() - bounds_.X0()) * rand() * nn);
      double const py (bounds_.Y0() + (bounds_.Y1() - bounds_.Y0()) * rand() * nn);
      double const pth (2 * M_PI * rand() * nn);
      container_->mount_.Set (px, py, pth);
      container_->motion_.Set (0.0, 0.0, 0.0);
    }
  }
  else {
    if (container_->getParent() != npm2::Simulator::world()) {
      container_attached_ = true;
    }
  }

}


void ContainerTeleport::
preSensing (ostream & err)
{
}


void ContainerTeleport::
preProcessing (ostream & err)
{
}

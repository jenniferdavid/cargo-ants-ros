/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#include "VisualRobox.hpp"
#include "MPDrawing.hpp"
#include "OCamera.hpp"
#include "ODrawing.hpp"
#include "DODrawing.hpp"
#include "DWDrawing.hpp"
#include "RHDrawing.hpp"
#include "BBDrawing.hpp"
#include "GridLayerCamera.hpp"
#include "GridLayerDrawing.hpp"
#include <npm/gfx/OdometryDrawing.hpp>
#include <npm/gfx/StillCamera.hpp>
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/expo/expo_parameters.hpp>
#include <sfl/bband/ReplanHandler.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include <sfl/dwa/SpeedObjective.hpp>
#include <sfl/api/RobotModel.hpp>

using namespace boost;


namespace npm {  
  
  VisualRobox::
  VisualRobox(std::string const & name,
	      expo::expo_parameters const & params,
	      boost::shared_ptr<sfl::Hull> hull,
	      boost::shared_ptr<sfl::LocalizationInterface> localization,
	      boost::shared_ptr<sfl::DiffDriveChannel> drive,
	      boost::shared_ptr<sfl::Multiscanner> mscan)
    : expo::Robox(params, hull, localization, drive, mscan)
  {
    AddDrawing(new MPDrawing(name + "_goaldrawing", *motionPlanner));
    AddDrawing(new DWDrawing(name + "_dwdrawing", *dynamicWindow));
    AddDrawing(new ODrawing(name + "_dodrawing", distanceObjective, dynamicWindow));
    AddDrawing(new ODrawing(name + "_hodrawing", headingObjective, dynamicWindow));
    AddDrawing(new ODrawing(name + "_sodrawing", speedObjective, dynamicWindow));
    sfl::ReplanHandler const * rph(0);
    if (params.bband_enabled) {
      rph = dynamic_cast<sfl::ReplanHandler const *>(bubbleBand->GetReplanHandler());
      if (rph) {
	AddDrawing(new RHDrawing(name + "_rhdrawing",
				 rph,
				 RHDrawing::AUTODETECT));
      }
      AddDrawing(new BBDrawing(name + "_bbdrawing",
			       *bubbleBand,
			       BBDrawing::AUTODETECT));
      if (rph) {
	AddDrawing(new GridLayerDrawing(name + "_local_gldrawing",
					rph->GetNF1(),
					false));
	AddDrawing(new GridLayerDrawing(name + "_global_gldrawing",
					rph->GetNF1(),
					true));
      }
    }
    AddDrawing(new OdometryDrawing(name + "_odomdrawing",
				   *odometry,
				   robotModel->WheelBase() / 2));
    AddDrawing(new DODrawing(name + "_collisiondrawing",
			     distanceObjective,
			     headingObjective,
			     dynamicWindow,
			     robotModel));
    
    AddCamera(new StillCamera(name + "_dwcamera",
			      0,
			      0,
			      dynamicWindow->Dimension(),
			      dynamicWindow->Dimension()));
    AddCamera(new OCamera(name + "_ocamera", *dynamicWindow));
    if (rph) {
      AddCamera(new GridLayerCamera(name + "_local_glcamera",
				    rph->GetNF1()));
    }
    double a, b, c, d;
    distanceObjective->GetRange(a, b, c, d);
    AddCamera(new StillCamera(name + "_collisioncamera", a, b, c, d));
  }
  
  
  void VisualRobox::
  AddDrawing(Drawing * drawing)
  {
    m_drawing.push_back(shared_ptr<Drawing>(drawing));
  }
  
  
  void VisualRobox::
  AddCamera(Camera * camera)
  {
    m_camera.push_back(shared_ptr<Camera>(camera));
  }
  
}

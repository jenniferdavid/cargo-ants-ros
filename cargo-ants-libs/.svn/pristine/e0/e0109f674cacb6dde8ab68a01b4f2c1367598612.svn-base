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

#ifndef NPM_VISUAL_ROBOX_HPP
#define NPM_VISUAL_ROBOX_HPP


#include <sfl/expo/Robox.hpp>
#include <vector>


namespace npm {
  
  class Drawing;
  class Camera;
  
  
  class VisualRobox
    : public expo::Robox
  {
  public:
    VisualRobox(std::string const & name,
		expo::expo_parameters const & params,
		boost::shared_ptr<sfl::Hull> hull,
		boost::shared_ptr<sfl::LocalizationInterface> localization,
		boost::shared_ptr<sfl::DiffDriveChannel> drive,
		boost::shared_ptr<sfl::Multiscanner> mscan);
    
  protected:
    void AddDrawing(Drawing * drawing);
    void AddCamera(Camera * camera);
    
    std::vector<boost::shared_ptr<Drawing> > m_drawing;
    std::vector<boost::shared_ptr<Camera> > m_camera;
  };
  
}

#endif // NPM_VISUAL_ROBOX_HPP

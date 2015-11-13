/* Nepumuk Mpbile Robot Simulator v2
 *
 * Copyright (C) 2004 Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen
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


#include "Camera.hpp"


namespace npm2 {
  
  
  Camera::registry_t Camera::registry;
  
  
  Camera::
  Camera (const string & name, const string & _comment)
    : fpplib::Configurable (name),
      comment (_comment)
  {
    registry.add (name, this);
  }
  
  
  Camera::
  ~Camera ()
  {
    registry.remove (name, this);
  }
  
}

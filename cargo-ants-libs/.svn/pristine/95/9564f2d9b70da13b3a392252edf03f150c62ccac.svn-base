/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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

#ifndef NPM_NOISE_MODEL_HPP
#define NPM_NOISE_MODEL_HPP


namespace npm {
  
  class NoiseModel
  {
  public:
    /**
       If one of min_factor or max_factor are <=0, then the
       proportional component is ignored. If min_offset>max_offset,
       then the absolute component is ignored.
    */
    NoiseModel(double min_factor, double max_factor,
	       double min_offset, double max_offset);
    
    const double operator () (double value) const;
    
    double min_factor, max_factor, min_offset, max_offset;
  };
  
}

#endif // NPM_NOISE_MODEL_HPP

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

#include "NoiseModel.hpp"
#include <sfl/util/Random.hpp>
#include <err.h>
#include <stdlib.h>

using namespace sfl;

namespace npm {
  

  NoiseModel::
  NoiseModel(double _min_factor, double _max_factor,
	     double _min_offset, double _max_offset)
    : min_factor(_min_factor),
      max_factor(_max_factor),
      min_offset(_min_offset),
      max_offset(_max_offset)
  {
  }
  
  
  const double NoiseModel::
  operator () (double value)
    const
  {
		try {
			if((min_factor > 0) && (max_factor > 0) && (min_factor < max_factor))
				value *= Random::Uniform(min_factor, max_factor);
			if(min_offset < max_offset)
				value += Random::Uniform(min_offset, max_offset);
		}
		catch (std::runtime_error ee) {
			errx(EXIT_FAILURE, "exception in NoiseModel(): %s", ee.what());
		}
    return value;
  }

}

/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#include "Objective.hpp"
#include "DynamicWindow.hpp"
#include <sfl/util/numeric.hpp>
#include <limits>


using std::numeric_limits;


namespace sfl {


  const double Objective::minValue(0);
  const double Objective::maxValue(1);
  
  
  Objective::
  Objective(const DynamicWindow & dynamic_window):
    dimension(dynamic_window.Dimension()),
    m_dynamic_window(dynamic_window),
    m_value(dimension, dimension)
  {
  }
  
  
  void Objective::
  Rescale(size_t qdlMin, size_t qdlMax, size_t qdrMin, size_t qdrMax)
  {
    double min(numeric_limits<double>::max());
    double max(numeric_limits<double>::min());
    for(size_t l = qdlMin; l <= qdlMax; ++l)
      for(size_t r = qdrMin; r <= qdrMax; ++r)
	if(m_dynamic_window.Admissible(l, r)){
	  if(m_value[l][r] < min)
	    min = m_value[l][r];
	  if(m_value[l][r] > max)
	    max = m_value[l][r];
	}
    if((max - min) < epsilon) return;
    const double scale(1 / (max - min));
    for(size_t l = qdlMin; l <= qdlMax; ++l)
      for(size_t r = qdrMin; r <= qdrMax; ++r)
	if(m_dynamic_window.Admissible(l, r))
	  m_value[l][r] = scale * (m_value[l][r] - min);
  }
  
  
  double Objective::
  Min(size_t qdlMin, size_t qdlMax, size_t qdrMin, size_t qdrMax) const
  {
    double min(numeric_limits<double>::max());
    for(size_t l = qdlMin; l <= qdlMax; ++l)
      for(size_t r = qdrMin; r <= qdrMax; ++r)
	if(m_dynamic_window.Admissible(l, r))
	  if(m_value[l][r] < min)
	    min = m_value[l][r];
    return min;
  }
  
  
  double Objective::
  Max(size_t qdlMin, size_t qdlMax, size_t qdrMin, size_t qdrMax) const
  {
    double max(numeric_limits<double>::min());
    for(size_t l = qdlMin; l <= qdlMax; ++l)
      for(size_t r = qdrMin; r <= qdrMax; ++r)
	if(m_dynamic_window.Admissible(l, r))
	  if(m_value[l][r] > max)
	    max = m_value[l][r];
    return max;
  }
  
}

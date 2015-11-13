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


#include "NF1Wave.hpp"
#include "NF1.hpp"
#include <limits>


using namespace std;


namespace sfl {
  
  
  const double NF1Wave::FREE(-2);
  const double NF1Wave::OBSTACLE(-1);
  const double NF1Wave::GOAL(0);
  NF1Wave::sindexlist_t NF1Wave::propagation_neighbor;
  NF1Wave::sindexlist_t NF1Wave::gradient_neighbor;
  
  
  NF1Wave::
  NF1Wave()
  {
    if(propagation_neighbor.empty()){
      propagation_neighbor.push_back(sindex_t( 1,  0));
      propagation_neighbor.push_back(sindex_t( 0,  1));
      propagation_neighbor.push_back(sindex_t(-1,  0));
      propagation_neighbor.push_back(sindex_t( 0, -1));
      gradient_neighbor.push_back(sindex_t( 1,  0));
      gradient_neighbor.push_back(sindex_t( 1,  1));
      gradient_neighbor.push_back(sindex_t( 0,  1));
      gradient_neighbor.push_back(sindex_t(-1,  1));
      gradient_neighbor.push_back(sindex_t(-1,  0));
      gradient_neighbor.push_back(sindex_t(-1, -1));
      gradient_neighbor.push_back(sindex_t( 0, -1));
      gradient_neighbor.push_back(sindex_t( 1, -1));
    }
  }
  
  
  void NF1Wave::
  Propagate(grid_t & grid) const
  {
    sindexlist_t current;
    sindexlist_t next(m_seed);
    while( ! next.empty()){
      next.swap(current);
      next.clear();
      for(size_t jj(0); jj < current.size(); ++jj){
	const sindex_t icell(current[jj]);
	const double nextval(grid[icell] + 1);
	for(size_t ii(0); ii < propagation_neighbor.size(); ++ii){
	  const sindex_t nbor(propagation_neighbor[ii] + icell);
	  if(grid.ValidIndex(nbor) && (FREE == grid[nbor])){
	    grid[nbor] = nextval;
	    next.push_back(nbor);
	  }
	}
      }
    }
  }
  
  
  NF1Wave::sindex_t NF1Wave::
  SmallestNeighbor(grid_t & grid, sindex_t index) const
  {
    double minval(numeric_limits<double>::max());
    sindex_t result(-1, -1);
    for(size_t ii(0); ii < gradient_neighbor.size(); ++ii){
      const sindex_t nbor(gradient_neighbor[ii] + index);
      if( ! grid.ValidIndex(nbor))
	continue;
      const double gridval(grid[nbor]);
      if((gridval >= 0) && (gridval < minval)){
	result = nbor;
	minval = gridval;
      }
    }
    return result;
  }
  
  
  void NF1Wave::
  Reset()
  {
    m_seed.clear();
  }
  
  
  void NF1Wave::
  AddSeed(sindex_t index)
  {
    m_seed.push_back(index);
  }
  
}

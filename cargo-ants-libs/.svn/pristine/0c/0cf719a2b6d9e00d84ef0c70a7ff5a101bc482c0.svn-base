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


#ifndef SUNFLOWER_NF1WAVE_H
#define SUNFLOWER_NF1WAVE_H


#include <sfl/util/array2d.hpp>
#include <vector>


namespace sfl {
  
  
  class NF1Wave
  {
  public:
    typedef vec2d<ssize_t> sindex_t;
    typedef array2d<double> grid_t;
    
    static const double FREE;//     = -2;
    static const double OBSTACLE;// = -1;
    static const double GOAL;//     =  0;
    
    NF1Wave();
    
    void Reset();
    void AddSeed(sindex_t index);
    void Propagate(grid_t & grid) const;
    sindex_t SmallestNeighbor(grid_t & grid, sindex_t index) const;
    
  private:
    typedef std::vector<sindex_t> sindexlist_t;
    
    static sindexlist_t propagation_neighbor;
    static sindexlist_t gradient_neighbor;
    sindexlist_t m_seed;
  };
  
}

#endif // SUNFLOWER_NF1WAVE_H

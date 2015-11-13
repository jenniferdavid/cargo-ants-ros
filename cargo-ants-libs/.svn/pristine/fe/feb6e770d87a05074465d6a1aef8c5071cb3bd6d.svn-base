/* 
 * Copyright (C) 2005 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#ifndef SFL_RANDOM_HPP
#define SFL_RANDOM_

#include <stdexcept>
#include <stdint.h>

namespace sfl {

  namespace Random {
    
    uint32_t Roll() throw(std::runtime_error);
    bool Uniform(double chance) throw(std::runtime_error);
    double Uniform(double vmin, double vmax) throw(std::runtime_error);
    int Uniform(int vmin, int vmax) throw(std::runtime_error);
    double Unit() throw(std::runtime_error);
  
  }

}

#endif // SFL_RANDOM_HPP

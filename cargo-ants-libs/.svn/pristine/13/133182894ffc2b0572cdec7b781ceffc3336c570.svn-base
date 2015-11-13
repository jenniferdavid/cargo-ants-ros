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

#include "Random.hpp"
#include "numeric.hpp"
#include <iostream>
#include <fstream>
#include <memory>
#include <limits>

using namespace std;

static auto_ptr<istream> urandom_is;

namespace sfl {
  
  namespace Random {
    
    
    uint32_t Roll() throw(runtime_error)
    {
      if (urandom_is.get() == 0)
	urandom_is = auto_ptr<istream>(new ifstream("/dev/urandom"));
      if (urandom_is.get() == 0)
	throw runtime_error("could not open /dev/urandom");
      uint32_t result(0);
      for (size_t ii(0); ii < 4; ++ii) {
	char byte;
	if ( ! ((*urandom_is) >> byte))
	  throw runtime_error("could not read from /dev/urandom");
	result |= (byte & 0xFF) << (ii * 8);
      }
      return result;
    }
    
    
    bool Uniform(double chance) throw(runtime_error)
    {
      return Roll() <= chance * numeric_limits<uint32_t>::max();
    }
    
    
    double Unit() throw(runtime_error)
    {
      return double(Roll()) / numeric_limits<uint32_t>::max();
    }
    
    
    double Uniform(double vmin, double vmax) throw(runtime_error)
    {
      const double unit(Unit());
      return minval(maxval(unit * vmin + (1 - unit) * vmax, vmin), vmax);
    }
    
    
    int Uniform(int vmin, int vmax) throw(runtime_error)
    {
      while(true){
	const int v(static_cast<int>(floor(Uniform(static_cast<double>(vmin),
						   static_cast<double>(vmax+1)))));
	if(v <= vmax)		// tiny chance of getting vmax + 1
	  return v;
      }
    }
    
  }
  
}

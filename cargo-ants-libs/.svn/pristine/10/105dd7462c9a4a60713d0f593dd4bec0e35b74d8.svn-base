/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx net>
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


#ifndef SUNFLOWER_VEC2D_HPP
#define SUNFLOWER_VEC2D_HPP


#include <iostream>


namespace sfl {
  
  
  template<typename value_t>
  class vec2d
  {
  public:
    vec2d(): v0(0), v1(0) {}
    vec2d(value_t _v0, value_t _v1): v0(_v0), v1(_v1) {}
    vec2d(const vec2d & orig): v0(orig.v0), v1(orig.v1) {}
    
    const bool operator == (const vec2d & rhs)
    { return (v0 == rhs.v0) && (v1 == rhs.v1); }
    
    const vec2d & operator = (const vec2d & rhs)
    { v0 = rhs.v0; v1 = rhs.v1; return * this; }
    
    const vec2d & operator -= (const vec2d & rhs)
    { v0 -= rhs.v0; v1 -= rhs.v1; return * this; }
    
    const vec2d & operator += (const vec2d & rhs)
    { v0 += rhs.v0; v1 += rhs.v1; return * this; }
    
    const vec2d & operator /= (const value_t & rhs)
    { v0 /= rhs; v1 /= rhs; return * this; }
    
    const vec2d & operator *= (const value_t & rhs)
    { v0 *= rhs; v1 *= rhs; return * this; }
    
    vec2d operator - (const vec2d & rhs) const
    { return vec2d(v0 - rhs.v0, v1 - rhs.v1); }
    
    vec2d operator + (const vec2d & rhs) const
    { return vec2d(v0 + rhs.v0, v1 + rhs.v1); }
    
    vec2d operator / (const value_t & rhs)
    { return vec2d(v0 / rhs, v1 / rhs); }
    
    vec2d operator * (const value_t & rhs)
    { return vec2d(v0 * rhs, v1 * rhs); }
    
    /** \note practical e.g. for ssize_t -> size_t, but a bit dangerous */
    template<typename convert_t>
    vec2d(const convert_t & convert): v0(convert.v0), v1(convert.v1) {}
    
    /** \note practical e.g. for ssize_t -> size_t, but a bit dangerous */
    template<typename convert_t>
    const vec2d & operator = (const convert_t & rhs)
    { v0 = rhs.v0; v1 = rhs.v1; return * this; }
    
    value_t v0, v1;
  };
  
  
  template<typename value_t>
  std::ostream & operator << (std::ostream & os, vec2d<value_t> const & vec)
  {
    os << "(" << vec.v0 << "  " << vec.v1 << ")";
    return os;
  }
  
}

#endif // SUNFLOWER_VEC2D_HPP

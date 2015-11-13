/* 
 * Copyright (C) 2005 Roland Philippsen <roland dot philippsen at gmx net>
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


#ifndef SUNFLOWER_ARRAY2D_HPP
#define SUNFLOWER_ARRAY2D_HPP


#include <sfl/util/vec2d.hpp>
#include <boost/scoped_array.hpp>

#ifdef OPENBSD
// is this a bug or a feature?
typedef __ssize_t ssize_t;
#endif // OPENBSD

#ifdef WIN32
# include <sfl/util/win32.hpp>
#endif // WIN32


namespace sfl {
  
  
  /**
     Simple 2D-array with "self destroying" underlying data.
     
     \note If you want to put this into an STL container, wrap it into a
     boost::shared_ptr to avoid problems with the non-copyable
     boost::scoped_array fields.
  */
  template<typename value_t>
  class array2d
  {
  public:
    typedef vec2d<size_t> index_t;
    typedef vec2d<ssize_t> sindex_t;
    typedef boost::scoped_array<value_t> inner_t;
    typedef boost::scoped_array<inner_t> outer_t;
    
    array2d(): size(0, 0), xsize(0), ysize(0) {}
    
    array2d(size_t _xsize, size_t _ysize)
      : size(_xsize, _ysize), xsize(_xsize), ysize(_ysize),
	data(new inner_t[_xsize]){
      for(size_t ix(0); ix < _xsize; ++ix)
	data[ix].reset(new value_t[_ysize]); }
    
    explicit array2d(index_t _size)
      : size(_size), xsize(_size.v0), ysize(_size.v1),
	data(new inner_t[_size.v0]) {
      for(size_t ix(0); ix < _size.v0; ++ix)
        data[ix].reset(new value_t[_size.v1]); }
    
    array2d(size_t _xsize, size_t _ysize, const value_t & init)
      : size(_xsize, _ysize), xsize(_xsize), ysize(_ysize),
	data(new inner_t[_xsize]) {
      for(size_t ix(0); ix < _xsize; ++ix){
	data[ix].reset(new value_t[_ysize]);
	for(size_t iy(0); iy < _ysize; ++iy) data[ix][iy] = init; } }
    
    array2d(index_t _size, const value_t & init)
      : size(_size), xsize(_size.v0), ysize(_size.v1),
	data(new inner_t[_size.v0]) {
      for(size_t ix(0); ix < _size.v0; ++ix){
	data[ix].reset(new value_t[_size.v1]);
	for(size_t iy(0); iy < _size.v1; ++iy) data[ix][iy] = init; } }
    
    array2d(const array2d & orig)
      : size(orig.size), xsize(orig.xsize), ysize(orig.ysize),
	data(new inner_t[orig.xsize]) {
      for(size_t ix(0); ix < orig.xsize; ++ix){
	data[ix].reset(new value_t[orig.ysize]);
	for(size_t iy(0); iy < orig.ysize; ++iy)
	  data[ix][iy] = orig.data[ix][iy]; } }
    
    inner_t & operator [] (size_t ix) { return data[ix]; }
    const inner_t & operator [] (size_t ix) const { return data[ix]; }
    
    value_t & operator [] (index_t idx) { return data[idx.v0][idx.v1];}
    const value_t & operator [] (index_t idx) const
    { return data[idx.v0][idx.v1];}
    
    value_t & operator [] (sindex_t idx) { return data[idx.v0][idx.v1];}
    const value_t & operator [] (sindex_t idx) const
    { return data[idx.v0][idx.v1];}
    
    bool ValidIndex(size_t ix, size_t iy) const
    { return (ix < xsize) && (iy < ysize); }
    
    bool ValidIndex(ssize_t ix, ssize_t iy) const {
      return (ix >= 0) && (ix < static_cast<ssize_t>(xsize))
	&& (iy >= 0) && (iy < static_cast<ssize_t>(ysize)); }
    
    bool ValidIndex(index_t idx) const
    { return (idx.v0 < xsize) && (idx.v1 < ysize); }
    
    bool ValidIndex(sindex_t idx) const {
      return (idx.v0 >= 0) && (idx.v0 < static_cast<ssize_t>(xsize))
	&& (idx.v1 >= 0) && (idx.v1 < static_cast<ssize_t>(ysize)); }
    
    void Fill(const value_t & value) {
      for(size_t ix(0); ix < xsize; ++ix)
	for(size_t iy(0); iy < ysize; ++iy)
	  data[ix][iy] = value; }
    
    const index_t size;
    const size_t xsize;
    const size_t ysize;
    outer_t data;
  };
  
}

#endif // SUNFLOWER_ARRAY2D_HPP

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

#ifndef SUNFLOWER_RINGBUF_HPP
#define SUNFLOWER_RINGBUF_HPP


#include <vector>


namespace sfl {
  
  template<typename value_t>
  class ringbuf
  {
  public:
    
    explicit ringbuf(size_t _len)
      : len(_len) {
      data.reserve(len);
    }
    
		/** Append a value to the ring buffer, possibly pushing out older
				value. Afterwards, ringbuf[0] will return the value last
				passed to push_back(). */
    void push_back(const value_t & val){
      if(data.size() >= len){
				data[oldest] = val;
				youngest = oldest;
				++oldest;
				if(oldest >= len)
					oldest = 0;
      }
      else if(data.empty()){
				youngest = 0;
				oldest = 0;
				data.push_back(val);
      }
      else{
				youngest = data.size();
				data.push_back(val);
      }
    }
    
    size_t size() const
    { return data.size(); }
    
    bool empty() const
    { return data.empty(); }
		
		/** ringbuf[0] returns the youngest elements, and ringbuf[size()-1] the oldest. */
    value_t & operator [] (size_t idx){
      if(youngest >= idx)
				return data[youngest - idx];
      if(data.size() >= len)
				return data[len + youngest - idx];
      return data[data.size() + youngest - idx - 1];
    }
    
		/** ringbuf[0] returns the youngest elements, and ringbuf[size()-1] the oldest. */
    const value_t & operator [] (size_t idx) const {
      if(youngest >= idx)
				return data[youngest - idx];
      if(data.size() >= len)
				return data[len + youngest - idx];
      return data[data.size() + youngest - idx - 1];
    }
    
		/** The maximum amount of values in the ringbuf. The size can be
				smaller than len, if fewer than len values have been added
				using push_back(). */
    const size_t len;
    std::vector<value_t> data;
    size_t youngest, oldest;
  };
  
}

#endif // SUNFLOWER_RINGBUF_HPP

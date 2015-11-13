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


#include "Lookup.hpp"
#include <sfl/util/numeric.hpp>
#include <iostream>
#include <limits>


using std::ostream;
using std::numeric_limits;


namespace sfl {
  
  
  Lookup::
  Lookup(const array2d<double> & buffer,
	 double _minValue, double _maxValue, double _invalidValue)
    : minValue(_minValue), maxValue(_maxValue), invalidValue(_invalidValue),
      m_key(buffer.xsize, buffer.ysize), m_no_valid(true)
  {
    // initialize histogram, but only with valid values
    histogram_t histogram;
    for(size_t ix(0); ix < buffer.xsize; ++ix)
      for(size_t iy(0); iy < buffer.ysize; ++iy)
	if((buffer[ix][iy] >= minValue) && (buffer[ix][iy] <= maxValue))
	  histogram.insert(buffer[ix][iy]);
    if(histogram.empty())
      return;			// empty buffer, no valid entry
    m_actualVmin = * histogram.begin();
    m_actualVmax = * histogram.rbegin();
    
    // initialize binlist...  yes, there are maxNbins + 1 bins,
    // because the last is "only" there to keep the upper (inclusive)
    // bound for the last interval
    const double scale((m_actualVmax - m_actualVmin) / maxNbins);
    binlist_t binlist;
    binlist.push_back(bin_s(m_actualVmin, invalidValue));
    for(size_t ii(1); ii < maxNbins; ++ii)
      binlist.push_back(bin_s(m_actualVmin + ii * scale, invalidValue));
    binlist.push_back(bin_s(m_actualVmax, invalidValue));
    
    if( ! LloydMax(histogram, binlist))
      return;			// oops, probably a bug... abort()?
    if( ! Quantize(buffer, histogram, binlist))
      return;			// oops, probably a bug... abort()?
    
    m_no_valid = false;
  }
  
  
  double Lookup::
  Get(size_t iqdl, size_t iqdr) const
  {
    if(m_no_valid || (iqdl >= m_key.xsize) || (iqdr >= m_key.ysize))
      return invalidValue;
    const key_t key(m_key[iqdl][iqdr]);
    if(key >= m_quantizer.size())
      return invalidValue;      
    return m_quantizer[key];
  }
  
  
  bool Lookup::
  LloydMax(const histogram_t & histogram, binlist_t & binlist) const
  {
    bool finished(false);
    while( ! finished){
      finished = true;
      
      // calculate levels
      const binlist_t::iterator ilast(--binlist_t::iterator(binlist.end()));
      binlist_t::iterator inext(binlist.begin());
      if((inext == ilast) || (inext == binlist.end()))
	return false;		// oops: empty or single-element binlist
      binlist_t::iterator icurr(inext);
      ++inext;
      while(inext != binlist.end()){
	double val;
	if(inext == ilast)
	  val = PMeanInc(histogram, icurr->bound, inext->bound);
	else
	  val = PMean(histogram, icurr->bound, inext->bound);
	if((icurr->val == invalidValue)
	   || (absval(icurr->val - val) > epsilon)){
	  finished = false;
	  icurr->val = val;
	}
	icurr = inext;
	++inext;
      }
      
      // remove empty bins, beware the last bin always contains
      // invalidValue but it has to stay in place for the upper bound
      // of the last interval, so we test against ilast, but also
      // check binlist.end() because std::list::erase() can return
      // std::list::end() if the last element is removed (which should
      // never happen here, but well... at least we don't go
      // currupting RAM elsewhere)
      for(icurr = binlist.begin();
	  (icurr != ilast) && (icurr != binlist.end());
	  ++icurr)
	if(icurr->val == invalidValue)
	  icurr = binlist.erase(icurr);
      
      // adjust boundaries, keeping the first last at the actual range
      // of the histogram
      icurr = binlist.begin();
      if((icurr == ilast) || (icurr == binlist.end()))
	return false;		// oops: removed too many bins
      icurr->bound = m_actualVmin;
      binlist_t::iterator iprev(icurr);
      ++icurr;
      while(icurr != ilast){
	icurr->bound = 0.5 * (iprev->val + icurr->val);
	iprev = icurr;
	++icurr;
      }
      icurr->bound = m_actualVmax; // should be redundant...
    } //   while(!finished)
    return true;
  }
  
  
  bool Lookup::
  Quantize(const array2d<double> & buffer,
	   const histogram_t & histogram, const binlist_t & binlist)
  {
    if(binlist.empty())
      return false;
    
    // initialize quantizer lookup
    m_quantizer.clear();
    const binlist_t::const_iterator
      ilast(--binlist_t::const_iterator(binlist.end()));
    binlist_t::const_iterator inext(binlist.begin());
    if((inext == ilast) || (inext == binlist.end()))
      return false;		// oops: empty or single-element binlist
    binlist_t::const_iterator icurr(inext);
    ++inext;
    while(inext != binlist.end()){
      if(inext == ilast)
	m_quantizer.push_back(PMinInc(histogram, icurr->bound, inext->bound));
      else
	m_quantizer.push_back(PMin(histogram, icurr->bound, inext->bound));
      icurr = inext;
      ++inext;
    }
    
    // quantize buffer and store it in m_key
    for(size_t ix(0); ix < buffer.xsize; ++ix)
      for(size_t iy(0); iy < buffer.ysize; ++iy){
	if((buffer[ix][iy] < minValue) || (buffer[ix][iy] > maxValue))
	  m_key[ix][iy] = m_quantizer.size(); // flag as invalidValue
	else{
	  m_key[ix][iy] = 0;
	  inext = binlist.begin();
	  ++inext;
	  while(inext != ilast){ // no need to test for ilast->bound
	    if(buffer[ix][iy] < inext->bound)
	      break;
	    ++m_key[ix][iy];
	    ++inext;
	  }
	}
      }
    
    return true;
  }


  double Lookup::
  PMean(const histogram_t & histogram, double from, double to) const
  {
    double sum(0);
    size_t count(0);
    for(histogram_t::const_iterator ih(histogram.begin());
	ih != histogram.end(); ++ih)
      if( * ih >= from){
	if( * ih < to){
	  sum += * ih;
	  ++count;
	}
	else
	  break;
      }
    if(count < 1)
      return invalidValue;
    return sum / count;
  }


  double Lookup::
  PMeanInc(const histogram_t & histogram, double from, double to) const
  {
    double sum(0);
    size_t count(0);
    for(histogram_t::const_iterator ih(histogram.begin());
	ih != histogram.end(); ++ih)
      if( * ih >= from){
	if( * ih <= to){
	  sum += * ih;
	  ++count;
	}
	else
	  break;
      }
    if(count < 1)
      return invalidValue;
    return sum / count;
  }
  
  
  double Lookup::
  PMin(const histogram_t & histogram, double from, double to) const
  {
    for(histogram_t::const_iterator ih(histogram.begin());
	ih != histogram.end(); ++ih)
      if(( * ih >= from) && ( * ih < to))
	return * ih;
    return invalidValue;
  }
  
  
  double Lookup::
  PMinInc(const histogram_t & histogram, double from, double to) const
  {
    for(histogram_t::const_iterator ih(histogram.begin());
	ih != histogram.end(); ++ih)
      if(( * ih >= from) && ( * ih <= to))
	return * ih;
    return invalidValue;
  }
  
  
  void Lookup::
  DumpStats(const array2d<double> & buffer, ostream & os) const
  {
    os << "Lookup::DumpStats():\n";
    if((buffer.xsize != m_key.xsize) || (buffer.ysize != m_key.ysize))
      os << "  dimension mismatch\n";
    else{
      double diffmin(numeric_limits<double>::max());
      double diffmax(numeric_limits<double>::min());
      double diffsum(0);
      size_t diffcount(0);
      size_t invalid_mismatch(0);
      for(size_t ix(0); ix < buffer.xsize; ++ix)
	for(size_t iy(0); iy < buffer.ysize; ++iy){
	  if((Get(ix, iy) == invalidValue)
	     && (buffer[ix][iy] >= minValue)
	     && (buffer[ix][iy] <= maxValue))
	    ++invalid_mismatch;
	  else{
	    const double diff(Get(ix, iy) - buffer[ix][iy]);
	    if(diff < diffmin) diffmin = diff;
	    if(diff > diffmax) diffmax = diff;
	    diffsum += diff;
	    ++diffcount;
	  }
	}
      os << "  diffmin   = " << diffmin << "\n"
	 << "  diffmax   = " << diffmax << "\n"
	 << "  diffcount = " << diffcount << "\n";
      if(diffcount > 0)
	os << "  diffmean  = " << diffsum / diffcount << "\n";
      os << "  invalid_mismatch = " << invalid_mismatch << "\n";
    }
  }
  
}

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


#ifndef SUNFLOWER_LOOKUP_HPP
#define SUNFLOWER_LOOKUP_HPP


#include <sfl/util/array2d.hpp>
#include <vector>
#include <list>
#include <set>


namespace sfl {


  class Lookup
  {
  public:
    Lookup(const array2d<double> & buffer,
	   double minValue, double maxValue,
	   /** must be outside [minValue, maxValue] (also used internally) */
	   double invalidValue);
    
    double Get(size_t iqdl, size_t iqdr) const;
    
    void DumpStats(const array2d<double> & buffer, std::ostream & os) const;
    
    const double minValue;
    const double maxValue;
    const double invalidValue;

  protected:
    struct bin_s {
      bin_s(double _bound, double _val): bound(_bound), val(_val) {}
      double bound, val;
    };
    
    typedef unsigned short key_t;
    typedef std::multiset<double> histogram_t;
    typedef std::list<bin_s> binlist_t;
    typedef std::vector<double> quantizer_t;
    
    static const size_t maxNbins = (1 << sizeof(key_t)) - 2;
    
    quantizer_t m_quantizer;
    array2d<key_t> m_key;
    bool m_no_valid;
    double m_actualVmin, m_actualVmax;
    
    /** prunes binlist in-place using Lloyd-Max algorithm */
    bool LloydMax(const histogram_t & histogram, binlist_t & binlist) const;
    
    /** \return mean of values (from <= val < to) or invalidValue */
    double PMean(const histogram_t & hist, double from, double to) const;
    
    /** \return mean of values (from <= val <= to) or invalidValue */
    double PMeanInc(const histogram_t & hist, double from, double to) const;
    
    /** \return min of values (from <= val < to) or invalidValue */
    double PMin(const histogram_t & hist, double from, double to) const;
    
    /** \return min of values (from <= val <= to) or invalidValue */
    double PMinInc(const histogram_t & hist, double from, double to) const;
    
    bool Quantize(const array2d<double> & buffer,
		  const histogram_t & histogram, const binlist_t & binlist);
  };

}

#endif // SUNFLOWER_LOOKUP_HPP

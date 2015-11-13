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


#ifndef SUNFLOWER_TIMESTAMP_HPP
#define SUNFLOWER_TIMESTAMP_HPP


#include <iosfwd>

#ifdef WIN32
# include <sfl/util/win32.hpp>
#else // WIN32
# include <time.h>
#endif // WIN32

struct timezone;

namespace sfl {


#ifdef WIN32
	typedef fake_timespec timespec_t;
#else // WIN32
	typedef struct ::timespec timespec_t;
#endif // WIN32


  /**
     Encapsulates timestamps.
  */
  class Timestamp
  {
  public:
    /** last representable moment */
    static const Timestamp first;

    /** first representable moment */
    static const Timestamp last;
    
    /** Default Timestamp with all zeros. */
    Timestamp();
    
    /** in case you need complete control... */
    Timestamp(long seconds, long nanoseconds);
    
    /**
       Converts a timestamp_t into a Timestamp instance. If you need
       a Timestamp of "unspecified" time, use last or first.
    */
    explicit Timestamp(const timespec_t & stamp);
    
    /** legacy */
    static const Timestamp & Last() { return last; }
    
    /** legacy */
    static const Timestamp & First() { return first; }
    
    /** Conversion operator. */
    Timestamp & operator = (const timespec_t & original);
    
    /** Prints the Timestamp as "seconds.nanoseconds". */
    friend std::ostream & operator << (std::ostream & os, const Timestamp & t);
    
    /**
       Basic comparison operator so that you can write code like this:
       
       \code
       Timestamp t0(some_data.GetTimestamp());
       Timestamp t1(other_data.GetTimestamp());
       if(t0 < t1)
         remove_old_data(some_data);
       else
         remove_old_data(other_data);
       \endcode

       It is also important for creating chronological maps of data,
       for example in order to implement a sensor history using STL:

       \code
       typedef std::multimap<Timestamp, double> distance_history_t;
       distance_history_t disthist;
       while(Whatever())
         disthist.insert(make_pair(Timestamp::Now(), ReadDistanceSensor()));
       // print readings in reverse:
       for(distance_history_t::reverse_iterator id(disthist.rbegin());
           id != disthist.rend();
	   ++id)
	 cerr << id->first << ": " << id->second << "\n";
       \endcode
    */
    friend bool operator < (const Timestamp & left, const Timestamp & right);

    friend bool operator <= (const Timestamp & left, const Timestamp & right);
    
    /** The opposite of Timestamp::operator<(). */
    friend bool operator > (const Timestamp & left, const Timestamp & right);
    
    friend bool operator >= (const Timestamp & left, const Timestamp & right);
    
    /** Equality operator. */
    friend bool operator == (const Timestamp & left, const Timestamp & right);
    
    /** Decrement operator. */
    Timestamp & operator -= (const Timestamp & other);
    
    /** Increment operator. */
    Timestamp & operator += (const Timestamp & other);
    
    /** const access to the underlying timespec instance */
    const timespec_t & Get() const
    { return m_stamp; }
    
    double ToSeconds() const;
    void FromSeconds(double sec);
    
    /** Convert a timestamp from a single-value milliseconds
	measure. */
    template<typename milli_t>
    static Timestamp FromMilliseconds(milli_t milli) {
      long const seconds(milli / 1000);
      long const nanoseconds((milli - 1000 * seconds) * 1000000);
      return Timestamp(seconds, nanoseconds);
    }
    
    /** Use gettimeofday() to set the time.
	\note If gettimeofday() fails, the returned timestamp will be 0. */
    static Timestamp Now(struct timezone * tz = 0);
    
  private:
    timespec_t m_stamp;
  };

}

sfl::Timestamp operator - (sfl::Timestamp const & lhs, sfl::Timestamp const & rhs);
sfl::Timestamp operator + (sfl::Timestamp const & lhs, sfl::Timestamp const & rhs);

#endif // SUNFLOWER_TIMESTAMP_HPP

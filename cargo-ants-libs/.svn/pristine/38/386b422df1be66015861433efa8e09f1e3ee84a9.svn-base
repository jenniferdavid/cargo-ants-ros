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


#include "Timestamp.hpp"
#include <sstream>
#include <limits>
#include <iomanip>
#include <cmath>
#include <sys/time.h>
#include <time.h>


using namespace std;


namespace sfl {
  
  
  const Timestamp Timestamp::first(numeric_limits<long>::min(),
				   numeric_limits<long>::min());
  
  const Timestamp Timestamp::last(numeric_limits<long>::max(),
				  numeric_limits<long>::max());
  
  
  Timestamp::
  Timestamp()
  {
    m_stamp.tv_sec = 0;
    m_stamp.tv_nsec = 0;
  }
  
  
  Timestamp::
  Timestamp(long seconds, long nanoseconds)
  {
    m_stamp.tv_sec = seconds;
    m_stamp.tv_nsec = nanoseconds;
  }


  Timestamp::
  Timestamp(const timespec_t & stamp)
    : m_stamp(stamp)
  {
  }
  
  
  Timestamp & Timestamp::
  operator = (const timespec_t & original)
  {
    m_stamp = original;
    return * this;
  }
  
  
  ostream & operator << (ostream & os,
			 const Timestamp & t)
  {
    os << t.m_stamp.tv_sec << ".";
    const char oldfill(os.fill('0'));
    const int oldwidth(os.width(9));
    os << t.m_stamp.tv_nsec;
    os.width(oldwidth);
    os.fill(oldfill);
    return os;
  }
  
  
  bool operator < (const Timestamp & left, const Timestamp & right)
  {
    return
      (   left.m_stamp.tv_sec  <  right.m_stamp.tv_sec ) ||
      ( ( left.m_stamp.tv_sec  == right.m_stamp.tv_sec ) &&
	( left.m_stamp.tv_nsec <  right.m_stamp.tv_nsec )   );
  }
  
  
  bool operator <= (const Timestamp & left, const Timestamp & right)
  {
    return ! (left > right);
  }
  
  
  bool operator > (const Timestamp & left, const Timestamp & right)
  {
    return
      (   left.m_stamp.tv_sec  >  right.m_stamp.tv_sec ) ||
      ( ( left.m_stamp.tv_sec  == right.m_stamp.tv_sec ) &&
	( left.m_stamp.tv_nsec >  right.m_stamp.tv_nsec )   );
  }
  
  
  bool operator >= (const Timestamp & left, const Timestamp & right)
  {
    return ! (left < right);
  }
  
  
  bool operator == (const Timestamp & left, const Timestamp & right)
  {
    return
      ( left.m_stamp.tv_sec  == right.m_stamp.tv_sec ) &&
      ( left.m_stamp.tv_nsec == right.m_stamp.tv_nsec );
  }
  
  
  Timestamp & Timestamp::
  operator -= (const Timestamp & other)
  {
    if(other.m_stamp.tv_nsec > m_stamp.tv_nsec){
      --m_stamp.tv_sec;
      m_stamp.tv_nsec += 1000000000;
    }
    m_stamp.tv_sec  -= other.m_stamp.tv_sec;
    m_stamp.tv_nsec -= other.m_stamp.tv_nsec;
    return * this;
  }
  
  
  Timestamp & Timestamp::
  operator += (const Timestamp & other)
  {
    m_stamp.tv_sec  += other.m_stamp.tv_sec;
    m_stamp.tv_nsec += other.m_stamp.tv_nsec;
    if (m_stamp.tv_nsec > 1000000000) {
      m_stamp.tv_nsec -= 1000000000;
      ++m_stamp.tv_sec;
    }
    return * this;
  }
  
  
  double Timestamp::
  ToSeconds() const
  {
    return 1.0 * m_stamp.tv_sec + 1.0e-9 * m_stamp.tv_nsec;
  }
  
  
  void Timestamp::
  FromSeconds(double sec)
  {
    m_stamp.tv_sec = static_cast<time_t>(floor(sec));
    m_stamp.tv_nsec = static_cast<long>(floor(1.0e9 * (sec - m_stamp.tv_sec)));
  }
  
  
  Timestamp Timestamp::
  Now(struct timezone * tz)
  {
    Timestamp now;
    struct timeval tv;
    if (0 != gettimeofday(&tv, tz))
      return now;
    now.m_stamp.tv_sec = tv.tv_sec;
    now.m_stamp.tv_nsec = tv.tv_usec * 1000;
    return now;
  }
  
}


sfl::Timestamp operator - (sfl::Timestamp const & lhs, sfl::Timestamp const & rhs)
{
  sfl::Timestamp result(lhs);
  result -= rhs;
  return result;
}


sfl::Timestamp operator + (sfl::Timestamp const & lhs, sfl::Timestamp const & rhs)
{
  sfl::Timestamp result(lhs);
  result += rhs;
  return result;
}

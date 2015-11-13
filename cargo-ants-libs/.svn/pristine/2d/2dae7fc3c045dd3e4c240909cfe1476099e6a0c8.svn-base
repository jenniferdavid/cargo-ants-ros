/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2009 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#include <sfl/api/Timestamp.hpp>
#include <iostream>

using namespace sfl;
using namespace std;

int main(int argc, char ** argv)
{
  Timestamp const t0(17, 78764);
  Timestamp const t1(17, 78798);
  Timestamp const t2(18, 78798);
  Timestamp const t3(18, 98798);
  Timestamp const alsot3(18, 98798);
  
  if (t0 == t1) cout << "FAILED t0 == t1\n";
  if ( ! (t3 == alsot3)) cout << "FAILED t3 == alsot3\n";
  
  if (t0 > t1) cout << "FAILED t0 > t1\n";
  if (t1 > t2) cout << "FAILED t1 > t2\n";
  if (t2 > t3) cout << "FAILED t2 > t3\n";
  
  if (t0 >= t1) cout << "FAILED t0 >= t1\n";
  if (t1 >= t2) cout << "FAILED t1 >= t2\n";
  if (t2 >= t3) cout << "FAILED t2 >= t3\n";
  
  if (t1 < t0) cout << "FAILED t1 < t0\n";
  if (t2 < t1) cout << "FAILED t2 < t1\n";
  if (t3 < t2) cout << "FAILED t3 < t2\n";
  
  if (t1 <= t0) cout << "FAILED t1 <= t0\n";
  if (t2 <= t1) cout << "FAILED t2 <= t1\n";
  if (t3 <= t2) cout << "FAILED t3 <= t2\n";
	
	Timestamp const tmili(Timestamp::FromMilliseconds(123456789));
	cout << "check tmili " << tmili << "\n";
	if (tmili.Get().tv_sec != 123456) cout << "FAILED sec == 123456\n";
	if (tmili.Get().tv_nsec != 789000000) cout << "FAILED nsec == 789000000\n";
}

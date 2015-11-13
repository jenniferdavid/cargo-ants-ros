/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#include "strutil.hpp"
using namespace std;

namespace sfl {
  
  template<>
  std::string to_string<bool>(const bool & flag) {
    if (flag)
      return "true";
    return "false";      
  }
  
  
  template<>
  bool string_to<bool>(const std::string & str, bool & foo) {
    if((str == "true") || (str == "TRUE") || (str == "True")
       || (str == "on") || (str == "ON") || (str == "On")){
      foo = true;
      return true;
    }
    else if((str == "false") || (str == "FALSE") || (str == "False")
	    || (str == "off") || (str == "OFF") || (str == "Off")){
      foo = false;
      return true;
    }
    return false;
  }
  
  
  bool splitstring(string const & input, char separator,
		   string & head, string & tail)
  {
    if (input.empty()) {
      head = "";
      tail = "";
      return false;
    }
    string::size_type col(input.find(separator, 0));
    head = input.substr(0, col);
    if (string::npos != col)
      tail = input.substr(col + 1);
    else
      tail = "";
    return true;
  }
  
}

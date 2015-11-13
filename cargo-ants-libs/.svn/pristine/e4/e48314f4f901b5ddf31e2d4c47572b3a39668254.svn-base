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


#ifndef SFL_STRUTIL_HPP
#define SFL_STRUTIL_HPP


#include <string>
#include <sstream>


namespace sfl {
  
  /** convert "anything" to a string by using its output operator */
  template<typename Foo>
  std::string to_string(const Foo & foo) {
    std::ostringstream os;
    os << foo;
    return os.str();
  }
  
  /** booleans are better represented by "true" and "false" than by 1 and 0. */
  template<>
  std::string to_string<bool>(const bool & flag);
  
  /** convert a string to "something" based on its input operator */
  template<typename Foo>
  bool string_to(const std::string & str, Foo & foo) {
    Foo bar;
    std::istringstream is(str);
    if( ! (is >> bar))
      return false;
    foo = bar;
    return true;
  }
  
  /** booleans are converted by string matching: "true", "True",
      "TRUE", "on", "On", or "ON" yield a true boolean, whereas
      "false", "False", "FALSE", "off", "Off", or "OFF" yield a false
      boolean. Anything else results in a failure (and foo is not
      touched). */
  template<>
  bool string_to<bool>(const std::string & str, bool & foo);
  
  /** very useful for booleans that are encoded as char, int, short,
      ... sets them to 1 if string_to<bool> yields true, or to 0 if
      string_to<bool> yields false, but doesn't touch foo if
      string_to<bool> failed. */
  template<typename Foo>
  bool string_to_bool(const std::string & str, Foo & foo) {
    bool bar;
    if( ! string_to(str, bar))
      return false;
    foo = bar ? 1 : 0;
    return true;
  }

  
  /**
     Split a string at the first occurrence of a separator, and store
     the two portions in head and tail. For example,
     "head:tail_or:whatever::comes:then" with separator ':' will yield
     "head" and "tail_or:whatever::comes:then". The same string split
     along '_' would yield "head:tail" and "or:whatever::comes:then".
     
     Note that it's OK to pass the same instance as input and tail,
     but DO NOT pass the head twice.
     
     \code
     for (int ii(1); ii < argc; ++ii) {
       string head;
       string tail(argv[ii]);
       while (splitstring(tail, ':', head, tail))
         cout << head << "\n";
       cout << head << "\n";
     }
     \endcode
     
     \return true if there is more to be extracted, allowing you to
     easily tokenize a string. But see also tokenize() which does just
     that.
  */
  bool splitstring(std::string const & input, char separator,
		   std::string & head, std::string & tail);
  
  
  /**
     For any tokenlist_t that accepts push_back(string const &) and
     can return its size().
  */
  template<typename tokenlist_t>
  size_t tokenize(std::string const & input, char separator, tokenlist_t & output) {
    std::string head;
    std::string tail(input);
    while (splitstring(tail, separator, head, tail))
      output.push_back(head);
    output.push_back(head);
    return output.size();
  }
  
  
  /**
     For any tokenlist_t whose operator[]() returns a string const &
     and which can return its size().
  */
  template<typename tokenlist_t, typename value_t>
  bool token_to(tokenlist_t const & tokenlist, size_t index, value_t & value) {
    if (index >= tokenlist.size())
      return false;
    return string_to(tokenlist[index], value);
  }
  
}

#endif // SFL_STRUTIL_HPP

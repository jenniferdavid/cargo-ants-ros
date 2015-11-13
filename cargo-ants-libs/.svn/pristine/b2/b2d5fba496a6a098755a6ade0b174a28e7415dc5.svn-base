/* 
 * Copyright (C) 2005
 * Centre National de Recherche Scientifique, France.
 * All rights reserved.
 * 
 * Developed at
 * Laboratoire d'Automatique et d'Analyse des Systemes, LAAS-CNRS.
 * Visit our homepage at http://www.laas.fr/
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


#ifndef SFL_OPTION_DICTIONARY_HPP
#define SFL_OPTION_DICTIONARY_HPP


#include <string>
#include <map>
#include <stdexcept>


namespace sfl {
  
  class OptionDictionary
  {
  public:
    /** \note returns empty string if undefined name */
    std::string GetOption(const std::string & key) const;
    
    /** \note overrides already existing values */
    void SetOption(const std::string & key, const std::string & value);
    
    void ReadFlatText(std::istream & is) throw(std::runtime_error);
    void ReadFlatFile(std::string const & filename) throw(std::runtime_error);
    
    void WriteFlatText(std::string const & prefix, std::ostream & os) throw(std::runtime_error);
    void WriteFlatFile(std::string const & prefix, std::string const & filename, bool append)
      throw(std::runtime_error);
    
  protected:
    typedef std::map<std::string, std::string> option_t;
    
    option_t m_option;
  };

}

#endif // SFL_OPTION_DICTIONARY_HPP

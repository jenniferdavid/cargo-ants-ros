/*
 * fpplib - Factory and Parameter Parsing Library
 *
 * Copyright (c) 2011 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FPPLIB_REGISTRY_HPP
#define FPPLIB_REGISTRY_HPP

#include <string>
#include <map>
#include <vector>


namespace fpplib {
  
  using std::string;
  using std::map;
  using std::vector;


  /**
     Maintains a dictionary of (single-entry) instances, and at the
     same time a vector of instances.  Multiply defined names end up
     being single entries in the dictionary (newer additions kick out
     older entries), whereas the vector remembers all of them.
     Instances with empty names are ignored (stored neither in the
     dictionary nor the vector).
     
     By default, the registry owns the instances passed to it (except
     the ones it ignored because they had an empty name).
  */
  template<typename value_type, bool owns_registered_instances = true>
  class Registry
  {
  public:
    typedef value_type * pointer_type;
    
    typedef map<string, pointer_type> map_t;
    map_t map_;
    
    typedef vector<pointer_type> vector_t;
    vector_t vector_;
    
    
    virtual ~Registry()
    {
      if (owns_registered_instances) {
	for (size_t ii (0); ii < vector_.size(); ++ii) {
	  delete vector_[ii];
	}
      }
    }
    
    void add(string const & instance_name,
	     pointer_type pointer)
    {
      if ( ! instance_name.empty()) {
	map_[instance_name] = pointer;
	vector_.push_back(pointer);
      }
    }
    
    /** Removes an entry, and deletes it in case this registry owns
	its instances (which is the default behavior). The pointer is
	needed to ensure that any instance with the given name
	actually matches the one you want to remove.  Pass a null
	pointer if you want to skip this check.
    */
    void remove(string const & instance_name,
		pointer_type pointer)
    {
      typename map_t::iterator im(map_.find(instance_name));
      if (map_.end() == im) {
	return;			// not found
      }
      if ((0 != pointer) && (pointer != im->second)) {
	return;			// mismatching instance
      }
      map_.erase (im);
      for (typename vector_t::iterator iv(vector_.begin()); iv != vector_.end(); ++iv) {
	if (*iv == pointer) {
	  vector_.erase (iv);
	  break;
	}
      }
      if (owns_registered_instances) {
	delete pointer;
      }
    }
    
    pointer_type find(string const & instance_name) const
    {
      typename map_t::const_iterator ii(map_.find(instance_name));
      if (map_.end() != ii) {
	return ii->second;
      }
      return 0;
    }
    
    inline size_t size() const
    {
      return vector_.size();
    }
    
    inline pointer_type at(size_t index) const
    {
      return vector_[index];
    }
  };
  
}

#endif // FPPLIB_REGISTRY_HPP

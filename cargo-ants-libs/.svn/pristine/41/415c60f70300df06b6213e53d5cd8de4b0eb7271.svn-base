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


#ifndef SUNFLOWER_OBJECTIVE_HPP
#define SUNFLOWER_OBJECTIVE_HPP


#include <sfl/util/array2d.hpp>
#include <boost/shared_ptr.hpp>
#include <iosfwd>


namespace sfl {
  
  
  class DynamicWindow;
  class Scan;
  
  
  class Objective
  {
  public:
    Objective(const DynamicWindow & dynamic_window);
    
    /** \note empty default implementation */
    virtual ~Objective() {}
    
    virtual void Initialize(std::ostream * progress_stream) = 0;
    
    /** If true, then this objective can inform the DWA about
	admissible speeds. Such objectives are updated first, then used
	to update the cached information about which speeds are
	admissible in DynamicWindow. */
    virtual bool YieldsAdmissible() const = 0;
    
    /** This method only gets called if YieldsAdmissible() returns
	true. It is used by DynamicWindow::Update() to inform other
	objectives about the set of admissible velocities. */
    virtual bool Admissible(int qdlIndex, int qdrIndex) const = 0;
    
    /** If true, then this objective subclass updates the entire
	velocity space, regardless of the (qdlMin, qdlMax, qdrMin,
	qdrMax) that get passed to Calculate(). Mostly useful just for
	graphical output though. */
    virtual bool UsesEntireVelocitySpace() const { return false; }
    
    /** The method that updates the objective values. If
	YieldsAdmissible() returns true, then DynamicWindow calls this
	method early in its update cycle, and then uses the objective
	values to initialize the set of admissible velocities. Else,
	this method gets called later in the DWA update cycle.
	
	\note If YieldsAdmissible() returns true in this objective
	subclass, then Calculate() should (obviously?) not rely on
	DynamicWindow::Admissible(), as it will not yet have been
	updated.
    */
    virtual void Calculate(double timestep, size_t qdlMin, size_t qdlMax,
			   size_t qdrMin, size_t qdrMax,
			   double carrot_lx, double carrot_ly,
			   boost::shared_ptr<const Scan> local_scan) = 0;
    
    /** \pre all indices must be valid. */
    void Rescale(size_t qdlMin, size_t qdlMax,
		 size_t qdrMin, size_t qdrMax);
    
    /** \pre all indices must be valid. */
    double Value(size_t qdlIndex, size_t qdrIndex) const
    { return m_value[qdlIndex][qdrIndex]; }
    
    /** \pre all indices must be valid. */
    double Min(size_t qdlMin, size_t qdlMax,
	       size_t qdrMin, size_t qdrMax) const;

    /** \pre all indices must be valid. */
    double Max(size_t qdlMin, size_t qdlMax,
	       size_t qdrMin, size_t qdrMax) const;

    static const double minValue;// = 0;
    static const double maxValue;// = 1;
    const size_t dimension;
    
  protected:
    const DynamicWindow & m_dynamic_window;
    array2d<double> m_value;
  };
  
}

#endif // SUNFLOWER_OBJECTIVE_HPP

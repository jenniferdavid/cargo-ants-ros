/* -*- mode: C++; tab-width: 2 -*- */
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


#ifndef SUNFLOWER_GRIDFRAME_HPP
#define SUNFLOWER_GRIDFRAME_HPP


#include <sfl/util/Frame.hpp>
#include <sfl/util/array2d.hpp>
#include <iosfwd>


namespace sfl {
  
  
  class GridFrame
    : public Frame
  {
  public:
    typedef vec2d<double> position_t;
		
		class index_t: public vec2d<ssize_t> {
		public:
			typedef vec2d<ssize_t> base_t;
			
			index_t(): base_t() {}
			index_t(ssize_t v0, ssize_t v1): base_t(v0, v1) {}
			index_t(const index_t & orig): base_t(orig) {}
			
			const bool operator < (const base_t & rhs) const
			{ return (v0 < rhs.v0) || ((v0 == rhs.v0) && (v1 < rhs.v1)); }
			
			const index_t & operator = (const base_t & rhs)
			{ v0 = rhs.v0; v1 = rhs.v1; return * this; }
		};
    
		
    struct draw_callback {
      virtual ~draw_callback() {}
			/** \note defaults to no operation */
      virtual void operator () (ssize_t ix, ssize_t iy) = 0;
    };
		
		/** Debug utility that simply prints out the (ix, iy) passed to
				it, with one leading whitespace. */
		struct dbg_draw_callback: public draw_callback {
			explicit dbg_draw_callback(std::ostream & os);
      virtual void operator () (ssize_t ix, ssize_t iy);
			std::ostream & m_os;
		};
		
		/** Wrap an existing draw_callback, making sure it never gets
				indices that are out of range. */
		struct rangecheck_draw_callback: public draw_callback {
			rangecheck_draw_callback(draw_callback & cb,
															 ssize_t xbegin, ssize_t xend,
															 ssize_t ybegin, ssize_t yend);
			
			/** Checks xbegin<=ix<xend and ybegin<=iy<yend and forwards the
					call to the registered callback. The count is updated if the
					range check passed. */
			virtual void operator () (ssize_t ix, ssize_t iy);
			
			draw_callback & cb;
			ssize_t const xbegin;
			ssize_t const xend;
			ssize_t const ybegin;
			ssize_t const yend;
			size_t count;
		};
    
    
    explicit GridFrame(double delta);
    GridFrame(double x, double y, double theta, double delta);
    GridFrame(const GridFrame & orig);
    GridFrame(const Frame & frame, double delta);
		
		GridFrame & operator = (GridFrame const & rhs);
    
    void Configure(double position_x, double position_y, double position_theta,
		   double delta);
    
		/**
			 Compute the index of a point given in global coordinates (px, py).
		*/
    index_t GlobalIndex(double px, double py) const;
		
		/**
			 Compute the index of a position given in global coordinates.
		*/
    index_t GlobalIndex(position_t point) const;
		
		/**
			 Compute the index of a point given in local coordinates (px,
			 py).  "Local" means that the coordinates are relative to the
			 origin of this grid frame.
		*/
    index_t LocalIndex(double px, double py) const;
		
		/**
			 Compute the index of a position given in local coordinates.
			 "Local" means that the coordinates are relative to the origin
			 of this grid frame.
		*/
    index_t LocalIndex(position_t point) const;
    
		/**
			 Compute the global coordinates of the center of the cell at index (ix, iy).
		*/
    position_t GlobalPoint(ssize_t ix, ssize_t iy) const;
		
		/**
			 Compute the global coordinates of the center of the cell at the given index.
		*/
    position_t GlobalPoint(index_t index) const;
		
		/**
			 Compute the local coordinates of the center of the cell at index (ix, iy).
		*/
    position_t LocalPoint(ssize_t ix, ssize_t iy) const;
		
		/**
			 Compute the local coordinates of the center of the cell at the given index.
		*/
    position_t LocalPoint(index_t index) const;
    
		/**
			 Draws a purely grid-index based line using the Differential
			 Analyzer Algorithm. The range of valid (ix, iy) is
			 <code>xbegin <= ix < xend</code> and <code>ybegin <= iy
			 < yend</code>.
       
			 \return The number of grid cells drawn.
		*/
    static size_t DrawDDALine(ssize_t ix0, ssize_t iy0,
															ssize_t ix1, ssize_t iy1,
															ssize_t xbegin, ssize_t xend,
															ssize_t ybegin, ssize_t yend,
															draw_callback & cb);
		
		/** "without" limit checks */
    static size_t DrawDDALine(ssize_t ix0, ssize_t iy0,
															ssize_t ix1, ssize_t iy1,
															draw_callback & cb);
		
    /**
       Calls the provided callback functor for each index that lies on
       the line.
       
       \todo Simply calls DrawDDALine() but could be smarter.
       
       \return The number of grid cells drawn.
    */
    size_t DrawLocalLine(double x0, double y0, double x1, double y1,
												 ssize_t xbegin, ssize_t xend,
												 ssize_t ybegin, ssize_t yend,
												 draw_callback & cb) const;
    
    /**
       Same as DrawLocalLine() but first transforms the given
       endpoints from the global to the local frame of reference.
    */
    size_t DrawGlobalLine(double x0, double y0, double x1, double y1,
													ssize_t xbegin, ssize_t xend,
													ssize_t ybegin, ssize_t yend,
													draw_callback & cb) const;

    /**
			 \note If you need range checks, use rangecheck_draw_callback to
			 wrap your callback.
		*/
		static void DrawMidpointCircle(ssize_t icx, ssize_t icy, size_t irad,
																	 draw_callback & cb);
		
    /**
			 \note If you need range checks, use rangecheck_draw_callback to
			 wrap your callback.
		*/
		void DrawLocalCircle(double cx, double cy, double rad,
												 draw_callback & cb) const;
		
    /**
			 \note If you need range checks, use rangecheck_draw_callback to
			 wrap your callback.
		*/
		void DrawGlobalCircle(double cx, double cy, double rad,
													draw_callback & cb) const;
		
    double Delta() const { return m_delta; }
    
  protected:
    double m_delta;
    double m_delta_inv;
  };
  
}

#endif // SUNFLOWER_GRIDFRAME_HPP

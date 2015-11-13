/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#ifndef SFL_MAPPER2D_HPP
#define SFL_MAPPER2D_HPP


#include <sfl/gplan/RWTravmap.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <map>
#include <set>


namespace sfl {
	
	
	class Scan;
	class GridFrame;
	
	
	struct travmap_cost_decay {
		virtual ~travmap_cost_decay() {}
		virtual double operator () (double normdist) const = 0;
	};
	
	struct linear_travmap_cost_decay: public travmap_cost_decay {
		/** \return (1-normdist) */
		virtual double operator () (double normdist) const;
	};
	
	struct exponential_travmap_cost_decay: public travmap_cost_decay {
		double const power;
		explicit exponential_travmap_cost_decay(double power);
		/** \return pow(1-normdist, power) */
		virtual double operator () (double normdist) const;
	};
	
	
  class Mapper2d
	{
	public:
		struct travmap_grow_strategy {
			virtual ~travmap_grow_strategy() {}
			
			/**
				 Determine if and how the travmap might need to be grown to
				 accomodate the point (ix, iy), and potentially resize it
				 accordingly.
				 
				 \return true if the travmap was grown.
			*/
			virtual bool operator () (TraversabilityMap & travmap,
																ssize_t ix, ssize_t iy) = 0;
		};

		struct never_grow: public travmap_grow_strategy {
			virtual bool operator () (TraversabilityMap & travmap,
																ssize_t ix, ssize_t iy)  { return false; }
		};

		struct always_grow: public travmap_grow_strategy {
			virtual bool operator () (TraversabilityMap & travmap,
																ssize_t ix, ssize_t iy);
		};
		
		
	private:
		Mapper2d();
		Mapper2d(const Mapper2d &);
		
	protected:
		Mapper2d(double robot_radius,
						 double buffer_zone,
						 double padding_factor,
						 /** use linear_travmap_cost_decay for legacy behavior */
						 boost::shared_ptr<travmap_cost_decay const> decay,
						 boost::shared_ptr<TraversabilityMap> travmap,
						 boost::shared_ptr<travmap_grow_strategy> grow_strategy);
		
  public:
		typedef GridFrame::index_t index_t;
		typedef std::set<index_t> index_buffer_t;
		typedef TraversabilityMap::draw_callback draw_callback;
		
		
		Mapper2d(const GridFrame & gridframe,
						 ssize_t grid_xbegin,
						 ssize_t grid_xend,
						 ssize_t grid_ybegin,
						 ssize_t grid_yend,
						 double robot_radius,
						 double buffer_zone,
						 double padding_factor,
						 int freespace,
						 int obstacle,
						 /** use linear_travmap_cost_decay for legacy behavior */
						 boost::shared_ptr<travmap_cost_decay const> decay,
						 const std::string & name,
						 /** Optional. Defaults to never_grow. */
						 boost::shared_ptr<travmap_grow_strategy> grow_strategy);

		virtual ~Mapper2d() {}
		
		/**
			 Create a Mapper2d from a traversability map file.
		*/
		static boost::shared_ptr<Mapper2d>
		Create(double robot_radius,
					 double buffer_zone,
					 double padding_factor,
					 /** use linear_travmap_cost_decay for legacy behavior */
					 boost::shared_ptr<travmap_cost_decay const> decay,
					 const std::string & traversability_file,
					 /** Optional. Defaults to never_grow. */
					 boost::shared_ptr<travmap_grow_strategy> grow_strategy,
					 std::ostream * err_os);
		
		
		/**
			 Convenience method for creating an automatically growing map
			 with exponential cost decay. Uses a padding_factor of 2,
			 because that's the value I saw being used in legacy code, which
			 marked it as coming from legacy code. Double legacy, wonderful.
		*/
		static boost::shared_ptr <Mapper2d>
		Create (GridFrame const & gframe,
						double robot_radius,
						double buffer_zone,
						double decay_power);
		
		
		/**
			 Update of traversability map based on a Scan instance, where
			 each scan point is considered an obstacle. We only call
			 TraversabilityMap::SetValue() for actually changed cells, so
			 the draw_callback is only called for those cells that actually
			 get modified. The travmap_grow_strategy registered with the
			 Mapper2d determines if and how the TraversabilityMap gets
			 resized.
			 
			 \return The number of cells that got changed.
		*/
		size_t Update(const Frame & pose, const Scan & scan,
									draw_callback * cb = 0);
		
		/**
			 Similar to Update(const Frame&, const Scan&, draw_callback *),
			 but based on arrays of local (x, y) obstacle point coordinates.
			 
			 \return The number of cells that got changed.
		*/
		size_t Update(const Frame & pose,
									size_t length, double * locx, double * locy,
									draw_callback * cb = 0);

		/**
			 Draw a (non-filled) circle of obstacle points using
			 GridFrame::DrawGlobalCircle(). Each grid cell on the circle is
			 considered a workspace-obstacle, and will be grown by the robot
			 radius and buffer zone.
			 
			 \return The number of cells that got changed.
		*/
		size_t AddObstacleCircle(double globx, double globy, double radius,
														 /** gets passed to AddOneObstacle() */
														 bool force,
														 /** gets passed to AddOneObstacle() */
														 draw_callback * cb = 0);
		
		/**
			 Draw a line of obstacle points from (gx0, gy0) to (gx1, gy1)
			 using GridFrame::DrawGlobalLine(). Each grid cell on the line
			 is considered a workspace-obstacle, and will be grown by the
			 robot radius and buffer zone.
			 
			 \return The number of cells that got changed.
		*/
		size_t AddObstacleLine(double gx0, double gy0,
													 double gx1, double gy1,
													 /** gets passed to AddOneObstacle() */
													 bool force,
													 /** gets passed to AddOneObstacle() */
													 draw_callback * cb = 0);
		
		boost::shared_ptr<RDTravmap> CreateRDTravmap() const;
		boost::shared_ptr<WRTravmap> CreateWRTravmap();
		
		const GridFrame & GetGridFrame() const { return gridframe; }
		
		
		const GridFrame gridframe;
		const double buffer_zone;
		const double grown_safe_distance;
		const double grown_robot_radius;
		
		
		/**
			 In case you want to draw buffered obstacle lines into the map,
			 call GridFrame::DrawGlobalLine() (or DrawLocalLine() or
			 DrawDDALine()) with an instance of buffered_obstacle_adder.
		*/
		class buffered_obstacle_adder
			: public GridFrame::draw_callback
		{
		public:
			/**
				 The extra Mapper2d::draw_callback constructor argument can be
				 used to pass through another drawer, which will get called
				 for each individual cell in the buffered obstacle region.
			*/
			buffered_obstacle_adder(Mapper2d * _m2d,
															/** gets passed to Mapper2d::AddOneObstacle() */
															bool force,
															/** gets passed to Mapper2d::AddOneObstacle() */
															Mapper2d::draw_callback * _cb);
			virtual void operator () (ssize_t ix, ssize_t iy);
			Mapper2d * m2d;
			bool force;
			Mapper2d::draw_callback * cb;
			size_t count;
		};
		
		/** \return The number of cells that were changed. */
		size_t AddOneGlobalObstacle(/** Global X-coordinate of the new
																		obstacle. */
																double globx,
																/** Global Y-coordinate of the new
																		obstacle. */
																double globy,
																/** If force==true then always apply
																		the obstacle buffer mask, even if
																		the travmap says there already is
																		a W-space obstacle here. This is
																		used from within
																		RemoveOneObstacle() and
																		UpdateObstacles(). */
																bool force,
																/** If non-null, this gets informed of cost
																		changes. */
																draw_callback * cb);
		
		/** \return The number of cells that were changed. See also
				AddOneGlobalObstacle(double, double, bool, draw_callback *). */
		size_t AddOneObstacle(index_t source_index, bool force, draw_callback * cb);
		
		/**
			 Similar to Update(const Frame&, const Scan&, draw_callback*),
			 but also updates the cells along the rays of the scan, up to
			 max_remove_distance.
			 
			 \return The number of cells that got changed.
		*/
		size_t SwipedUpdate(const Frame & pose,
												const Multiscanner::raw_scan_collection_t & scans,
												double max_remove_distance,
												draw_callback * cb = 0);
		
		/** \return An UPPER BOUND on the number of cells that were changed. */
		size_t RemoveOneObstacle(index_t source_index, draw_callback * cb);
		
		/**
			 Removes and adds W-space obstacles. Forces all operations, so
			 if there are many duplicates in the buffers then it gets slowed
			 down.
			 
			 \note draw_callback can be called multiple times for a given
			 index, with different costs being fed to it each time.
			 
			 \return An UPPER BOUND on the number of cells that were changed.
		*/
		size_t UpdateObstacles(/** if non-null, all cells that should become W-obstacles */
													 index_buffer_t const * add,
													 bool force_add,
													 /** if non-null, all W-obstacles that
															 should become freespace (non-const
															 pointer because spurious entries get
															 pruned from it) */
													 index_buffer_t * remove,
													 draw_callback * cb);
		
		/** Maintain the map size, but set all cells to freespace. */
		void ClearAllObstacles(draw_callback * cb);
		
		int ComputeCost(double dist_from_obstacle) const;
		
		index_buffer_t const & GetFreespaceBuffer() const { return m_freespace_buffer; }
		index_buffer_t const & GetObstacleBuffer() const { return m_obstacle_buffer; }
		index_buffer_t const & GetSwipeCheckBuffer() const { return m_swipe_check_buffer; }
		index_buffer_t const & GetSwipeHoleBuffer() const { return m_swipe_hole_buffer; }
		index_buffer_t const & GetSwipeRepairBuffer() const { return m_swipe_repair_buffer; }
		
		/** Use at your own risk: gives access to the underlying
				travmap. */
		boost::shared_ptr<TraversabilityMap> GetTravmap();
		boost::shared_ptr<TraversabilityMap const> GetTravmap() const;
		
	protected:
    typedef std::map<index_t, int> addmask_t; // cell-to-cost map
		typedef std::map<index_t, bool> removemask_t;	// true: wipe and re-add, false: re-add only
		
		void InitAddmask();
		void InitRemovemask();
		
		boost::shared_ptr<TraversabilityMap> m_travmap;
		addmask_t m_addmask;
		ssize_t m_addmask_x0, m_addmask_y0, m_addmask_x1, m_addmask_y1;	// bbox
		
		boost::shared_ptr<travmap_grow_strategy> m_grow_strategy;
		boost::shared_ptr<travmap_cost_decay const> m_cost_decay;

		removemask_t m_removemask;
		
		// currently unused... could be useful for speedups later
		ssize_t m_removemask_x0, m_removemask_y0, m_removemask_x1, m_removemask_y1;	// bbox
		
		index_buffer_t m_freespace_buffer;
		index_buffer_t m_obstacle_buffer;
		index_buffer_t m_swipe_check_buffer;
		index_buffer_t m_swipe_hole_buffer;
		index_buffer_t m_swipe_repair_buffer;
	};

}

#endif // SFL_MAPPER2D_HPP

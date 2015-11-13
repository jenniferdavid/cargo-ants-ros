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


#include "Mapper2d.hpp"
#include "GridFrame.hpp"
#include <sfl/util/pdebug.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/api/Scan.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>


using namespace std;
using namespace boost;


//static const double sqrt_of_two(1.41421356237);
static const double sqrt_of_half(0.707106781187);

namespace local {

	struct swipe_cb
		: public sfl::GridFrame::draw_callback
	{
		sfl::Mapper2d::index_buffer_t & swipe_buffer;
		sfl::Mapper2d::index_buffer_t & freespace_buffer;
		const sfl::TraversabilityMap & travmap;
		
		swipe_cb(sfl::Mapper2d::index_buffer_t & _swipe_buffer,
						 sfl::Mapper2d::index_buffer_t & _freespace_buffer,
						 const sfl::TraversabilityMap & _travmap)
			: swipe_buffer(_swipe_buffer), freespace_buffer(_freespace_buffer), travmap(_travmap)
		{
		}
		
		virtual void operator () (ssize_t ix, ssize_t iy)
		{
			sfl::Mapper2d::index_t const ii(ix, iy);
			swipe_buffer.insert(ii);
			if (travmap.IsWObst(ix, iy)) {
				PVDEBUG("swipe_cb on W-obst %zd %zd\n", ix, iy);
				freespace_buffer.insert(ii);
			}
		}
		
	};
	
}

using namespace local;


namespace sfl {
	
	
	Mapper2d::
	Mapper2d(const GridFrame & _gridframe,
					 ssize_t grid_xbegin,
					 ssize_t grid_xend,
					 ssize_t grid_ybegin,
					 ssize_t grid_yend,
					 double robot_radius,
					 double _buffer_zone,
					 double padding_factor,
					 int _freespace,
					 int _obstacle,
					 shared_ptr<travmap_cost_decay const> decay,
					 const std::string & name,
					 shared_ptr<travmap_grow_strategy> grow_strategy)
		: gridframe(_gridframe),
			buffer_zone(_buffer_zone),
			grown_safe_distance(robot_radius + _buffer_zone
													+ padding_factor * _gridframe.Delta() * sqrt_of_half),
			grown_robot_radius(robot_radius
												 + padding_factor * _gridframe.Delta() * sqrt_of_half),
			m_travmap(new TraversabilityMap(_gridframe,
																			grid_xbegin, grid_xend,
																			grid_ybegin, grid_yend,
																			_freespace, _obstacle, name)),
			m_cost_decay(decay)
	{
		InitAddmask();
		InitRemovemask();
		if (grow_strategy)
			m_grow_strategy = grow_strategy;
		else
			m_grow_strategy.reset(new never_grow());			
	}
	
	
	Mapper2d::
	Mapper2d(double robot_radius,
					 double _buffer_zone,
					 double padding_factor,
					 shared_ptr<travmap_cost_decay const> decay,
					 shared_ptr<TraversabilityMap> travmap,
					 shared_ptr<travmap_grow_strategy> grow_strategy)
		: gridframe(travmap->gframe),
			buffer_zone(_buffer_zone),
			grown_safe_distance(robot_radius + _buffer_zone
													+ padding_factor * gridframe.Delta() * sqrt_of_half),
			grown_robot_radius(robot_radius
												 + padding_factor * gridframe.Delta() * sqrt_of_half),
			m_travmap(travmap),
			m_cost_decay(decay)
	{
		InitAddmask();
		InitRemovemask();
		if (grow_strategy)
			m_grow_strategy = grow_strategy;
		else
			m_grow_strategy.reset(new never_grow());			
	}
	
	
	shared_ptr<Mapper2d> Mapper2d::
	Create(double robot_radius,
				 double buffer_zone,
				 double padding_factor,
				 shared_ptr<travmap_cost_decay const> decay,
				 const std::string & traversability_file,
				 boost::shared_ptr<travmap_grow_strategy> grow_strategy,
				 std::ostream * err_os)
	{
		ifstream trav(traversability_file.c_str());
    if( ! trav){
      if(err_os) *err_os << "ERROR in Mapper2d::Create():\n"
												 << "  invalid traversability file \""
												 << traversability_file << "\".\n";
      return shared_ptr<Mapper2d>();
    }
    shared_ptr<TraversabilityMap>
      traversability(TraversabilityMap::Parse(trav, err_os));
    if( ! traversability){
      if(err_os) *err_os << "ERROR in Mapper2d::Create():\n"
												 << "  TraversabilityMap::Parse() failed on \""
												 << traversability_file << "\".\n";
      return shared_ptr<Mapper2d>();
    }
		
		shared_ptr<Mapper2d>
			result(new Mapper2d(robot_radius, buffer_zone, padding_factor, decay,
													traversability, grow_strategy));
		return result;
	}
	
	
	shared_ptr <Mapper2d> Mapper2d::
	Create (GridFrame const & gframe,
					double robot_radius, double buffer_zone, double decay_power)
	{
		shared_ptr <travmap_cost_decay> cost_decay (new exponential_travmap_cost_decay (decay_power));
		shared_ptr <travmap_grow_strategy> grow_strategy (new always_grow());
    shared_ptr <TraversabilityMap> travmap (new TraversabilityMap (gframe, 0, 0, 0, 0));
		static double const padding_factor (2.0);
		shared_ptr <Mapper2d> m2d (new Mapper2d (robot_radius, buffer_zone, padding_factor,
																						 cost_decay, travmap, grow_strategy));
		return m2d;
	}
	
	
	size_t Mapper2d::
	Update(const Frame & pose, size_t length, double * locx, double * locy,
				 draw_callback * cb)
	{
		size_t count(0);
		for(size_t ii(0); ii < length; ++ii){
			double xw(locx[ii]);
			double yw(locy[ii]);
			pose.To(xw, yw);
			count += AddOneGlobalObstacle(xw, yw, false, cb);
		}
		return count;
	}
	
	
	size_t Mapper2d::
	Update(const Frame & pose, const Scan & scan,
				 draw_callback * cb)
	{
		const Scan::array_t & scan_data(scan.data);
		size_t count(0);		
		for(size_t ii(0); ii < scan_data.size(); ++ii)
			count +=
				AddOneGlobalObstacle(scan_data[ii].globx, scan_data[ii].globy, false, cb);
		return count;
	}
	
	
	size_t Mapper2d::
	AddOneGlobalObstacle(double globx, double globy, bool force, draw_callback * cb)
	{
		return AddOneObstacle(gridframe.GlobalIndex(globx, globy), force, cb);
	}
	
	
	size_t Mapper2d::
	AddOneObstacle(index_t source_index, bool force, draw_callback * cb)
	{
		ssize_t const ix0(source_index.v0);
		ssize_t const iy0(source_index.v1);
		
		// We can skip this one if it is already a W-space obstacle (and
		// we are allowed to).
		if (( ! force) && (m_travmap->IsWObst(ix0, iy0))) {
			PVDEBUG("source %zd %zd is already W-obst\n", ix0, iy0);
			return 0;
		}
		
		// maybe grow the grid...
		ssize_t const bbx0(m_addmask_x0 + ix0);
		ssize_t const bby0(m_addmask_y0 + iy0);
		ssize_t const bbx1(m_addmask_x1 + ix0);
		ssize_t const bby1(m_addmask_y1 + iy0);
		(*m_grow_strategy)(*m_travmap, bbx0, bby0);
		(*m_grow_strategy)(*m_travmap, bbx0, bby1);
		(*m_grow_strategy)(*m_travmap, bbx1, bby0);
		(*m_grow_strategy)(*m_travmap, bbx1, bby1);
		
		// Ready to insert W-space obstacle and perform C-space expansion
		m_travmap->SetWObst(ix0, iy0, cb);
		size_t count(1);
		PVDEBUG("added W-obst at source %zd %zd\n", ix0, iy0);
		
		// Perform C-space extension with buffer zone.
		for (addmask_t::const_iterator is(m_addmask.begin()); is != m_addmask.end(); ++is) {
			ssize_t const ix(ix0 + is->first.v0);
			ssize_t const iy(iy0 + is->first.v1);
			if ((ix0 == ix) && (iy0 == iy))
				continue;								// we've already flagged the center
			
			int old_value;
			if (m_travmap->GetValue(ix, iy, old_value) // bound check in here
					&& (is->second > old_value)) {
				m_travmap->SetValue(ix, iy, is->second, cb);
				++count;
				PVDEBUG("added C-obst at target %zd %zd, cost %d (old cost %d)\n",
								ix, iy, is->second, old_cost);
			}
		}
		
		PVDEBUG("changed %zu cells\n", count);
		return count;
	}
	
	
	size_t Mapper2d::
	RemoveOneObstacle(index_t source_index, draw_callback * cb)
	{
		index_buffer_t remove;
		remove.insert(source_index);
		return UpdateObstacles(0, false, &remove, cb);
	}
	
	
	size_t Mapper2d::
	UpdateObstacles(index_buffer_t const * add,
									bool force_add,
									index_buffer_t * remove,
									draw_callback * cb)
	{
		size_t count(0);
		m_swipe_hole_buffer.clear();
		m_swipe_repair_buffer.clear();
		
		if (remove) {
			
			// remove former W-obstacles
			index_buffer_t spurious;
			for (index_buffer_t::const_iterator is(remove->begin()); is != remove->end(); ++is) {
				ssize_t const ix0(is->v0);
				ssize_t const iy0(is->v1);
				if ( ! m_travmap->IsWObst(ix0, iy0)) {
					PDEBUG("skip source %zd %zd because not W-obst\n", ix0, iy0);
					spurious.insert(*is);
					continue;
				}
				
				if (m_travmap->IsValid(ix0, iy0)) {
					PDEBUG("wipe source %zd %zd\n", ix0, iy0);
					++count;
					m_travmap->SetFree(ix0, iy0, cb);
				}
				else
					PDEBUG("source %zd %zd is out of bounds\n", ix0, iy0);
			}
			
			// should normally be a no-op
			for (index_buffer_t::const_iterator is(spurious.begin()); is != spurious.end(); ++is)
				remove->erase(*is);
			
			// determine the areas influenced by the former W-obstacles:
			// inner areas are later wiped, outer areas only scanned for
			// already existing W-obstacles
			for (index_buffer_t::const_iterator is(remove->begin()); is != remove->end(); ++is) {
				ssize_t const ix0(is->v0);
				ssize_t const iy0(is->v1);
				for (removemask_t::const_iterator ir(m_removemask.begin());
						 ir != m_removemask.end(); ++ir) {
					ssize_t const tx(ix0 + ir->first.v0);
					ssize_t const ty(iy0 + ir->first.v1);
					if (m_travmap->IsWObst(tx, ty)) {
						PDEBUG("add again target %zd %zd because W-obst\n", tx, ty);
						m_swipe_repair_buffer.insert(index_t(tx, ty));
					}
					else if (ir->second && ( ! m_travmap->IsFree(tx, ty))) {
						PDEBUG("wipe target %zd %zd\n", tx, ty);
						m_swipe_hole_buffer.insert(index_t(tx, ty));
					}
				}
			}
			
			// wipe out the region of influence of each removed W-obstacle
			for (index_buffer_t::const_iterator iw(m_swipe_hole_buffer.begin());
					 iw != m_swipe_hole_buffer.end(); ++iw)
				m_travmap->SetFree(iw->v0, iw->v1, cb);
			count += m_swipe_hole_buffer.size();
			
			// add back all the W-obstacles which have a region of influence
			// that intersects the set of cells that were just wiped (set
			// force add to true, because we did not set these cells to
			// W-obstacle cost)
			for (index_buffer_t::const_iterator ia(m_swipe_repair_buffer.begin());
					 ia != m_swipe_repair_buffer.end(); ++ia)
				count += AddOneObstacle(*ia, true, cb);
		}
		
		// forcing optional, depends on user
		if (add)
			for (index_buffer_t::const_iterator ia(add->begin()); ia != add->end(); ++ia)
				count += AddOneObstacle(*ia, force_add, cb);
		
		return count;
	}
	
	
	/**
		 \todo Removals should be done "en vrac", not one by
		 one. Multiscanner::raw_scan_collection_t is bad because it uses
		 the robot's pose, not each sensor's at the time of capture.
	*/
	size_t Mapper2d::
	SwipedUpdate(const Frame & pose,
							 const Multiscanner::raw_scan_collection_t & scans,
							 double max_remove_distance,
							 draw_callback * cb)
	{
		m_freespace_buffer.clear();
		m_obstacle_buffer.clear();
		m_swipe_check_buffer.clear();
		swipe_cb swipe(m_swipe_check_buffer, m_freespace_buffer, *m_travmap);
		ssize_t const bbx0(m_travmap->grid.xbegin());
		ssize_t const bbx1(m_travmap->grid.xend());
		ssize_t const bby0(m_travmap->grid.ybegin());
		ssize_t const bby1(m_travmap->grid.yend());
		
		// compute sets of swiped and obstacle cells
		for (size_t is(0); is < scans.size(); ++is) {
			double const spx(scans[is]->scanner_pose.X());
			double const spy(scans[is]->scanner_pose.Y());
			double const spt(scans[is]->scanner_pose.Theta());
			Scan::array_t const & scan_data(scans[is]->data);
			const index_t i0(gridframe.GlobalIndex(spx, spy));
			
			for (size_t ir(0); ir < scan_data.size(); ++ir) {
				index_t const ihit(gridframe.GlobalIndex(scan_data[ir].globx, scan_data[ir].globy));
				index_t iswipe;
				if (scan_data[ir].rho <= max_remove_distance) {
					// swipe up to the laser point
					iswipe = ihit;
				}
				else {
					// swipe along ray, but stop after max_remove_distance
					double const theta(spt + scan_data[ir].phi);
					iswipe = gridframe.GlobalIndex(spx + max_remove_distance * cos(theta),
																				 spy + max_remove_distance * sin(theta));
				}
				gridframe.DrawDDALine(i0.v0, i0.v1, iswipe.v0, iswipe.v1,
															bbx0, bbx1, bby0, bby1,
															swipe);
				if (scan_data[ir].in_range) {
					// always insert them into m_obstacle_buffer, do not check
					// m_travmap->IsWObst(ihit.v0, ihit.v1), because otherwise
					// neighboring rays can erase known W-obstacles
					PDEBUG("in range W-obst: global %g  %g   index %zd %zd\n",
								 scan_data[ir].globx, scan_data[ir].globy,
								 ihit.v0, ihit.v1);
					m_obstacle_buffer.insert(ihit);
				}
			}
		}
		
		// remove new obstacles from swiped set (due to grid effects, it
		// would otherwise be possible for a later ray to erase an
		// obstacle seen by an earlier ray)
		for (index_buffer_t::const_iterator io(m_obstacle_buffer.begin());
				 io != m_obstacle_buffer.end(); ++io)
			m_freespace_buffer.erase(*io);
		
		return UpdateObstacles(&m_obstacle_buffer, false, &m_freespace_buffer, cb);
	}
	
	
	shared_ptr<RDTravmap> Mapper2d::
	CreateRDTravmap() const
	{
		shared_ptr<RDTravmap> rdt(new RDTravmap(m_travmap));
		return rdt;
	}
	
	
	shared_ptr<WRTravmap> Mapper2d::
	CreateWRTravmap()
	{
		shared_ptr<WRTravmap> wrt(new WRTravmap(m_travmap));
		return wrt;
	}
	
	
	Mapper2d::buffered_obstacle_adder::
	buffered_obstacle_adder(Mapper2d * _m2d, bool _force, Mapper2d::draw_callback * _cb)
		: m2d(_m2d),
			force(_force),
			cb(_cb),
			count(0)
	{
	}
	
	
	void Mapper2d::buffered_obstacle_adder::
	operator () (ssize_t ix, ssize_t iy)
	{
		count += m2d->AddOneObstacle(index_t(ix, iy), force, cb);
	}
	
	
	size_t Mapper2d::
	AddObstacleCircle(double globx, double globy, double radius,
										bool force, draw_callback * cb)
	{
		buffered_obstacle_adder boa(this, force, cb);
		gridframe.DrawGlobalCircle(globx, globy, radius, boa);
		return boa.count;
	}
	
	
	size_t Mapper2d::
	AddObstacleLine(double gx0, double gy0,
									double gx1, double gy1,
									bool force,
									draw_callback * cb)
	{
		buffered_obstacle_adder boa(this, force, cb);
		gridframe.DrawGlobalLine(gx0, gy0, gx1, gy1,
														 std::numeric_limits<ssize_t>::min(),
														 std::numeric_limits<ssize_t>::max(), 
														 std::numeric_limits<ssize_t>::min(),
														 std::numeric_limits<ssize_t>::max(),
														 boa);
		return boa.count;
	}
	
	
	bool Mapper2d::always_grow::
	operator () (TraversabilityMap & travmap,
							 ssize_t ix, ssize_t iy)
	{
		return travmap.Autogrow(ix, iy, travmap.freespace);
	}
	
	
	/** \todo Could cache a lot of the intermediate values, but this is
			only called during init. */
	int Mapper2d::
	ComputeCost(double dist_from_obstacle) const
	{
		if (dist_from_obstacle >= grown_safe_distance)
			return m_travmap->freespace;
		int cost(m_travmap->obstacle);
		if (dist_from_obstacle > grown_robot_radius) {
			double const normdist((dist_from_obstacle - grown_robot_radius) / buffer_zone);
			double const vv(m_travmap->freespace + 1
											+ (*m_cost_decay)(normdist) * (m_travmap->obstacle - m_travmap->freespace - 2));
			cost = boundval(m_travmap->freespace + 1, static_cast<int>(rint(vv)), m_travmap->obstacle - 1);
		}
		return cost;
	}
	
	
	void Mapper2d::
	InitAddmask()
	{
		m_addmask_x0 = 0;
		m_addmask_y0 = 0;
		m_addmask_x1 = 0;
		m_addmask_y1 = 0;
		const ssize_t offset(static_cast<ssize_t>(ceil(grown_safe_distance / gridframe.Delta())));
    for (ssize_t ix(-offset); ix <= offset; ++ix) {
      const double x2(sqr(ix * gridframe.Delta()));
      for (ssize_t iy(-offset); iy <= offset; ++iy) {
				int const cost(ComputeCost(sqrt(sqr(iy * gridframe.Delta()) + x2)));
				if (cost > m_travmap->freespace) {
					m_addmask.insert(make_pair(index_t(ix, iy), cost));
					// update the addmask's bounding box
					if (ix < m_addmask_x0)
						m_addmask_x0 = ix;
					if (ix > m_addmask_x1)
						m_addmask_x1 = ix;
					if (iy < m_addmask_y0)
						m_addmask_y0 = iy;
					if (iy > m_addmask_y1)
						m_addmask_y1 = iy;
				}
			}
		}
	}
	
	
	void Mapper2d::
	InitRemovemask()
	{
		m_removemask_x0 = 0;
		m_removemask_y0 = 0;
		m_removemask_x1 = 0;
		m_removemask_y1 = 0;
    for (addmask_t::const_iterator ii(m_addmask.begin()); ii != m_addmask.end(); ++ii)
			for (addmask_t::const_iterator jj(m_addmask.begin()); jj != m_addmask.end(); ++jj) {
				ssize_t const ix(ii->first.v0 - jj->first.v0); // "minus" because it's a convolution
				ssize_t const iy(ii->first.v1 - jj->first.v1);
				m_removemask.insert(make_pair(index_t(ix, iy), false));
				if (ix < m_removemask_x0)
					m_removemask_x0 = ix;
				if (ix > m_removemask_x1)
					m_removemask_x1 = ix;
				if (iy < m_removemask_y0)
					m_removemask_y0 = iy;
				if (iy > m_removemask_y1)
					m_removemask_y1 = iy;
			}
    for (addmask_t::const_iterator ii(m_addmask.begin()); ii != m_addmask.end(); ++ii)
			m_removemask[ii->first] = true;
	}
	
	
	double linear_travmap_cost_decay::
	operator () (double normdist) const
	{
		return 1 - normdist;
	}
	
	
	exponential_travmap_cost_decay::
	exponential_travmap_cost_decay(double _power)
		: power(_power)
	{
	}
	
	
	double exponential_travmap_cost_decay::
	operator () (double normdist) const
	{
		return pow(1 - normdist, power);
	}
	
	
	void Mapper2d::
	ClearAllObstacles(draw_callback * cb)
	{
		for (ssize_t ix(m_travmap->GetXBegin()); ix < m_travmap->GetXEnd(); ++ix)
			for (ssize_t iy(m_travmap->GetYBegin()); iy < m_travmap->GetYEnd(); ++iy)
				if ( ! m_travmap->IsFree(ix, iy))
					m_travmap->SetFree(ix, iy, cb);
	}
	
	
	boost::shared_ptr<TraversabilityMap const> Mapper2d::
	GetTravmap() const
	{
		return m_travmap;
	}
	
	
	boost::shared_ptr<TraversabilityMap> Mapper2d::
	GetTravmap()
	{
		return m_travmap;
	}
	
}

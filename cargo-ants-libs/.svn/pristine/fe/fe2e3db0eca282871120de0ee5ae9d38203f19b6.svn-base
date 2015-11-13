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


#ifndef SUNFLOWER_DISTANCEOBJECTIVE_HPP
#define SUNFLOWER_DISTANCEOBJECTIVE_HPP


#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Scan.hpp>
#include <sfl/util/Hull.hpp>
#include <sfl/util/array2d.hpp>
#include <sfl/dwa/Objective.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>


namespace sfl {
  
  
  class Lookup;
  
  
  /**
     The distance objective is what makes the dynamic window chose
     <ul>
     <li> which motion commands are forbidden </li>
     <li> which motion commands maximise clearance </li>
     </ul>
  
     It basically does 2 things: Predict collisions (and how far away
     they are), and translate these predictions into an objective value
     (typically between 0 and 1).

     All collision predictions are precalculated and stored in a lookup
     table. Apart from the traditional time-space tradeoff, this also
     implies an effectively "blown up" robot outline (the amount of
     enlargement depends on the grid resolution, it's on the order
     of sqrt(2) * resolution / 2).

     \note Don't get confused by the fact that there are two grids used
     here: One comes from Objectve (or rather, from DynamicWindow) and
     represents the actuator speed space of the robot. The other is a
     local obstacle grid attached to the robot and is a sampling of the
     workspace. The latter is (mostly) referred to with grid_WHATEVER.
  */
  class DistanceObjective:
    public Objective
  {
  public:
    typedef enum { NONE, ZONE, HULL } region_t;
    
    DistanceObjective(/** NOT a shared ptr because of circularity */
		      const DynamicWindow & dynamic_window,
		      boost::shared_ptr<const RobotModel> robot_model,
		      double grid_width,
		      double grid_height,
		      double grid_resolution);
    
    virtual void Initialize(std::ostream * progress_stream);
    
    virtual bool YieldsAdmissible() const { return true; }
    
    virtual bool Admissible(int qdlIndex, int qdrIndex) const;
    
    bool CheckLookup(std::ostream * os) const;
    
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value). */
    virtual void Calculate(double timestep, size_t qdlMin, size_t qdlMax,
			   size_t qdrMin, size_t qdrMax,
			   double carrot_lx, double carrot_ly,
			   boost::shared_ptr<const Scan> local_scan);
    
    void GetRange(double & x0, double & y0, double & x1, double & y1) const;
    size_t DimX() const;
    size_t DimY() const;
    bool CellOccupied(size_t ix, size_t iy) const;
    
    /** \return one of region_t */
    short GetRegion(size_t ix, size_t iy) const;
    
    /** \return invalidTime if not valid (no collision) */
    double CollisionTime(size_t ix, size_t iy, size_t iqdl, size_t iqdr) const;
    
    /** \note Expects signed ssize_t to be consistent with FindXindex(). */
    double FindXlength(ssize_t i) const;
    
    /** \note Expects signed ssize_t to be consistent with FindYindex(). */
    double FindYlength(ssize_t i) const;
    
    /** \note Returns signed ssize_t to facilitate domain detection. */
    ssize_t FindXindex(double d) const;
    
    /** \note Returns signed ssize_t to facilitate domain detection. */
    ssize_t FindYindex(double d) const;
    
    boost::shared_ptr<const Hull> GetHull() const;
    boost::shared_ptr<const Hull> GetPaddedHull() const;
    boost::shared_ptr<const Hull> GetEvaluationHull() const;
    double GetDeltaX() const;
    double GetDeltaY() const;
    size_t GetNNear() const;
    bool GetNear(size_t index, double & lx, double & ly) const;
    bool ObstacleInHull() const;
    
    void DumpGrid(std::ostream & os, const char * prefix) const;
    
    double PredictCollision(const Hull & hull,
			    double qdl, double qdr,
			    double lx, double ly) const;
    
    static const double invalidTime;
    
    
  protected:
    typedef array2d<boost::shared_ptr<Lookup> > lookup_t;
    typedef vec2d<double> point_t;
    typedef std::vector<point_t> nearpoints_t;
    
    
    void ResetGrid();
    
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value). */
    void UpdateGrid(boost::shared_ptr<const Scan> local_scan);
    
    /** \return The minimum predicted time until collision for a given
	actuator command, given the current obstacles. In case the are
	no collisions, or if they all would appear after the maximum
	braking time, invalidTime is returned to signal "no
	danger".
	\todo OLD HACK returns epsilon if imminent collision, rfct pls!!!
    */
    double MinTime(size_t iqdl, size_t iqdr);    
    
    
    static const size_t nearpoints_chunksize;
    const double m_qdd_max;
    const double m_max_brake_time;
    const boost::shared_ptr<const RobotModel> m_robot_model;
    const boost::shared_ptr<const Hull> m_hull;
    boost::shared_ptr<const Hull> m_padded_hull;
    boost::shared_ptr<const Hull> m_evaluation_hull; // overkill, use bbox
    
    boost::scoped_ptr<array2d<bool> > m_grid;
    boost::scoped_ptr<array2d<short> > m_region;
    
    double m_x0, m_y0, m_x1, m_y1; // bounding box of grid (currently symetric)
    double _dx, _dy, _dxInv, _dyInv; // effective resolution along x and y
    size_t _dimx, _dimy;	// dimensions of grid
    size_t m_n_zone;
    bool m_point_in_hull;
    std::vector<double> m_qd_lookup;
    array2d<double> m_base_brake_time;
    boost::scoped_ptr<lookup_t> m_time_lookup;
    nearpoints_t m_nearpoints;
    size_t m_n_nearpoints;
  };

}

#endif // SUNFLOWER_DISTANCEOBJECTIVE_HPP

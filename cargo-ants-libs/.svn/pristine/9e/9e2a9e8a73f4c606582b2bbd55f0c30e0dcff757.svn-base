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


#include "PNF.hpp"
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <pnf/Flow.hpp>
#include <pnf/BufferZone.hpp>
#include <pnf/PNFRiskMap.hpp>
#include <iostream>
#include <cmath>

#include <pthread.h>
#include <errno.h>

// for deletion of smart pointers containing forwardly declared types:
#include <estar/Algorithm.hpp>
#include <estar/Facade.hpp>
#include <estar/Kernel.hpp>

// this must be included last (or at least pretty late), otherwise it
// breaks... did not analyse why though, must be some preprocessor
// issues
#include <npm/pdebug.hpp>


using sfl::GridFrame;
using sfl::Frame;
using sfl::absval;
using pnf::BufferZone;
using pnf::Flow;
using std::cerr;
using namespace npm;


namespace local {
  
  static void * run(void * arg);
  
  typedef enum { IDLE, RUNNING, WAIT, QUIT } state_t;
  
  class poster
  {
    pthread_t thread_id;
    
  public:
    pthread_mutex_t mutex;
    state_t state;
    PNF::step_t step;
    pnf::Flow * flow;
    const double robot_r;
    const bool enable_thread;
    
    poster(pnf::Flow * _flow, double _robot_r,
	   bool _enable_thread)
      : state(WAIT), step(PNF::NONE), flow(_flow),
	robot_r(_robot_r), enable_thread(_enable_thread)
    {
      // actually, we could skip even the mutexing if !enable_thread
      pthread_mutexattr_t attr;
      pthread_mutexattr_init(&attr);
      pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
      pthread_mutex_init(&mutex, &attr);
      pthread_mutexattr_destroy(&attr);
      if( ! enable_thread)
	thread_id = 0;
      else
	switch(pthread_create(&thread_id, 0, run, this)){
	case EAGAIN:
	  cerr << __func__ << "(): insufficient resources for thread.\n";
	  exit(EXIT_FAILURE);
	case EINVAL:
	  cerr << __func__ << "(): BUG [invalid attr for pthread_create()]\n";
	  exit(EXIT_FAILURE);
	}
    }
    
    ~poster()
    {
      if(EINVAL == pthread_mutex_lock(&mutex)){
	PDEBUG("WARNING pthread_mutex_lock() failed ==> stale poster\n");
	return;
      }
      state = QUIT;
      if(EINVAL == pthread_mutex_unlock(&mutex)){
	PDEBUG("WARNING pthread_mutex_unlock() failed ==> stale poster\n");
	return;
      }
      if(enable_thread)
	pthread_join(thread_id, 0);
      pthread_mutex_destroy(&mutex);
    }
    
    /** \return TRUE iff we should keep running the 'thread' */
    bool do_step();
  };
  
}

namespace npm {

PNF::
PNF(double _robot_x, double _robot_y,
    double _robot_r, double _robot_v,
    double _goal_x, double _goal_y, double _goal_r,
    double _grid_width, size_t _grid_wdim,
    bool enable_thread)
  : robot_x(_robot_x),
    robot_y(_robot_y),
    robot_r(_robot_r),
    robot_v(_robot_v),
    goal_x(_goal_x),
    goal_y(_goal_y),
    goal_r(_goal_r),
    grid_width(_grid_width),
    grid_wdim(_grid_wdim),
    resolution(grid_width / grid_wdim)
{
  const double dx(goal_x - robot_x);
  const double dy(goal_y - robot_y);
  const double width_offset(resolution * (grid_wdim - 1) / 2.0);
  double xm_frame(-width_offset);
  double ym_frame(-width_offset);
  const Frame frame(robot_x, robot_y, atan2(dy, dx));
  frame.To(xm_frame, ym_frame);
  
  m_frame.reset(new GridFrame(xm_frame, ym_frame, frame.Theta(), resolution));
  
  const size_t
    xdim(static_cast<size_t>(ceil((sqrt(dx*dx+dy*dy)+grid_width)/resolution)));
  const bool perform_convolution(false);
  const bool alternate_worst_case(false);
  m_flow.reset(Flow::Create(xdim, grid_wdim, resolution,
			    perform_convolution, alternate_worst_case));
  
  // very important to use LOCAL grid coordinates
  m_frame->From(_robot_x, _robot_y);
  if( ! m_flow->SetRobot(_robot_x, _robot_y, robot_r, robot_v)){
    cerr << "m_flow->SetRobot(" << _robot_x << ", " << _robot_y << ", "
	 << robot_r << ", " << robot_v << ") failed\n";
    exit(EXIT_FAILURE);
  }
  PVDEBUG("SetRobot(%g   %g   %g   %g) on flow %lu\n",
	  _robot_x, _robot_y, robot_r, robot_v, m_flow.get());

  m_frame->From(_goal_x, _goal_y);
  if( ! m_flow->SetGoal(_goal_x, _goal_y, goal_r)){
    cerr << "m_flow->SetGoal(" << _goal_x << ", " << _goal_y << ", "
	 << goal_r << ") failed\n";
    exit(EXIT_FAILURE);
  }
  PVDEBUG("SetGoal(%g   %g   %g) on flow %lu\n",
	  _goal_x, _goal_y, goal_r, m_flow.get());
  
  m_poster.reset(new local::poster(m_flow.get(), robot_r, enable_thread));
}


bool PNF::
AddStaticObject(double globx, double globy)
{
  PVDEBUG("%g   %g\n", globx, globy);
  Wait();
  const GridFrame::index_t
    idx(m_frame->GlobalIndex(GridFrame::position_t(globx, globy)));
  if((idx.v0 < 0) || (idx.v1 < 0)
     || (static_cast<ssize_t>(idx.v0) >= m_flow->xsize)
     || (static_cast<ssize_t>(idx.v1) >= m_flow->ysize))
    return false;
  m_flow->AddStaticObject(static_cast<ssize_t>(idx.v0),
			  static_cast<ssize_t>(idx.v1));
  return true;
}


bool PNF::
SetDynamicObject(size_t id, double globx, double globy,
		 double r, double v)
{
  PVDEBUG("%lu   %g   %g   %g   %g\n", id, globx, globy, r, v);
  Wait();
  m_frame->From(globx, globy);
  return m_flow->SetDynamicObject(id, globx, globy, r, v);
}


bool PNF::
RemoveStaticObject(double globx, double globy)
{
  PVDEBUG("%g   %g\n", globx, globy);
  Wait();  
  const GridFrame::index_t
    idx(m_frame->GlobalIndex(GridFrame::position_t(globx, globy)));
  if((idx.v0 < 0) || (idx.v1 < 0)
     || (static_cast<ssize_t>(idx.v0) >= m_flow->xsize)
     || (static_cast<ssize_t>(idx.v1) >= m_flow->ysize))
    return false;
  m_flow->RemoveStaticObject(static_cast<ssize_t>(idx.v0),
			     static_cast<ssize_t>(idx.v1));
  return true;
}


void PNF::
RemoveDynamicObject(size_t id)
{
  PVDEBUG("%lu\n", id);
  Wait();
  m_flow->RemoveDynamicObject(id);
}


void PNF::
StartPlanning()
{
  PVDEBUG("m_poster->flow is instance %lu\n", m_poster->flow);
  Wait();
  if(EINVAL == pthread_mutex_lock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_lock() error\n";
    exit(EXIT_FAILURE);
  }
  m_poster->state = local::RUNNING;
  m_poster->step = NONE;
  if(EINVAL == pthread_mutex_unlock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_unlock() error\n";
    exit(EXIT_FAILURE);
  }
}


PNF::step_t PNF::
GetStep(bool fake_thread) const
{
  if(( ! m_poster->enable_thread)
     && fake_thread)		// fake it!
    m_poster->do_step();	// ignore retval (indicates keep_running)
  if(EINVAL == pthread_mutex_lock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_lock() error\n";
    exit(EXIT_FAILURE);
  }
  step_t result(NONE);
  if(local::WAIT != m_poster->state)
    result = m_poster->step;
  if(EINVAL == pthread_mutex_unlock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_unlock() error\n";
    exit(EXIT_FAILURE);
  }
  return result;
}


void PNF::
Wait()
{
  if(EINVAL == pthread_mutex_lock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_lock() error\n";
    exit(EXIT_FAILURE);
  }
  m_poster->state = local::WAIT;
  m_poster->step = NONE;
  if(EINVAL == pthread_mutex_unlock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_unlock() error\n";
    exit(EXIT_FAILURE);
  }
}

}


namespace local {
  
  
  bool poster::do_step()
  {
    static const pnf::Sigma riskmap(0.95, 1.5); // TO DO: Magic numbers stink!
    state_t prevstate(static_cast<state_t>(-1));
    
    if(EINVAL == pthread_mutex_lock(&mutex)){
      cerr << __func__ << "(): pthread_mutex_lock() error\n";
      exit(EXIT_FAILURE);
    }
    
    if(WAIT == state){
      if(prevstate != state)
	PDEBUG("0x%08zX waiting\n", (size_t) thread_id);
      prevstate = state;
      // do nothing
    }
    
    else if(QUIT == state){
      PDEBUG("0x%08zX quit\n", (size_t) thread_id);
      if(EINVAL == pthread_mutex_unlock(&mutex)){
	cerr << __func__ << "(): pthread_mutex_unlock() error\n";
	exit(EXIT_FAILURE);
      }
      return false;
    }
    
    else{
      PVDEBUG("flow is instance %lu\n", flow);
      
      if( ! flow->HaveEnvdist()){
	PDEBUG("PropagateEnvdist()...\n");
	flow->PropagateEnvdist(false);
	if( ! flow->HaveEnvdist()){
	  PDEBUG("PropagateEnvdist() failed\n");
	  step = PNF::NONE;
	}
	else{
	  PDEBUG("PropagateEnvdist() succeeded\n");
	  flow->MapEnvdist();
	  step = PNF::ENVDIST;
	}
      }
      
      else if( ! flow->HaveAllObjdist()){
	// could use PropagateObjdist(size_t id) for granularity...
	PVDEBUG("PropagateAllObjdist()...\n");
	flow->PropagateAllObjdist();
	if( ! flow->HaveAllObjdist()){
	  PDEBUG("PropagateAllObjdist() failed\n");
	  step = PNF::ENVDIST;
	}
	else{
	  PDEBUG("PropagateAllObjdist() succeeded\n");
	  step = PNF::OBJDIST;
	}
      }
      
      else if( ! flow->HaveRobdist()){
	PVDEBUG("PropagateRobdist()...\n");
	flow->PropagateRobdist();
	if( ! flow->HaveRobdist()){
	  PDEBUG("PropagateRobdist() failed\n");
	  step = PNF::OBJDIST;
	}
	else{
	  PDEBUG("PropagateRobdist() succeeded\n");
	  const double static_buffer_factor(1);	// multiplies robot_radius (?)
	  const double static_buffer_degree(2);
	  flow->ComputeAllCooc(static_buffer_factor, static_buffer_degree);
	  flow->ComputeRisk(riskmap);
	  step = PNF::ROBDIST;
	}
      }
      
      else if( ! flow->HavePNF()){
	PVDEBUG("PropagatePNF()...\n");
	flow->PropagatePNF();
	if( ! flow->HavePNF()){
	  PDEBUG("PropagatePNF() failed\n");
	  step = PNF::ROBDIST;
	}
	else{
	  PDEBUG("PropagatePNF() succeeded\n");
	  step = PNF::DONE;
	}
      }
      
      if(PNF::DONE == step)
	state = IDLE;
      else
	state = RUNNING;
      
      if(prevstate != state)
	PDEBUG("0x%08zX %s\n", (size_t) thread_id, 
	       (IDLE == state) ? "idle" : "running");
      prevstate = state;
    }
    
    if(EINVAL == pthread_mutex_unlock(&mutex)){
      cerr << __func__ << "(): pthread_mutex_unlock() error\n";
      exit(EXIT_FAILURE);
    }
    
    return true;
  }
  
  
  void * run(void * arg)
  {
    poster * posterp(reinterpret_cast<poster*>(arg));
    bool keep_running(true);
    while(keep_running)
      keep_running  = posterp->do_step();
    return 0;
  }


struct flow_draw_callback
  : public GridFrame::draw_callback
{
  flow_draw_callback(Flow * flow): m_flow(flow) {}
  void operator () (ssize_t ix, ssize_t iy) {
    PVDEBUG("grid: %zu   %zu\n", ix, iy);
    m_flow->AddStaticObject(ix, iy);
  }
  Flow * m_flow;
};

}


namespace npm {

bool PNF::
AddStaticLine(double x0, double y0,
	      double x1, double y1)
{
  PVDEBUG("global: %g   %g   %g   %g\n", x0, y0, x1, y1);
  Wait();
  
  local::flow_draw_callback cb(m_flow.get());
  return 0 != m_frame->DrawGlobalLine(x0, y0, x1, y1,
				      0, m_flow->xsize, 0, m_flow->ysize,
				      cb);
}

}

/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
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

#include "AslBot.hpp"
#include "PathDrawing.hpp"
#include "ThreadDrawing.hpp"
#include "smart_params.hpp"
#include <asl/Algorithm.hpp>
#include <asl/MappingThread.hpp>
#include <asl/PlanningThread.hpp>
#include <asl/ControlThread.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Pose.hpp>
#include <sfl/gplan/Mapper2d.hpp>
#include <sfl/util/strutil.hpp>
#include <estar/Facade.hpp>
#include <estar/graphics.hpp>
#include "../common/HAL.hpp"
#include "../common/RobotServer.hpp"
#include "../common/GoalInstanceDrawing.hpp"
#include "../common/TraversabilityDrawing.hpp"
#include "../common/World.hpp"
#include "../common/wrap_gl.hpp"
#include "../common/MapperUpdateDrawing.hpp"
#include "../estar/EstarDrawing.hpp"
#include "../common/pdebug.hpp"
#include "../common/RobotDescriptor.hpp"
#include <iostream>
#include <unistd.h>							// for usleep (on OpenBSD)


using namespace npm;
using namespace sfl;
using namespace asl;
using namespace estar;
using namespace boost;
using namespace std;


class AslPlanProxy:
	public PlanProxy,
	public KeyListener
{
public:
  AslPlanProxy(const asl::Algorithm * asl_algo, bool enabled)
		: m_asl_algo(asl_algo), m_enabled(enabled) {}
	
  virtual const estar::Facade * GetFacade()
	{ return m_asl_algo->GetEstar().get(); }
	
  virtual const sfl::GridFrame * GetFrame()
	{ return & m_asl_algo->GetGridFrame(); }
	
	virtual bool Enabled() const { return m_enabled; }
	
	virtual void KeyPressed(unsigned char key)
	{ if('o' == key) m_enabled = ! m_enabled; }
	
  const asl::Algorithm * m_asl_algo;
	bool m_enabled;
};


class AslTraversabilityProxy:
	public RDTravProxy,
	public KeyListener
{
public:
	AslTraversabilityProxy(shared_ptr<RDTravmap> rdtravmap, bool enabled)
		: RDTravProxy(rdtravmap) { enable = enabled; }
	
	virtual void KeyPressed(unsigned char key)
	{ if('o' == key) enable = ! enable; }
};


class AslColorScheme: public gfx::ColorScheme {
public:
	AslColorScheme(): m_aslbot_value(0), m_queue_bottom(0) {}
	
	void Update(const asl::Algorithm * algo, const AslBot & aslbot) {
		shared_ptr<const estar::Facade> estar(algo->GetEstar());
		if( ! estar){								// using "fake" planner
			m_queue_bottom = 2;				// whatever... never used in this case?
			m_aslbot_value = 1;
			return;
		}
		if( ! estar->GetLowestInconsistentValue(m_queue_bottom))
			m_queue_bottom = estar::infinity;
		const Frame pose(aslbot.GetPose());
		if ( ! m_rdtravmap)					// bit of a hack...
			m_rdtravmap = algo->GetMapper2d()->CreateRDTravmap();
		const GridFrame::index_t
			idx(m_rdtravmap->GetGridFrame().GlobalIndex(pose.X(), pose.Y()));
		if (m_rdtravmap->IsValid(idx.v0, idx.v1))
			m_aslbot_value = estar->GetValue(idx.v0, idx.v1);
		else
			m_aslbot_value = estar::infinity;
		PVDEBUG("aslbot: %g   bottom %g\n", m_aslbot_value, m_queue_bottom);
	}
	
	virtual void Set(double value) const {
		if(value <= 0)
			glColor3d(0, 0, 0);
		else if(value >= m_queue_bottom)
			glColor3d(0.4, 0, 0);
		else{
			double green(value / m_aslbot_value);
			if(green >= 1)
				green = green - 1;
			const double red(value / m_queue_bottom);
			const double blue(1 - red);
			glColor3d(red, green, blue);
		}
	}

private:
	shared_ptr<RDTravmap> m_rdtravmap;
	double m_aslbot_value;
	double m_queue_bottom;
};


class CycleColorScheme: public gfx::ColorScheme {
public:
	CycleColorScheme(double period, double width)
		: m_cc(new gfx::ColorCycle(gfx::ColorScheme::Get(gfx::INVERTED_GREY),
															 period, width))
	{}
	
	virtual void Set(double value) const {
		if (value >= estar::infinity)
			glColor3d(0.4, 0, 0);
		else
			m_cc->Set(value);
	}
	
private:
	shared_ptr<gfx::ColorCycle> m_cc;
};


class MetaColorScheme: public gfx::ColorScheme {
public:
	virtual void Set(double value) const {
		if(value >= (1 - estar::epsilon))
			glColor3d(0, 0, 1);
		else if(value <= estar::epsilon)
			glColor3d(1, 0, 0);
		else
			glColor3d(value, value, value);
	}
};


class AslGoalDrawing: public GoalInstanceDrawing {
public:
	AslGoalDrawing(const std::string name, const asl::Algorithm * asl_algo)
		: GoalInstanceDrawing(name, Goal()), m_asl_algo(asl_algo) {}
	
	virtual void Draw() {
		shared_ptr<const Goal> goal(m_asl_algo->GetGoal());
		if(goal)
			GoalInstanceDrawing::Draw(*goal);
	}
	
  const asl::Algorithm * m_asl_algo;
};


namespace foo {
	
	// if you get "strange" compiler errors, try including the header(s)
	// that define the thread you're trying to plot
	template<>
	void set_bg_color(asl::Planner::status_t status)
	{
		switch(status){
		case asl::Planner::HAVE_PLAN:   glColor3d(0,   1,   0  ); break;
		case asl::Planner::BUFFERING:   glColor3d(0.5, 1,   0  ); break;
		case asl::Planner::PLANNING:    glColor3d(1,   0.5, 0  ); break;
		case asl::Planner::AT_GOAL:     glColor3d(0,   0,   1  ); break;
		case asl::Planner::UNREACHABLE:
		case asl::Planner::GOAL_OUT_OF_GRID:
		case asl::Planner::ROBOT_OUT_OF_GRID:
		case asl::Planner::ROBOT_IN_OBSTACLE: glColor3d(1,   0,   0  ); break;
		case asl::Planner::NO_GOAL:           glColor3d(1, 0.5,   0.5); break;
		case asl::Planner::ERROR:             glColor3d(1,   0,   0.5); break;
		default:                              glColor3d(1,   0,   1  );
		}
	}
		
	// if you get "strange" compiler errors, try including the header(s)
	// that define the thread you're trying to plot
	template<>
	void set_fg_color(asl::Planner::status_t status)
	{
		switch(status){
		case asl::Planner::HAVE_PLAN:   glColor3d(0.5, 0,   0  ); break;
		case asl::Planner::BUFFERING:   glColor3d(0.5, 0,   0.5); break;
		case asl::Planner::PLANNING:    glColor3d(0.5, 0,   0.5); break;
		case asl::Planner::AT_GOAL:     glColor3d(0,   0.5, 0  ); break;
		case asl::Planner::UNREACHABLE:
		case asl::Planner::GOAL_OUT_OF_GRID:
		case asl::Planner::ROBOT_OUT_OF_GRID:
		case asl::Planner::ROBOT_IN_OBSTACLE: glColor3d(0,   0,   0.5); break;
		case asl::Planner::NO_GOAL:           glColor3d(1, 0.5,   0.5); break;
		case asl::Planner::ERROR:             glColor3d(0,   0.5, 0.5); break;
		default:                              glColor3d(0,   0.5, 0.5);
		}
	}
	
	template<>
	struct stats_select<asl::MappingThread> {
		typedef asl::MappingThread::stats_t stats_t;
	};
	
	template<>
	struct stats_select<asl::PlanningThread> {
		typedef asl::PlanningThread::stats_t stats_t;
	};
	
	// if you get "strange" compiler errors, try including the header(s)
	// that define the thread you're trying to plot
	template<>
	void set_bg_color(asl::Controller::status_t status)
	{
		switch(status){
		case asl::Controller::SUCCESS: glColor3d(0,   1,   0  ); break;
		case asl::Controller::STARVED: glColor3d(1,   0,   1  ); break;
		case asl::Controller::FAILURE: glColor3d(1,   0,   0  ); break;
		case asl::Controller::ERROR:   glColor3d(1,   0,   0.5); break;
		default:                       glColor3d(1,   0,   1  );
		}
	}
	
	// if you get "strange" compiler errors, try including the header(s)
	// that define the thread you're trying to plot
	template<>
	void set_fg_color(asl::Controller::status_t status)
	{
		switch(status){
		case asl::Controller::SUCCESS: glColor3d(1,   0,   1  ); break;
		case asl::Controller::STARVED: glColor3d(0,   0.5, 0  ); break;
		case asl::Controller::FAILURE: glColor3d(0,   1,   1  ); break;
		case asl::Controller::ERROR:   glColor3d(0,   1,   0.5); break;
		default:                       glColor3d(0,   1,   0  );
		}
	}
	
	template<>
	struct stats_select<asl::ControlThread> {
		typedef asl::ControlThread::stats_t stats_t;
	};

}


AslBot::
AslBot(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, 2, true),
		m_asl_cs(new AslColorScheme()),
		m_error(false)
{
}


void AslBot::
CreateMePlease(shared_ptr<RobotDescriptor> descriptor, const World & world)
{
  smartparams params(descriptor);
	
	double carrot_distance(5);
	string_to(descriptor->GetOption("carrot_distance"), carrot_distance);
	
	double carrot_stepsize(0.5);
	string_to(descriptor->GetOption("carrot_stepsize"), carrot_stepsize);
	
	size_t carrot_maxnsteps(30);
	string_to(descriptor->GetOption("carrot_maxnsteps"), carrot_maxnsteps);
	
	double replan_distance(3);
	string_to(descriptor->GetOption("replan_distance"), replan_distance);
	
	string traversability_file(descriptor->GetOption("traversability_file"));
	if(traversability_file == ""){
    cerr << "ERROR: AslBot needs an a-priori traversability map.\n"
				 << "  Use the traversability_file option!\n";
    exit(EXIT_FAILURE);
  }
	
	int estar_step(-1);
	string_to(descriptor->GetOption("estar_step"), estar_step);
	
	double wavefront_buffer(2);
	string_to(descriptor->GetOption("wavefront_buffer"), wavefront_buffer);
	
	string goalmgr_filename(descriptor->GetOption("goalmgr_filename"));
	if(goalmgr_filename != ""){
		if(descriptor->HaveGoals()){
			cerr << "ERROR goalmgr_filename option conflicts with Goal statements.\n"
					 << "  Cannot mix npm Goal with Option goalmgr_filename.\n";
			exit(EXIT_FAILURE);
		}
	}
	else if( ! descriptor->HaveGoals()){
		cerr << "ERROR no goals and no goalmgr_filename option given.\n";
		exit(EXIT_FAILURE);
	}
	
	m_odo.reset(new Odometry(GetHAL(), RWlock::Create("aslbot")));	
	
	shared_ptr<estar::AlgorithmOptions>
		estar_options(new estar::AlgorithmOptions());
	string_to(descriptor->GetOption("estar_check_upwind"),
						estar_options->check_upwind);
	string_to(descriptor->GetOption("estar_check_local_consistency"),
						estar_options->check_local_consistency);
	string_to(descriptor->GetOption("estar_check_queue_key"),
						estar_options->check_queue_key);
	string_to(descriptor->GetOption("estar_auto_reset"),
						estar_options->auto_reset);
	string_to(descriptor->GetOption("estar_auto_flush"),
						estar_options->auto_flush);
	
	shared_ptr<travmap_grow_options> grow_options;
	{
		double travmap_grow_margin_low(-1);
		double travmap_grow_margin_high(-1);
		bool grow_options_given(false);
		grow_options_given |=
			string_to(descriptor->GetOption("travmap_grow_margin_low"),
								travmap_grow_margin_low);
		grow_options_given |=
			string_to(descriptor->GetOption("travmap_grow_margin_high"),
								travmap_grow_margin_high);
		if (grow_options_given) {
			if ((travmap_grow_margin_low < 0) && (travmap_grow_margin_high > 0))
				travmap_grow_margin_low = 0.5 * travmap_grow_margin_high;
			else if ((travmap_grow_margin_high < 0) && (travmap_grow_margin_low > 0))
				travmap_grow_margin_high = 2 * travmap_grow_margin_low;
			if ((travmap_grow_margin_low <= 0) && (travmap_grow_margin_low <= 0))
				cerr << "WARNING ignoring invalid travmap_grow_options\n"
						 << " travmap_grow_margin_low " << travmap_grow_margin_low << "\n"
						 << " travmap_grow_margin_high " << travmap_grow_margin_high<<"\n";
			else
				grow_options.
					reset(new travmap_grow_options(travmap_grow_margin_low,
																				 travmap_grow_margin_high));
		}
	}
	
	bool estar_grow_grid(false);
	string_to(descriptor->GetOption("estar_grow_grid"), estar_grow_grid);
	
	m_mscan.reset(new Multiscanner(GetHAL()));
	InitScanners(m_mscan, params);
	
	bool swiped_map_update(false);
	string_to(descriptor->GetOption("swiped_map_update"), swiped_map_update);
	
	double max_swipe_distance(4);
	string_to(descriptor->GetOption("max_swipe_distance"), max_swipe_distance);
	
	double robot_radius;					// XXX arghlgmpf!!!!
	InitAlgorithm(descriptor, params,
								carrot_distance, carrot_stepsize, carrot_maxnsteps,
								replan_distance, traversability_file, estar_step,
								wavefront_buffer, goalmgr_filename, swiped_map_update, max_swipe_distance,
								estar_options, grow_options, estar_grow_grid, robot_radius);
	
	m_simul_rwlock = RWlock::Create("npm::AslBot");
	if( ! m_simul_rwlock){
		cerr << "ERROR sfl::RWlock::Create() failed\n";
		exit(EXIT_FAILURE);
	}
	m_simul_rwlock->Wrlock();
	
	int thread_statlen;
	if( ! string_to(descriptor->GetOption("thread_statlen"), thread_statlen))
		thread_statlen = 100;
	if(thread_statlen < 1){
		cerr << "ERROR invalid thread_statlen = " << thread_statlen << "< 1\n";
		exit(EXIT_FAILURE);
	}
	
	if( ! string_to(descriptor->GetOption("simul_usecsleep"),
									m_simul_usecsleep))
		m_simul_usecsleep = -1;
	
	m_mapping_thread.reset(new asl::MappingThread(m_asl_algo->GetMapper(),
																								m_odo, m_mscan,
																								thread_statlen,
																								m_simul_rwlock));
	if( ! string_to(descriptor->GetOption("mapping_usecsleep"),
									m_mapping_usecsleep))
		m_mapping_usecsleep = -1;
	if(0 <= m_mapping_usecsleep){
		PDEBUG("spawning mapping thread with usecsleep %u\n",
					 m_mapping_usecsleep);
		m_mapping_thread->Start(m_mapping_usecsleep);
	}
	else
		PDEBUG("mapping thread remains in synch with simulation\n");
	
	m_planning_thread.reset(new asl::PlanningThread(m_asl_algo->GetPlanner(),
																									GetHAL(),
																									thread_statlen,
																									m_simul_rwlock));
	if( ! string_to(descriptor->GetOption("planning_usecsleep"),
									m_planning_usecsleep))
		m_planning_usecsleep = -1;
	if(0 <= m_planning_usecsleep){
		PDEBUG("spawning planning thread with usecsleep %u\n",
					 m_planning_usecsleep);
		m_planning_thread->Start(m_planning_usecsleep);
	}
	else
		PDEBUG("planning thread remains in synch with simulation\n");
	
	m_control_thread.
		reset(new asl::ControlThread(0.1,	// XXX magic value
																 m_asl_algo->GetController(),
																 GetHAL(),
																 thread_statlen, m_simul_rwlock));
	if( ! string_to(descriptor->GetOption("control_usecsleep"),
									m_control_usecsleep))
		m_control_usecsleep = -1;
	if(0 <= m_control_usecsleep){
		PDEBUG("spawning control thread with usecsleep %u\n",
					 m_control_usecsleep);
		m_control_thread->Start(m_control_usecsleep);
	}
	else
		PDEBUG("control thread remains in synch with simulation\n");
	
	InitDrive(params);
	InitBody(params);

  const string & name(descriptor->name);
	AddDrawing(new AslGoalDrawing(name + "_goaldrawing", m_asl_algo.get()));
  
	bool slow_drawing_enabled(true);
	string_to(descriptor->GetOption("slow_drawing_enabled"),
						slow_drawing_enabled);
	
	if(m_asl_algo->GetEstar()){	// otherwise, just faking it, no E* to draw
		shared_ptr<AslPlanProxy>
			slow_proxy(new AslPlanProxy(m_asl_algo.get(), slow_drawing_enabled));
		world.AddKeyListener(slow_proxy);
		shared_ptr<AslPlanProxy>
			fast_proxy(new AslPlanProxy(m_asl_algo.get(), true));
		shared_ptr<MetaColorScheme> mcs(new MetaColorScheme());

		double estar_drawing_cycle_period(2 * robot_radius);
		string_to(descriptor->GetOption("estar_drawing_cycle_period"),
							estar_drawing_cycle_period);
		double estar_drawing_cycle_width(0.5 * robot_radius);
		string_to(descriptor->GetOption("estar_drawing_cycle_width"),
							estar_drawing_cycle_width);
		shared_ptr<CycleColorScheme>
			ccs(new CycleColorScheme(estar_drawing_cycle_period,
															 estar_drawing_cycle_width));
		
		AddDrawing(new EstarDrawing(name + "_estar_meta",
																slow_proxy, EstarDrawing::META, mcs));
		AddDrawing(new EstarDrawing(name + "_estar_value",
																slow_proxy, EstarDrawing::VALUE, m_asl_cs));
		AddDrawing(new EstarDrawing(name + "_estar_value_cycle",
																slow_proxy, EstarDrawing::VALUE, ccs));
		AddDrawing(new EstarDrawing(name + "_estar_rhs",
																slow_proxy, EstarDrawing::RHS, m_asl_cs));
		AddDrawing(new EstarDrawing(name + "_estar_rhs_cycle",
																slow_proxy, EstarDrawing::RHS, ccs));
		AddDrawing(new EstarDrawing(name + "_estar_queue",
																fast_proxy, EstarDrawing::QUEUE));
		AddDrawing(new EstarDrawing(name + "_estar_upwind",
																slow_proxy, EstarDrawing::UPWIND));
		AddDrawing(new EstarDrawing(name + "_estar_obst",
																slow_proxy, EstarDrawing::OBST));
		AddDrawing(new EstarDrawing(name + "_estar_status",
																slow_proxy, EstarDrawing::STATUS));
		AddCamera(new EstarCamera(name + "_estar_cspace", fast_proxy));
	}
	
	{
		shared_ptr<AslTraversabilityProxy>
			asl_proxy(new AslTraversabilityProxy(m_asl_algo->GetMapper2d()->
																					 CreateRDTravmap(),
																					 slow_drawing_enabled));
		world.AddKeyListener(asl_proxy);
		AddDrawing(new TraversabilityDrawing(name + "_travmap",
																				 asl_proxy));
		AddCamera(new TraversabilityCamera(name + "_travmap", asl_proxy));
	}
	
	size_t gradplot_frequency(1);
	string_to(descriptor->GetOption("gradplot_frequency"), gradplot_frequency);
  AddDrawing(new PathDrawing(name + "_carrot", this, gradplot_frequency));
	
 	AddDrawing(new ThreadDrawing<asl::MappingThread>
						 (name + "_mapping_thread", m_mapping_thread.get()));
 	AddDrawing(new ThreadDrawing<asl::PlanningThread>
						 (name + "_planning_thread", m_planning_thread.get()));
 	AddDrawing(new ThreadDrawing<asl::ControlThread>
						 (name + "_control_thread", m_control_thread.get()));
	AddDrawing(new MapperUpdateDrawing(name + "_mapper_free",
																		 MapperUpdateDrawing::FREESPACE,
																		 m_asl_algo->GetMapper2d()));
	AddDrawing(new MapperUpdateDrawing(name + "_mapper_obst",
																		 MapperUpdateDrawing::OBSTACLE,
																		 m_asl_algo->GetMapper2d()));
	AddDrawing(new MapperUpdateDrawing(name + "_mapper_check",
																		 MapperUpdateDrawing::CHECK,
																		 m_asl_algo->GetMapper2d()));
	AddDrawing(new MapperUpdateDrawing(name + "_mapper_hole",
																		 MapperUpdateDrawing::HOLE,
																		 m_asl_algo->GetMapper2d()));
	AddDrawing(new MapperUpdateDrawing(name + "_mapper_repair",
																		 MapperUpdateDrawing::REPAIR,
																		 m_asl_algo->GetMapper2d()));
	
	MoreGraphics(name, world, slow_drawing_enabled);
}


AslBot::
~AslBot()
{
	m_simul_rwlock->Unlock();
	m_mapping_thread.reset();
	m_planning_thread.reset();
	m_control_thread.reset();
}


bool AslBot::
PrepareAction(double timestep)
{
	PDEBUG("\n\n==================================================\n");
	if(m_error){
		cerr << "AslBot ERROR state, restart simulation\n";
		GetHAL()->deprecated_speed_set(0, 0);
		return false;
	}
	
	m_odo->Update();
	m_mscan->UpdateAll();
	
	m_simul_rwlock->Unlock();
	
	if(0 > m_mapping_usecsleep)
		m_mapping_thread->Step();
 	if(m_mapping_thread->error){
 		cerr << "ERROR in mapping thread\n";
 		m_error = true;
 		m_simul_rwlock->Wrlock();
 		return false;
 	}
	
	if(0 > m_planning_usecsleep)
		m_planning_thread->Step();
	if(m_planning_thread->error){
		cerr << "ERROR in planning thread\n";
		m_error = true;
		m_simul_rwlock->Wrlock();
		return false;
	}
	m_asl_cs->Update(m_asl_algo.get(), *this);
	
	m_control_thread->timestep = timestep;
	if(0 > m_control_usecsleep)
		m_control_thread->Step();
	if(m_control_thread->error){
		cerr << "ERROR in control thread\n";
		m_error = true;
		m_simul_rwlock->Wrlock();
		return false;
	}
	
	if(m_simul_usecsleep > 0)
		usleep(m_simul_usecsleep);
	m_simul_rwlock->Wrlock();
	return true;
}


void AslBot::
InitPose(double x, double y, double theta)
{
}


void AslBot::
SetPose(double x, double y, double theta)
{
}


void AslBot::
GetPose(double & x, double & y, double & theta)
{
  const Frame & pose(GetServer()->GetTruePose());
  x = pose.X();
  y = pose.Y();
  theta = pose.Theta();
}


const Frame & AslBot::
GetPose() const
{
  return GetServer()->GetTruePose();
}


void AslBot::
SetGoal(double timestep, const sfl::Goal & goal)
{
	m_asl_algo->SetGoal(goal);
}


shared_ptr<const Goal> AslBot::
GetGoal()
{
  return m_asl_algo->GetGoal();
}


bool AslBot::
GoalReached()
{
  return m_asl_algo->GoalReached(*m_odo->Get());
}


void AslBot::
CopyPaths(boost::shared_ptr<asl::path_t> & clean,
					boost::shared_ptr<asl::path_t> & dirty) const
{
	clean = m_asl_algo->CopyCleanPath();
	dirty = m_asl_algo->CopyDirtyPath();
}

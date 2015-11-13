/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#include "TheaterRobot.hpp"
#include "TheaterRobotParameters.hpp"
#include "TheaterRobotProxy.hpp"
#include "TheaterMotionManager.hpp"
#include "SequencerDrawing.hpp"
#include "BSplineObjective.hpp"
#include "../robox/GoalDrawing.hpp"
#include "../robox/OCamera.hpp"
#include "../robox/ODrawing.hpp"
#include "../robox/DODrawing.hpp"
#include "../robox/DWDrawing.hpp"
#include "../robox/Proxies.hpp"
#include "../common/OdometryDrawing.hpp"
#include "../common/StillCamera.hpp"
#include <npm/common/Lidar.hpp>
#include <sfl/api/DiffDrive.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/expo/MotionController.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/bband/BubbleBand.hpp>
#include <theater/Sequencer.hpp>
#include <theater/XO2Shell.hpp>
#include <theater/ProcessMonitor.hpp>
#include <theater/SynchronizedClock.hpp>
#include <theater/GUITrajectory.hpp>
#include <fstream>
#include <sstream>

using namespace std;

using sfl::DiffDrive;
using sfl::Odometry;
using sfl::Goal;
using sfl::Multiscanner;
using sfl::DynamicWindow;
using sfl::BubbleBand;
using sfl::Pose;
using sfl::Polygon;

using expo::MotionController;
using expo::MotionPlanner;

using curve::BSpline;


extern "C" {
#include <sys/time.h>
}


//////////////////////////////////////////////////
// For lazy initialization of drawings and cameras.

class DWA:
  public proxy::DWA
{
public:
  DWA(TheaterRobot & owner): _owner(owner) {}
  int Dimension() { return _owner.GetDynamicWindow()->Dimension(); }
  bool Forbidden(int qdlIndex, int qdrIndex)
  { return _owner.GetDynamicWindow()->Forbidden(qdlIndex, qdrIndex); }
  bool Admissible(int qdlIndex, int qdrIndex)
  { return _owner.GetDynamicWindow()->Admissible(qdlIndex, qdrIndex); }
  bool Reachable(int qdlIndex, int qdrIndex)
  { return _owner.GetDynamicWindow()->Reachable(qdlIndex, qdrIndex); }
  int QdlMinIndex() { return _owner.GetDynamicWindow()->QdlMinIndex(); }
  int QdlMaxIndex() { return _owner.GetDynamicWindow()->QdlMaxIndex(); }
  int QdrMinIndex() { return _owner.GetDynamicWindow()->QdrMinIndex(); }
  int QdrMaxIndex() { return _owner.GetDynamicWindow()->QdrMaxIndex(); }
  int QdlOptIndex() { return _owner.GetDynamicWindow()->QdlOptIndex(); }
  int QdrOptIndex() { return _owner.GetDynamicWindow()->QdrOptIndex(); }
  double ObjectiveMax() { return _owner.GetDynamicWindow()->ObjectiveMax(); }
  double ObjectiveMin() { return _owner.GetDynamicWindow()->ObjectiveMin(); }
  double GetObjective(int qdlIndex, int qdrIndex)
  { return _owner.GetDynamicWindow()->GetObjective(qdlIndex, qdrIndex); }
  sfl::HeadingObjective & GetHeadingObjective()
  { return _owner.GetDynamicWindow()->GetHeadingObjective(); }
  double Qd(int index) { return _owner.GetDynamicWindow()->Qd(index); }
  void GetSubGoal(double & local_x, double & local_y)
  { return _owner.GetDynamicWindow()->GetSubGoal(local_x, local_y); }
private:
  TheaterRobot & _owner;
};


class MP:
  public proxy::MP
{
public:
  MP(TheaterRobot & owner): _owner(owner) {}
  const Goal & GetGoal() { return _owner.GetMotionPlanner()->GetGoal(); }
private:
  TheaterRobot & _owner;
};


class DistanceObjective:
  public proxy::Objective
{
public:
  DistanceObjective(TheaterRobot & owner): _owner(owner) {}
  double Value(unsigned int qdlIndex, unsigned int qdrIndex)
    throw(runtime_error)
  { return _owner.GetDynamicWindow()->GetDistanceObjective().Value(qdlIndex, qdrIndex); }
  double Min(unsigned int qdlMin, unsigned int qdlMax,
	     unsigned int qdrMin, unsigned int qdrMax)
  { return _owner.GetDynamicWindow()->GetDistanceObjective().Min(qdlMin, qdlMax, qdrMin, qdrMax); }
  double Max(unsigned int qdlMin, unsigned int qdlMax,
	     unsigned int qdrMin, unsigned int qdrMax)
  { return _owner.GetDynamicWindow()->GetDistanceObjective().Max(qdlMin, qdlMax, qdrMin, qdrMax); }
private:
  TheaterRobot & _owner;
};


class HeadingObjective:
  public proxy::Objective
{
public:
  HeadingObjective(TheaterRobot & owner): _owner(owner) {}
  double Value(unsigned int qdlIndex, unsigned int qdrIndex)
    throw(runtime_error)
  { return _owner.GetDynamicWindow()->GetHeadingObjective().Value(qdlIndex, qdrIndex); }
  double Min(unsigned int qdlMin, unsigned int qdlMax,
	     unsigned int qdrMin, unsigned int qdrMax)
  { return _owner.GetDynamicWindow()->GetHeadingObjective().Min(qdlMin, qdlMax, qdrMin, qdrMax); }
  double Max(unsigned int qdlMin, unsigned int qdlMax,
	     unsigned int qdrMin, unsigned int qdrMax)
  { return _owner.GetDynamicWindow()->GetHeadingObjective().Max(qdlMin, qdlMax, qdrMin, qdrMax); }
private:
  TheaterRobot & _owner;
};


class SpeedObjective:
  public proxy::Objective
{
public:
  SpeedObjective(TheaterRobot & owner): _owner(owner) {}
  double Value(unsigned int qdlIndex, unsigned int qdrIndex)
    throw(runtime_error)
  { return _owner.GetDynamicWindow()->GetSpeedObjective().Value(qdlIndex, qdrIndex); }
  double Min(unsigned int qdlMin, unsigned int qdlMax,
	     unsigned int qdrMin, unsigned int qdrMax)
  { return _owner.GetDynamicWindow()->GetSpeedObjective().Min(qdlMin, qdlMax, qdrMin, qdrMax); }
  double Max(unsigned int qdlMin, unsigned int qdlMax,
	     unsigned int qdrMin, unsigned int qdrMax)
  { return _owner.GetDynamicWindow()->GetSpeedObjective().Max(qdlMin, qdlMax, qdrMin, qdrMax); }
private:
  TheaterRobot & _owner;
};


class BSO:
  public proxy::Objective
{
public:
  BSO(TheaterRobot & owner): _owner(owner) {}
  double Value(unsigned int qdlIndex, unsigned int qdrIndex)
    throw(runtime_error)
  { return _owner.GetBSplineObjective()->Value(qdlIndex, qdrIndex); }
  double Min(unsigned int qdlMin, unsigned int qdlMax,
	     unsigned int qdrMin, unsigned int qdrMax)
  { return _owner.GetBSplineObjective()->Min(qdlMin, qdlMax, qdrMin, qdrMax); }
  double Max(unsigned int qdlMin, unsigned int qdlMax,
	     unsigned int qdrMin, unsigned int qdrMax)
  { return _owner.GetBSplineObjective()->Max(qdlMin, qdlMax, qdrMin, qdrMax); }
private:
  TheaterRobot & _owner;
};


class ConcreteDistanceObjective:
  public proxy::DistanceObjective
{
public:
  ConcreteDistanceObjective(TheaterRobot & owner): _owner(owner) {}
  int DimX() { return _owner.GetDynamicWindow()->GetDistanceObjective().DimX(); }
  int DimY() { return _owner.GetDynamicWindow()->GetDistanceObjective().DimY(); }
  bool CellOccupied(int ix, int iy) { return _owner.GetDynamicWindow()->GetDistanceObjective().CellOccupied(ix, iy); }
  double CollisionTime(int ix, int iy, int iqdl, int iqdr) { return _owner.GetDynamicWindow()->GetDistanceObjective().CollisionTime(ix, iy, iqdl, iqdr); }
  double FindXlength(int i) { return _owner.GetDynamicWindow()->GetDistanceObjective().FindXlength(i); }
  double FindYlength(int i) { return _owner.GetDynamicWindow()->GetDistanceObjective().FindYlength(i); }
  const sfl::Hull & GetHull() { return _owner.GetDynamicWindow()->GetDistanceObjective().GetHull(); }
  const sfl::Hull & GetPaddedHull() { return _owner.GetDynamicWindow()->GetDistanceObjective().GetPaddedHull(); }
  const sfl::Hull & GetEvaluationHull() { return _owner.GetDynamicWindow()->GetDistanceObjective().GetEvaluationHull(); }
private:
  TheaterRobot & _owner;
};


class RobotModel:
  public proxy::RobotModel
{
public:
  RobotModel(TheaterRobot & owner): _owner(owner) {}
  void Actuator2Global(double qdl, double qdr,
		       double & sd, double & thetad)
  { _owner.GetRobotModel()->Actuator2Global(qdl, qdr, sd, thetad); }
  double WheelBase() { return _owner.GetRobotModel()->WheelBase(); }
private:
  TheaterRobot & _owner;
};


class DWACamera:
  public Camera
{
public:
  DWACamera(const string & name,
	    TheaterRobot & owner):
    Camera(name, true),
    _owner(owner)
  {
  }
  
  void ConfigureView(View & view)
  {
    if(_cam.get() == 0){
      const double d(_owner.GetDynamicWindow()->Dimension());
      _cam = auto_ptr<StillCamera>(new StillCamera(Name(), 0, 0, d, d, false));
    }
    _cam->ConfigureView(view);
  }
  
private:
  TheaterRobot & _owner;
  auto_ptr<StillCamera> _cam;
};


class CollisionCamera:
  public Camera
{
public:
  CollisionCamera(const string & name,
		  TheaterRobot & owner):
    Camera(name, true),
    _owner(owner)
  {
  }
  
  void ConfigureView(View & view)
  {
    if(_cam.get() == 0){
      double a, b, c, d;
      _owner.GetDynamicWindow()->GetDistanceObjective().GetRange(a, b, c, d);
      _cam = auto_ptr<StillCamera>(new StillCamera(Name(), a, b, c, d, false));
    }
    _cam->ConfigureView(view);
  }
  
private:
  TheaterRobot & _owner;
  auto_ptr<StillCamera> _cam;
};


class Lazyfier:
  public expo::Lazyfier
{
public:
  Lazyfier(TheaterRobot & owner): _owner(owner) {}
  DynamicWindow * GetDynamicWindow() { return _owner.GetDynamicWindow(); }
  Multiscanner * GetMultiscanner() { return _owner.GetMultiscanner(); }
  const sfl::RobotModel * GetRobotModel() { return _owner.GetRobotModel(); }
  BubbleBand * GetBubbleBand() { return _owner.GetBubbleBand(); }
  Odometry * GetOdometry() { return _owner.GetOdometry(); }
  MotionController * GetMotionController() { return _owner.GetMotionController(); }
private:
  TheaterRobot & _owner;
};

//
//////////////////////////////////////////////////


TheaterRobot::
TheaterRobot(const string & name,
	     const World & world,
	     double timestep,
	     TheaterRobotParameters * parameters):
  Robot(name, world, timestep, true),
  _parameters(parameters),
  _alpha_distance(-1),
  _alpha_heading(-1),
  _alpha_speed(-1),
  _alpha_bspline(-1),
  _k_rho_bspline(-1),
  _lambda_rho_bspline(-1),
  _mu_rho_bspline(-1),
  _k_phi_bspline(-1),
  _lambda_phi_bspline(-1),
  _mu_phi_bspline(-1),
  _coarse_dwa(false)
{
  //////////////////////////////////////////////////
  // Non-lazy initialization necessary here because it implicitly
  // creates the robot's DiffDriveDrawing() and unfortunately the
  // drawings have to be registered at construction time.
  _drive = DefineDiffDrive(GetParameters()->WheelBase(),
			   GetParameters()->WheelRadius());
  _front = DefineLidar(GetParameters()->FrontLidarMount(), "front");
  _rear = DefineLidar(GetParameters()->RearLidarMount(), "rear");
  //////////////////////////////////////////////////
  
  AddCamera(new OCamera(Name() + "_ocamera",
			auto_ptr<proxy::DWA>(new DWA(* this))));
  AddCamera(new DWACamera(Name() + "_dwcamera", * this));
  AddCamera(new CollisionCamera(Name() + "_collisioncamera", * this));
  
  AddDrawing(new GoalDrawing(Name() + "_goaldrawing",
			     auto_ptr<proxy::MP>(new MP(* this))));
  AddDrawing(new DWDrawing(Name() + "_dwdrawing",
			   auto_ptr<proxy::DWA>(new DWA(* this))));  
  AddDrawing(new ODrawing(Name() + "_dodrawing",
			  auto_ptr<proxy::Objective>(new DistanceObjective(* this)),
			  auto_ptr<proxy::DWA>(new DWA(* this))));
  AddDrawing(new ODrawing(Name() + "_bsodrawing",
			  auto_ptr<proxy::Objective>(new BSO(* this)),
			  auto_ptr<proxy::DWA>(new DWA(* this))));
  AddDrawing(new ODrawing(Name() + "_hodrawing",
			  auto_ptr<proxy::Objective>(new HeadingObjective(* this)),
			  auto_ptr<proxy::DWA>(new DWA(* this))));
  AddDrawing(new ODrawing(Name() + "_sodrawing",
			  auto_ptr<proxy::Objective>(new SpeedObjective(* this)),
			  auto_ptr<proxy::DWA>(new DWA(* this))));
  AddDrawing(new
	     DODrawing(Name() + "_collisiondrawing",
		       auto_ptr<proxy::DistanceObjective>(new ConcreteDistanceObjective(* this)),
		       auto_ptr<proxy::DWA>(new DWA(* this)),
		       auto_ptr<proxy::RobotModel>(new RobotModel(* this))));
  AddDrawing(new SequencerDrawing::Stage(Name() + "_sequencer_stage", * this));
}


TheaterRobot * TheaterRobot::
CreateButler(const string & name,
	     const World & world,
	     double timestep)
{
  TheaterRobotParameters * parameters(TheaterRobotParameters::CreateButler());
  if(parameters == 0)
    return 0;
  return new TheaterRobot(name, world, timestep, parameters);
}


TheaterRobot * TheaterRobot::
CreateDancer(const string & name,
	     const World & world,
	     double timestep)
{
  TheaterRobotParameters * parameters(TheaterRobotParameters::CreateDancer());
  if(parameters == 0)
    return 0;
  return new TheaterRobot(name, world, timestep, parameters);
}


TheaterRobot * TheaterRobot::
CreateAnimal(const string & name,
	     const World & world,
	     double timestep)
{
  TheaterRobotParameters * parameters(TheaterRobotParameters::CreateAnimal());
  if(parameters == 0)
    return 0;
  return new TheaterRobot(name, world, timestep, parameters);
}


void TheaterRobot::
InitPose(double x,
	 double y,
	 double theta)
{
  GetOdometry()->Init(Pose(x, y, theta));
}


void TheaterRobot::
SetPose(double x,
	double y,
	double theta)
{
  GetOdometry()->Set(Pose(x, y, theta));
}



void TheaterRobot::
GetPose(double & x,
	double & y,
	double & theta)
{
  x = GetOdometry()->Get().X();
  y = GetOdometry()->Get().Y();
  theta = GetOdometry()->Get().Theta();
}



const Goal & TheaterRobot::
GetGoal()
{
  return GetMotionPlanner()->GetGoal();
}



bool TheaterRobot::
GoalReached()
{
  return GetMotionPlanner()->GoalReached();
}



void TheaterRobot::
SetGoal(const Goal & goal)
{
  GetMotionPlanner()->SetGoal(goal);
}



void TheaterRobot::
PrepareAction()
{
  cout << "\n\n\n**************************************************\n"
       << Name() << "\n";
  
  GetMutableClock()->Step();
  GetSequencer()->Step();
  GetMotionManager()->PrepareAction(GetClock()->GetNow(), cout);
  GetMotionController()->Update();
  GetOdometry()->Update();
}


TheaterRobot::HALProxy::
HALProxy(TheaterRobot & owner):
  _owner(owner)
{
}


/**
   \todo Unify sunflower-light Timestamp and HALProxy with theater
   project timestamps.
*/
int TheaterRobot::HALProxy::
hal_time_get(struct timespec & stamp)
{
  struct timeval tv;
  int res(gettimeofday( & tv, 0));
  if(res != 0)
    return -1;

  TIMEVAL_TO_TIMESPEC( & tv, & stamp);

  return 0;
}


int TheaterRobot::HALProxy::
hal_odometry_set(double x, double y, double theta,
		 double sxx, double syy, double stt,
		 double sxy, double sxt, double syt)
{
  _owner.SetTruePose(x, y, theta);
  return 0;
}


int TheaterRobot::HALProxy::
hal_odometry_get(struct timespec & stamp,
		 double & x, double & y, double & theta,
		 double & sxx, double & syy, double & stt,
		 double & sxy, double & sxt, double & syt)
{
  int res(hal_time_get(stamp));
  if(res != 0)
    return res;
  
  _owner.TruePose().Get(x, y, theta);
  sxx = 1;
  syy = 1;
  stt = 1;
  sxy = 0;
  sxt = 0;
  syt = 0;

  return 0;
}


int TheaterRobot::HALProxy::
hal_speed_set(double qdl,
	      double qdr)
{
  _owner.GetDrive()->SetSpeed(qdl, qdr);

  return 0;
}


bool TheaterRobot::
Init(ostream & os)
{
  { // dwa_mode
    string val(GetOption("dwa_mode"));
    if(val == "coarse")
      _coarse_dwa = true;
  }
  
  { // alpha_distance
    istringstream is(GetOption("alpha_distance"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"alpha_distance\".\n";
	return false;
      }
      _alpha_distance = val;
    }
  }
  
  { // alpha_heading
    istringstream is(GetOption("alpha_heading"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"alpha_heading\".\n";
	return false;
      }
      _alpha_heading = val;
    }
  }
  
  { // alpha_speed
    istringstream is(GetOption("alpha_speed"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"alpha_speed\".\n";
	return false;
      }
      _alpha_speed = val;
    }
  }
  
  { // alpha_bspline
    istringstream is(GetOption("alpha_bspline"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"alpha_bspline\".\n";
	return false;
      }
      _alpha_bspline = val;
    }
  }
  
  { // k_rho_bspline
    istringstream is(GetOption("k_rho_bspline"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"k_rho_bspline\".\n";
	return false;
      }
      _k_rho_bspline = val;
    }
  }
  
  { // lambda_rho_bspline
    istringstream is(GetOption("lambda_rho_bspline"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"lambda_rho_bspline\".\n";
	return false;
      }
      _lambda_rho_bspline = val;
    }
  }
  
  { // mu_rho_bspline
    istringstream is(GetOption("mu_rho_bspline"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"mu_rho_bspline\".\n";
	return false;
      }
      _mu_rho_bspline = val;
    }
  }
  
  { // k_phi_bspline
    istringstream is(GetOption("k_phi_bspline"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"k_phi_bspline\".\n";
	return false;
      }
      _k_phi_bspline = val;
    }
  }
  
  { // lambda_phi_bspline
    istringstream is(GetOption("lambda_phi_bspline"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"lambda_phi_bspline\".\n";
	return false;
      }
      _lambda_phi_bspline = val;
    }
  }
  
  { // mu_phi_bspline
    istringstream is(GetOption("mu_phi_bspline"));
    if(is.str() != ""){
      double val;
      is >> val;
      if( ! is){
	os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
	   << "  Value (double) expected for option \"mu_phi_bspline\".\n";
	return false;
      }
      _mu_phi_bspline = val;
    }
  }
  
  string batchfilename(GetOption("BatchFilename"));
  if(batchfilename == ""){
    os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
       << "  Option \"BatchFilename\" must be set\n";
    return false;
  }

  ifstream batchfile(batchfilename.c_str());
  if( ! batchfile){
    os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
       << "  Problems opening batch file \"" << batchfilename << "\"\n";
    return false;
  }

  if( ! GetXO2Shell()->ParseBatch(batchfile, os)){
    os << "ERROR in TheaterRobot::Init(): " << Name() << "\n"
       << "  Problems executing batch file \"" << batchfilename << "\"\n";
    return false;
  }

  return true;
}


// void TheaterRobot::
// HackedBSplineStuff(BSpline * bspline)
// {
//   GetBSplineObjective()->SetBSpline(bspline);
// }


const SynchronizedClock * TheaterRobot::
GetClock()
{
  return GetMutableClock();
}


SynchronizedClock * TheaterRobot::
GetMutableClock()
{
  if(_clock.get() == 0)
    _clock =
      auto_ptr<SynchronizedClock>(new SynchronizedClock(* GetMonitor(),
							Timestep()));
  return _clock.get();
}


TheaterMotionManager * TheaterRobot::
GetMotionManager()
{
  if(_motion_manager.get() == 0)
    _motion_manager =
      auto_ptr<TheaterMotionManager>
      (new TheaterMotionManager(* GetMotionPlanner(),
				* GetDynamicWindow(),
				* GetMultiscanner(),
				* GetMotionController(),
				* GetOdometry(),
				* GetMutableRobotModel()));
  return _motion_manager.get();
}


ProcessMonitor * TheaterRobot::
GetMonitor()
{
  if(_monitor.get() == 0)
    _monitor = auto_ptr<ProcessMonitor>(new ProcessMonitor());
  return _monitor.get();
}


Odometry * TheaterRobot::
GetOdometry()
{
  if(_odometry.get() == 0){
    _odometry = auto_ptr<Odometry>(new Odometry(* GetHALProxy()));
    AddDrawing(new OdometryDrawing(Name() + "_odomdrawing",
				   * _odometry.get(),
				   GetRobotModel()->WheelBase() / 2));
  }
  return _odometry.get();
}


MotionController * TheaterRobot::
GetMotionController()
{
  if(_motionController.get() == 0) 
    _motionController =
      auto_ptr<MotionController>(new MotionController(Name(),
						      * GetRobotModel(),
						      * GetDrive(),
						      * GetHALProxy()));
  return _motionController.get();
}


const sfl::RobotModel * TheaterRobot::
GetRobotModel()
{
  return GetMutableRobotModel();
}


sfl::RobotModel * TheaterRobot::
GetMutableRobotModel()
{
  if(_robotModel.get() == 0){
    _robotModel =
      auto_ptr<sfl::RobotModel>(new sunflower::RobotModel(Timestep(), * GetParameters()));
    const Polygon & outline(_robotModel->GetOutline());
    for(int il(0); il < outline.GetNLines(); ++il)
      AddLine(outline.GetLine(il));
  }
  return _robotModel.get();
}


ostream * TheaterRobot::
GetOutstream()
{
  return & cout;
}


TheaterRobot::HALProxy * TheaterRobot::
GetHALProxy()
{
  if(_hal_proxy.get() == 0) 
    _hal_proxy = auto_ptr<HALProxy>(new HALProxy( * this));
  return _hal_proxy.get();
}


Lidar * TheaterRobot::
GetFrontLidar()
{
  return _front;
}


Lidar * TheaterRobot::
GetRearLidar()
{
  return _rear;
}


DiffDrive * TheaterRobot::
GetDrive()
{
  // Don't use lazy initialization here, has been done in ctor, see
  // comments there.
  return _drive;
}


DynamicWindow * TheaterRobot::
GetDynamicWindow()
{
  if(_dynamicWindow.get() == 0){
    if(_coarse_dwa)
      _dynamicWindow =
	auto_ptr<DynamicWindow>(new DynamicWindow(11,
						  3.6,
						  2.2,
						  0.3,
						  * GetRobotModel(),
						  * GetMotionController(),
						  0.1,
						  0.5,
						  0.4));
    else
      _dynamicWindow =
	auto_ptr<DynamicWindow>(new DynamicWindow(31,
						  3.6,
						  2.2,
						  0.1,
						  * GetRobotModel(),
						  * GetMotionController(),
						  0.1,
						  0.5,
						  0.4));
      
    _dynamicWindow->AddCustomObjective(GetBSplineObjective());

    // Attention, this results in recursive calls, which is OK because
    // of the if() at the beginning of this method... but it's a bit
    // ugly, would be nicer to pass the necessary pointers into the
    // LazyDWAInit() method.
    LazyDWAInit();
  }
  return _dynamicWindow.get();
}


BubbleBand * TheaterRobot::
GetBubbleBand()
{
  if(_bubbleBand.get() == 0) 
    _bubbleBand =
      auto_ptr<BubbleBand>(new BubbleBand(* GetRobotModel(), * GetOdometry()));
  return _bubbleBand.get();
}


Multiscanner * TheaterRobot::
GetMultiscanner()
{
  if(_multiscanner.get() == 0){
    _multiscanner = auto_ptr<Multiscanner>(new Multiscanner());
    _multiscanner->Add( & GetFrontLidar()->GetProxy());
    _multiscanner->Add( & GetRearLidar()->GetProxy());
  }
  return _multiscanner.get();
}


MotionPlanner * TheaterRobot::
GetMotionPlanner()
{
  if(_motionPlanner.get() == 0)
    _motionPlanner =
      auto_ptr<MotionPlanner>(new MotionPlanner(* GetLazyfier()));
  return _motionPlanner.get();
}


expo::Lazyfier * TheaterRobot::
GetLazyfier()
{
  if(_lazyfier.get() == 0)
    _lazyfier = auto_ptr<expo::Lazyfier>(new Lazyfier(* this));
  return _lazyfier.get();
}


TheaterRobotProxy * TheaterRobot::
GetRobotProxy()
{
  if(_robot_proxy.get() == 0) 
    _robot_proxy = auto_ptr<TheaterRobotProxy>(new TheaterRobotProxy(* this));
  return _robot_proxy.get();
}


Sequencer * TheaterRobot::
GetSequencer()
{
  if(_sequencer.get() == 0) 
    _sequencer = auto_ptr<Sequencer>(new Sequencer(* GetRobotProxy()));
  return _sequencer.get();
}


XO2Shell * TheaterRobot::
GetXO2Shell()
{
  if(_xo2_shell.get() == 0){
    _xo2_shell = auto_ptr<XO2Shell>(new XO2Shell());
    if( ! _xo2_shell->AddModule(GetSequencer())){
      cerr << "ERROR in TheaterRobot::TheaterRobot(): \"" << Name() << "\"\n"
	   << "  _xo2_shell->AddModule(GetSequencer()) failed\n";
      exit(EXIT_FAILURE);			// Yes, this is ugly.
    }
    if( ! _xo2_shell->AddModule(GetMutableClock()->GetXO2Module())){
      cerr << "ERROR in TheaterRobot::TheaterRobot(): \"" << Name() << "\"\n"
	   << "  _xo2_shell->AddModule(GetClock()->GetXO2Module()) failed\n";
      exit(EXIT_FAILURE);			// Yes, this is ugly.
    }
    if( ! _xo2_shell->AddModule(GUI::Trajectory::GetInstance())){
      cerr << "ERROR in TheaterRobot::TheaterRobot(): \"" << Name() << "\"\n"
	   << "  _xo2_shell->AddModule(GUI::Trajectory::GetInstance()) failed\n";
      exit(EXIT_FAILURE);			// Yes, this is ugly.
    }
  }
  return _xo2_shell.get();
}


BSplineObjective * TheaterRobot::
GetBSplineObjective()
{
  if(_bspline_objective.get() == 0) 
    _bspline_objective =
      auto_ptr<BSplineObjective>(new BSplineObjective(* GetDynamicWindow(),
						      0.5,
						      * GetRobotProxy()));
  return _bspline_objective.get();
}


TheaterRobotParameters * TheaterRobot::
GetParameters()
{
  return _parameters.get();
}


void TheaterRobot::
LazyDWAInit()
{
  if(_alpha_distance >= 0)
    GetDynamicWindow()->GetDistanceObjective().SetAlpha(_alpha_distance);
  if(_alpha_heading >= 0){
    GetDynamicWindow()->GetHeadingObjective().SetAlpha(_alpha_heading);
    GetMotionManager()->
      HackedAlphaHeading( & GetDynamicWindow()->GetHeadingObjective(),
			  _alpha_heading);
  }
  if(_alpha_speed >= 0)
    GetDynamicWindow()->GetSpeedObjective().SetAlpha(_alpha_speed);
  if(_alpha_bspline >= 0){
    GetBSplineObjective()->SetAlpha(_alpha_bspline);
    GetMotionManager()->HackedAlphaBSpline(GetBSplineObjective(),
					   _alpha_bspline);
  }
  if(_k_rho_bspline >= 0)
    GetBSplineObjective()->SetKRho(_k_rho_bspline);
  if(_lambda_rho_bspline >= 0)
    GetBSplineObjective()->SetLambdaRho(_lambda_rho_bspline);
  if(_mu_rho_bspline >= 0)
    GetBSplineObjective()->SetMuRho(_mu_rho_bspline);
  if(_k_phi_bspline >= 0)
    GetBSplineObjective()->SetKPhi(_k_phi_bspline);
  if(_lambda_phi_bspline >= 0)
    GetBSplineObjective()->SetLambdaPhi(_lambda_phi_bspline);
  if(_mu_phi_bspline >= 0)
    GetBSplineObjective()->SetMuPhi(_mu_phi_bspline);
}

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


#ifndef THEATER_ROBOT_HPP
#define THEATER_ROBOT_HPP


#include <npm/common/Robot.hpp>
#include <sfl/api/HAL.hpp>


namespace sfl {
  class Odometry;
  class Goal;
  class Multiscanner;
  class RobotModel;
  class DynamicWindow;
  class BubbleBand;
}

namespace expo {
  class MotionController;
  class MotionPlanner;
  class Lazyfier;
}

namespace curve {
  class BSpline;
}

namespace npm {
  class DiffDrive;
  class Lidar;
}

class TheaterRobotParameters;
class TheaterRobotProxy;
class TheaterMotionManager;

class Sequencer;
class XO2Shell;
class ProcessMonitor;
class BSplineObjective;
class SynchronizedClock;


class TheaterRobot
  : public npm::Robot
{
private:
  TheaterRobot(const std::string & name,
	       const npm::World & world,
	       double timestep,
	       /** \note transfers ownership */
	       TheaterRobotParameters * parameters);


public:
  static TheaterRobot * CreateButler(const std::string & name,
				     const npm::World & world,
				     double timestep);
  static TheaterRobot * CreateDancer(const std::string & name,
				     const npm::World & world,
				     double timestep);
  static TheaterRobot * CreateAnimal(const std::string & name,
				     const npm::World & world,
				     double timestep);

  
  void PrepareAction();
  void InitPose(double x, double y, double theta);
  void SetPose(double x, double y, double theta);
  void GetPose(double & x, double & y, double & theta);
  void SetGoal(const sfl::Goal & goal);
  boost::shared_ptr<const sfl::Goal> GetGoal();
  bool GoalReached();
  bool Init(std::ostream & os);

  ////  void HackedBSplineStuff(curve::BSpline * bspline);
  


  //////////////////////////////////////////////////
  // Accessors for RobotProxy, also implement lazy initialization.
  const SynchronizedClock * GetClock();
  TheaterMotionManager * GetMotionManager();
  ProcessMonitor * GetMonitor();
  sfl::Odometry * GetOdometry();
  expo::MotionController * GetMotionController();
  const sfl::RobotModel * GetRobotModel();
  std::ostream * GetOutstream();
  //////////////////////////////////////////////////
  // Added for cleaner lazy initialization.
  TheaterRobotParameters * GetParameters();
  sfl::DynamicWindow * GetDynamicWindow();
  sfl::BubbleBand * GetBubbleBand();
  sfl::Multiscanner * GetMultiscanner();
  expo::MotionPlanner * GetMotionPlanner();
  expo::Lazyfier * GetLazyfier();
  TheaterRobotProxy * GetRobotProxy();
  Sequencer * GetSequencer();
  XO2Shell * GetXO2Shell();
  BSplineObjective * GetBSplineObjective();
  //////////////////////////////////////////////////
  // Added for update loop and other hacks.
  SynchronizedClock * GetMutableClock();
  sfl::RobotModel * GetMutableRobotModel();
  //////////////////////////////////////////////////
  // NOT for lazy initialization.
  npm::Lidar * GetFrontLidar();
  npm::Lidar * GetRearLidar();
  npm::DiffDrive * GetDrive();
  //////////////////////////////////////////////////
  
protected:
  class HALProxy
    : public sfl::HAL
  {
  public:
    HALProxy(TheaterRobot & owner);
    
    virtual int hal_time_get(struct timespec & stamp);
    virtual int hal_odometry_set(double x, double y, double theta,
				 double sxx, double syy, double stt,
				 double sxy, double sxt, double syt);
    virtual int hal_odometry_get(struct timespec & stamp,
				 double & x, double & y, double & theta,
				 double & sxx, double & syy, double & stt,
				 double & sxy, double & sxt, double & syt);
    virtual int hal_speed_set(double qdl, double qdr);

  private:
    TheaterRobot & _owner;
  };

  HALProxy * GetHALProxy();
  
  
private:
  std::auto_ptr<HALProxy> _hal_proxy;
  std::auto_ptr<TheaterRobotParameters> _parameters;
  
  npm::Lidar * _front;		// deleted by superclass
  npm::Lidar * _rear;		// deleted by superclass
  npm::DiffDrive * _drive; // deleted by superclass
  
  std::auto_ptr<sfl::RobotModel> _robotModel;
  std::auto_ptr<expo::MotionController> _motionController;
  std::auto_ptr<sfl::DynamicWindow> _dynamicWindow;
  std::auto_ptr<sfl::Odometry> _odometry;
  std::auto_ptr<sfl::BubbleBand> _bubbleBand;
  std::auto_ptr<sfl::Multiscanner> _multiscanner;
  std::auto_ptr<expo::MotionPlanner> _motionPlanner;
  std::auto_ptr<expo::Lazyfier> _lazyfier;
  
  std::auto_ptr<TheaterMotionManager> _motion_manager;
  std::auto_ptr<ProcessMonitor> _monitor;
  std::auto_ptr<SynchronizedClock> _clock;
  std::auto_ptr<TheaterRobotProxy> _robot_proxy;
  std::auto_ptr<Sequencer> _sequencer;
  std::auto_ptr<XO2Shell> _xo2_shell;

  std::auto_ptr<BSplineObjective> _bspline_objective;

  //////////////////////////////////////////////////
  // Slightly hacked approach to even lazier DWA initialization: Set
  // alphas to -1, and if they're overridden by an option, do the
  // corresponding magic in LazyDWAInit().

  void LazyDWAInit();

  double _alpha_distance, _alpha_heading, _alpha_speed, _alpha_bspline;
  double _k_rho_bspline, _lambda_rho_bspline, _mu_rho_bspline;
  double _k_phi_bspline, _lambda_phi_bspline, _mu_phi_bspline;

  //////////////////////////////////////////////////

  bool _coarse_dwa;
};

#endif // THEATER_ROBOT_HPP

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


#include "RobotModel.hpp"
#include <iostream>
#include <cmath>


using boost::shared_ptr;


namespace sfl {


  RobotModel::Parameters::
  Parameters(double _safetyDistance,
	     double _wheelBase,
	     double _wheelRadius,
	     double _qdMax,
	     double _qddMax,
	     double _sdMax,
	     double _thetadMax,
	     double _sddMax,
	     double _thetaddMax)
    : safetyDistance(_safetyDistance),
      wheelBase(_wheelBase),
      wheelRadius(_wheelRadius),
      qdMax(_qdMax),
      qddMax(_qddMax),
      sdMax(_sdMax),
      thetadMax(_thetadMax),
      sddMax(_sddMax),
      thetaddMax(_thetaddMax)
  {
  }
  
  
  RobotModel::
  RobotModel(Parameters parameters,
	     shared_ptr<const Hull> hull):
    m_params(parameters),
    m_hull(hull),
    m_safety_hull(hull->CreateGrownHull(parameters.safetyDistance)),
    m_robot_radius(-1)		// lazy init
  {
  }
  
  
  
  shared_ptr<const Hull> RobotModel::
  GetHull()
    const
  {
    return m_hull;
  }



  double RobotModel::
  SecurityDistance()
    const
  {
    return m_params.safetyDistance;
  }



  double RobotModel::
  WheelBase()
    const
  {
    return m_params.wheelBase;
  }



  double RobotModel::
  WheelRadius()
    const
  {
    return m_params.wheelRadius;
  }



  double RobotModel::
  QdMax()
    const
  {
    return m_params.qdMax;
  }



  double RobotModel::
  QddMax()
    const
  {
    return m_params.qddMax;
  }



  double RobotModel::
  SdMax()
    const
  {
    return m_params.sdMax;
  }



  double RobotModel::
  ThetadMax()
    const
  {
    return m_params.thetadMax;
  }



  double RobotModel::
  SddMax()
    const
  {
    return m_params.sddMax;
  }



  double RobotModel::
  ThetaddMax()
    const
  {
    return m_params.thetaddMax;
  }



  void RobotModel::
  Actuator2Global(double qdl, double qdr,
		  double & sd, double & thetad)
    const
  {
    Actuator2Global(qdl, qdr, m_params.wheelBase, m_params.wheelRadius,
		    sd, thetad);
  }
  
  
  
  void RobotModel::
  Actuator2Global(double qdl, double qdr,
		  double wheelBase, double wheelRadius,
		  double & sd, double & thetad)
  {
    qdl *= wheelRadius;
    qdr *= wheelRadius;
    sd = 0.5 * (qdl + qdr);
    thetad = (qdr - qdl) / wheelBase;
  }



  void RobotModel::
  Global2Actuator(double sd, double thetad,
		  double & qdl, double & qdr)
    const
  {
    Global2Actuator(sd, thetad, m_params.wheelBase, m_params.wheelRadius,
		    qdl, qdr);
  }



  void RobotModel::
  Global2Actuator(double sd, double thetad,
		  double wheelBase, double wheelRadius,
		  double & qdl, double & qdr)
  {
    thetad *= 0.5 * wheelBase; 
    qdr = (sd + thetad) / wheelRadius;
    qdl = (sd - thetad) / wheelRadius;
  }



  void RobotModel::
  LocalKinematics(double sd, double thetad,
		  double timestep,
		  double & deltax,
		  double & deltay,
		  double & deltatheta)
  {
    // ATTENTION: In here, we use timestep and not m_timestep. The
    // former is given as parameter to this method in order to allow
    // callers to use a timestep different than the one configured in
    // their RobotModel instance.

    deltatheta = thetad * timestep;

    double R;			// radius of curvature

    if(absval(deltatheta) > epsilon){
      // use circular movement
      R = sd / thetad;
      deltax = R * sin(deltatheta);
      deltay = R * (1 - cos(deltatheta));
    }
    else{
      // approximate with linear movement
      R = sd * timestep;
      deltax = R * cos(0.5 * deltatheta);
      deltay = R * sin(0.5 * deltatheta);
    }
  }
  
  
  boost::shared_ptr<const Hull> RobotModel::
  GetSafetyHull() const
  {
    return m_safety_hull;
  }
  
  
  void RobotModel::
  PredictStandstillAct(double qdl, double qdr, double safety_delay,
		       double & dx, double & dy, double & dtheta) const
  {
    double sd, thetad;
    Actuator2Global(qdl, qdr, sd, thetad);
    double stoptime;
    if(absval(qdl) > absval(qdr))
      stoptime = absval(qdl) / QddMax();
    else
      stoptime = absval(qdr) / QddMax();
    stoptime += safety_delay;
    LocalKinematics(sd, thetad, stoptime, dx, dy, dtheta);
  }
  
  
  void RobotModel::
  PredictStandstillGlob(double sd, double thetad, double safety_delay,
			double & dx, double & dy, double & dtheta) const
  {
    double qdl, qdr;
    Global2Actuator(sd, thetad, qdl, qdr);
    double stoptime;
    if(absval(qdl) > absval(qdr))
      stoptime = absval(qdl) / QddMax();
    else
      stoptime = absval(qdr) / QddMax();
    stoptime += safety_delay;
    LocalKinematics(sd, thetad, stoptime, dx, dy, dtheta);
  }
  
  
  void RobotModel::
  ApplyActuatorLimits(double timestep,
		      double qdl_cur, double qdr_cur,
		      double qdl_des, double qdr_des,
		      double & qdl_cmd, double & qdr_cmd,
		      std::ostream * verbose_os)
  {
    // limit in actuator speed space
    const double dqd(timestep * m_params.qddMax);
    const double qdl_max(boundval(- m_params.qdMax, qdl_cur + dqd, m_params.qdMax));
    const double qdl_min(boundval(- m_params.qdMax, qdl_cur - dqd, m_params.qdMax));
    const double qdr_max(boundval(- m_params.qdMax, qdr_cur + dqd, m_params.qdMax));
    const double qdr_min(boundval(- m_params.qdMax, qdr_cur - dqd, m_params.qdMax));
    const double pqdl(boundval(qdl_min, qdl_des, qdl_max));
    const double pqdr(boundval(qdr_min, qdr_des, qdr_max));
    
    // limit in global speed space
    double sd, thetad;
    Actuator2Global(pqdl, pqdr, sd, thetad);
    sd =     boundval( - m_params.sdMax,     sd,     m_params.sdMax);
    thetad = boundval( - m_params.thetadMax, thetad, m_params.thetadMax);
    
    // transform back to actuator space
    Global2Actuator(sd, thetad, qdl_cmd, qdr_cmd);
    
    if (verbose_os)
      *verbose_os << "RobotModel::ApplyActuatorLimits()\n"
		  << "  timestep: " << timestep << "\n"
		  << "  current: (" << qdl_cur << ", " << qdr_cur << ")\n"
		  << "  desired: (" << qdl_des << ", " << qdr_des << ")\n"
		  << "  bound:   (" << qdl_cmd << ", " << qdr_cmd << ")\n";
  }
  
  
  double RobotModel::
  RobotRadius() const
  {
    if (0 > m_robot_radius)
      m_robot_radius = m_hull->CalculateRadius();
    return m_robot_radius;
  }
  
}

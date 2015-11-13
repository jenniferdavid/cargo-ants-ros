/* 
 * Copyright (C) 2005
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


#include "BSplineObjective.hpp"
#include "TheaterRobotProxy.hpp"
#include <theater/SynchronizedClock.hpp>
#include <curve/BSpline.hpp>
#include <common/numeric.hpp>
#include <sfl/api/MotionController.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/dwa/DynamicWindow.hpp>

using namespace curve;


BSplineObjective::
BSplineObjective(const sfl::DynamicWindow & dynamic_window,
		 double alpha,
		 const TheaterRobotProxy & proxy):
  sfl::Objective(dynamic_window, alpha),
  _proxy(proxy),
  _spline(0),
  _k_rho(0.5),
  _lambda_rho(1),
  _mu_rho(1),
  _k_phi(0.5),
  _lambda_phi(M_PI / 2),
  _mu_phi(1)
{
}


void BSplineObjective::
Initialize(std::ostream * progress_stream)
{
}


void BSplineObjective::
Calculate(unsigned int qdlMin,
	  unsigned int qdlMax,
	  unsigned int qdrMin,
	  unsigned int qdrMax)
{
  if(_spline == 0){
    _proxy.GetOutstream()
      << "DEBUG BSplineObjective::Calculate(): _spline == 0\n";
    for(unsigned int l = qdlMin; l <= qdlMax; ++l)
      for(unsigned int r = qdrMin; r <= qdrMax; ++r)
	_value[l][r] = _minValue;
    return;
  }

  // determine wanted x, y, theta and speed
  const double now(_proxy.GetClock().GetNow());
  if(now < _domain_start){
    _proxy.GetOutstream()
      << "DEBUG BSplineObjective::Calculate(): now < _domain_start\n";
    HandleDomainStart(qdlMin, qdlMax, qdrMin, qdrMax);
    return;
  }
  else if(now > _domain_end){
    _proxy.GetOutstream()
      << "DEBUG BSplineObjective::Calculate(): now > _domain_start\n";
    HandleDomainEnd(qdlMin, qdlMax, qdrMin, qdrMax);
    return;
  }
  
  Point pos;
  _spline->CalculatePoint(now, pos);
  double wanted_x(pos.x);
  double wanted_y(pos.y);
  
  Point vel;
  _spline->CalculateSpeed(now, vel);
  const double wanted_sd(sqrt(sqr(vel.x) + sqr(vel.y)));
  
  double wanted_theta;
  if(wanted_sd < epsilon){
    Point tangent;
    _spline->CalculateTangent(now, tangent);
    wanted_theta = atan2(tangent.y, tangent.x);
  }
  else
    wanted_theta = atan2(vel.y, vel.x);
  
  // transform to local frame
  _proxy.GetOdometry().Get().From(wanted_x, wanted_y, wanted_theta);
  wanted_theta = mod2pi(wanted_theta);

  _proxy.GetOutstream()
    << "DEBUG BSplineObjective::Calculate():\n"
    << "  wanted_x     = " << wanted_x << "\n"
    << "  wanted_y     = " << wanted_y << "\n"
    << "  wanted_theta = " << wanted_theta << "\n"
    << "  wanted_sd    = " << wanted_sd << "\n";

  // get current motion state
  double current_qdl, current_qdr;
  _proxy.GetMotionController().GetActuators(current_qdl, current_qdr);
  
  // calculate values
  for(unsigned int l = qdlMin; l <= qdlMax; ++l){
    const double qdl(_dynamic_window.Qd(l));
    for(unsigned int r = qdrMin; r <= qdrMax; ++r)
      if(_dynamic_window.Admissible(l, r))
	_value[l][r] = CalculateValue(now,
				      wanted_x, wanted_y, wanted_theta,
				      wanted_sd,
				      qdl, _dynamic_window.Qd(r),
				      current_qdl, current_qdr);
      else
	_value[l][r] = _minValue;
  }
}


void BSplineObjective::
SetBSpline(curve::BSpline * spline)
{
  _spline = spline;
  if(spline == 0)
    return;
  
  _domain_start = spline->GetDomainStart();
  _domain_end = spline->GetDomainEnd();
}


double BSplineObjective::
CalculateValue(double now,
	       double wanted_x,
	       double wanted_y,
	       double wanted_theta,
	       double wanted_sd,
	       double candidate_qdl,
	       double candidate_qdr,
	       double current_qdl,
	       double current_qdr)
  const
{
  double predicted_x, predicted_y, predicted_theta, duration;
  PredictPose(wanted_sd,
	      candidate_qdl, candidate_qdr,
	      current_qdl, current_qdr,
	      predicted_x, predicted_y, predicted_theta,
	      duration);

#define APPROACH 1

#if APPROACH == 0

  // take difference between predicted and wanted as basis
  const double rho(sqrt(sqr(predicted_x) + sqr(predicted_y)));
  const double current_theta(_proxy.GetOdometry().Get().Theta());
  const double phi(absval(predicted_theta + current_theta - wanted_theta));
  
#elif APPROACH == 1

  now += duration;
  if(now > _domain_end)
    return _minValue;
  
  Point pos;
  _spline->CalculatePoint(now, pos);
  double projected_x(pos.x);
  double projected_y(pos.y);
  
  Point vel;
  _spline->CalculateSpeed(now, vel);
  const double projected_sd(sqrt(sqr(vel.x) + sqr(vel.y)));
  
  double projected_theta;
  if(projected_sd < epsilon){
    Point tangent;
    _spline->CalculateTangent(now, tangent);
    projected_theta = atan2(tangent.y, tangent.x);
  }
  else
    projected_theta = atan2(vel.y, vel.x);
  
  _proxy.GetOdometry().Get().From(projected_x, projected_y, projected_theta);
  projected_theta = mod2pi(projected_theta);

  // take difference between wanted and projected as basis
  const double rho(sqrt(sqr(predicted_x - projected_x)
			+ sqr(predicted_y - projected_y)));
  double phi(predicted_theta - projected_theta);
  phi = absval(mod2pi(phi));
  
#endif // APPROACH

  const double
    raw_value(_k_rho * (_lambda_rho / (_mu_rho * rho + _lambda_rho))
	      + _k_phi * (_lambda_phi / (_mu_phi * phi + _lambda_phi)));
  
  return _minValue + (_maxValue - _minValue) * raw_value;
}


void BSplineObjective::
PredictPose(double wanted_sd,
	    double candidate_qdl,
	    double candidate_qdr,
	    double current_qdl,
	    double current_qdr,
	    double & local_x,
	    double & local_y,
	    double & local_theta,
	    double & duration)
  const
{
  // determine candidate global speed
  double candidate_sd, candidate_thetad;
  _proxy.GetRobotModel().Actuator2Global(candidate_qdl, candidate_qdr,
					 candidate_sd, candidate_thetad);
  
  // determine time for moving one timestep at candidate (qdl, qdr)
  // followed by acceleration or deceleration until wanted sd.
  const double delta_qdl(absval(candidate_qdl - current_qdl));
  const double delta_qdr(absval(candidate_qdr - current_qdr));
  if(delta_qdl > delta_qdr)
    duration = delta_qdl / _proxy.GetRobotModel().QddMax();
  else
    duration = delta_qdr / _proxy.GetRobotModel().QddMax();
  duration += _proxy.GetRobotModel().Timestep();
  
  // determine pose when reaching wanted_sd
  _proxy.GetRobotModel().LocalKinematics(candidate_sd, candidate_thetad,
					 duration,
					 local_x, local_y, local_theta);
}


/**
   \todo Just a simplistic hack!
*/
void BSplineObjective::
HandleDomainStart(unsigned int qdlMin,
		  unsigned int qdlMax,
		  unsigned int qdrMin,
		  unsigned int qdrMax)
{
  for(unsigned int l = qdlMin; l <= qdlMax; ++l)
    for(unsigned int r = qdrMin; r <= qdrMax; ++r)
      _value[l][r] = _minValue;
}


/**
   \todo Just a simplistic hack!
*/
void BSplineObjective::
HandleDomainEnd(unsigned int qdlMin,
		unsigned int qdlMax,
		unsigned int qdrMin,
		unsigned int qdrMax)
{
  HandleDomainStart(qdlMin, qdlMax, qdrMin, qdrMax);
}

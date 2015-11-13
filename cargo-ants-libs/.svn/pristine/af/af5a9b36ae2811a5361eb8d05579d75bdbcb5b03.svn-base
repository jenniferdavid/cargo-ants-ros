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


#ifndef BSPLINE_OBJECTIVE_HPP
#define BSPLINE_OBJECTIVE_HPP


#include <sfl/dwa/Objective.hpp>


class TheaterRobotProxy;


namespace curve {
  class BSpline;
}


class BSplineObjective:
  public sfl::Objective
{
public:
  BSplineObjective(const sfl::DynamicWindow & dynamic_window,
		   double alpha,
		   const TheaterRobotProxy & proxy);
  

  virtual void Initialize(std::ostream * progress_stream);
  virtual void Calculate(unsigned int qdlMin,
			 unsigned int qdlMax,
			 unsigned int qdrMin,
			 unsigned int qdrMax);
  
  void SetBSpline(curve::BSpline * spline);

  inline void SetKRho(double val);
  inline void SetLambdaRho(double val);
  inline void SetMuRho(double val);
  inline void SetKPhi(double val);
  inline void SetLambdaPhi(double val);
  inline void SetMuPhi(double val);


protected:
  /**
     \note all coordinates in local frame
  */
  double CalculateValue(double now,
			double wanted_x,
			double wanted_y,
			double wanted_theta,
			double wanted_sd,
			double candidate_qdl,
			double candidate_qdr,
			double current_qdl,
			double current_qdr) const;
  
  void PredictPose(double wanted_sd,
		   double candidate_qdl,
		   double candidate_qdr,
		   double current_qdl,
		   double current_qdr,
		   double & local_x,
		   double & local_y,
		   double & local_theta,
		   double & duration) const;
  
  void HandleDomainStart(unsigned int qdlMin,
			 unsigned int qdlMax,
			 unsigned int qdrMin,
			 unsigned int qdrMax);
  void HandleDomainEnd(unsigned int qdlMin,
		       unsigned int qdlMax,
		       unsigned int qdrMin,
		       unsigned int qdrMax);
  
  const TheaterRobotProxy & _proxy;
  
  curve::BSpline * _spline;
  double _domain_start;
  double _domain_end;

  double _k_rho;
  double _lambda_rho;
  double _mu_rho;
  double _k_phi;
  double _lambda_phi;
  double _mu_phi;
};


void BSplineObjective::
SetKRho(double val)
{
  _k_rho = val;
}


void BSplineObjective::
SetLambdaRho(double val)
{
  _lambda_rho = val;
}


void BSplineObjective::
SetMuRho(double val)
{
  _mu_rho = val;
}


void BSplineObjective::
SetKPhi(double val)
{
  _k_phi = val;
}


void BSplineObjective::
SetLambdaPhi(double val)
{
  _lambda_phi = val;
}


void BSplineObjective::
SetMuPhi(double val)
{
  _mu_phi = val;
}


#endif // BSPLINE_OBJECTIVE_HPP

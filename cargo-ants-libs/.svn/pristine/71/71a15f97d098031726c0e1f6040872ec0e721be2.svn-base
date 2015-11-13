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


#include "DODrawing.hpp"
#include <npm/gfx/wrap_gl.hpp>
#include <sfl/util/Hull.hpp>
#include <sfl/util/pdebug.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <cmath>

#include <stdio.h>

using namespace sfl;
using namespace npm;


static void DrawHull(sfl::HullIterator ihull,
		     double red, double green, double blue,
		     bool filled);


DODrawing::
DODrawing(const std::string & name,
	  boost::shared_ptr<const sfl::DistanceObjective> distobj,
	  boost::shared_ptr<const sfl::HeadingObjective> headobj,
	  boost::shared_ptr<const sfl::DynamicWindow> dwa,
	  boost::shared_ptr<const sfl::RobotModel> rm)
  : Drawing(name,
	    "sfl::DistanceObjective (greyscale with special colors)"),
    m_distobj(distobj),
    m_headobj(headobj),
    m_dwa(dwa),
    m_rm(rm)
{
}


void DODrawing::
Draw()
{
  {
    // draw grid occuppancy
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    const double dx(m_distobj->GetDeltaX() / 2);
    const double dy(m_distobj->GetDeltaY() / 2);
    for(size_t ix(0); ix < m_distobj->DimX(); ++ix){
      const double xx(m_distobj->FindXlength(ix));
      for(size_t iy(0); iy < m_distobj->DimY(); ++iy){
	const double yy(m_distobj->FindYlength(iy));
	if(m_distobj->CellOccupied(ix, iy)){
	  switch(m_distobj->GetRegion(ix, iy)){
	  case DistanceObjective::NONE:   glColor3d(0, 0, 1); break;
	  case DistanceObjective::ZONE:   glColor3d(1, 1, 0); break;
	  case DistanceObjective::HULL:   glColor3d(1, 0, 0); break;
	  default:
	    PDEBUG_ERR("BUG in DODrawing::Draw(): invalid region[%zu][%zu]=%d\n",
		       ix, iy, m_distobj->GetRegion(ix, iy));
	    exit(EXIT_FAILURE);
	  }
 	}
	else{
	  switch(m_distobj->GetRegion(ix, iy)){
	  case DistanceObjective::NONE:   continue; break;
	  case DistanceObjective::ZONE:   glColor3d(0.5, 0.5 , 0); break;
	  case DistanceObjective::HULL:   glColor3d(0.5, 0   , 0); break;
	  default:
	    PDEBUG_ERR("BUG in DODrawing::Draw(): invalid region[%zu][%zu]=%d\n",
		       ix, iy, m_distobj->GetRegion(ix, iy));
	    exit(EXIT_FAILURE);
	  }
	}
	glRectd(xx - dx, yy - dy, xx + dx, yy + dy);
      }
    }
  }
  
  if(m_distobj->ObstacleInHull())
    DrawHull(HullIterator(*m_distobj->GetHull()),           1, 0, 1  , true);
  else
    DrawHull(HullIterator(*m_distobj->GetHull()),           1, 0, 0  , false);
  DrawHull(HullIterator(*m_distobj->GetPaddedHull()),       1, 0, 0.5, false);
  DrawHull(HullIterator(*m_distobj->GetEvaluationHull()),   0, 0, 1  , false);
  DrawHull(HullIterator(*m_rm->GetHull()),                0.5, 0.5, 1, false);
  
  if((m_dwa->QdlOptIndex() >= 0) &&
     (m_dwa->QdrOptIndex() >= 0)){
//     DrawObstaclePaths(m_dwa->QdlOptIndex(),
// 		      m_dwa->QdrOptIndex());
    DrawPose(sfl::Frame(), 1, 1, 1);
    DrawPose(m_headobj->PredictedStandstill(m_dwa->QdlOptIndex(),
					    m_dwa->QdrOptIndex()),
	     0, 1, 0);
  }

  DrawPath();
  
  const size_t n_near(m_distobj->GetNNear());
  if(0 < n_near){
    glColor3d(1, 0, 0.5);
    glPointSize(2);
    glBegin(GL_POINTS);
    for(size_t ii(0); ii < n_near; ++ii){
      double lx, ly;
      if(m_distobj->GetNear(ii, lx, ly))
	glVertex2d(lx, ly);
    }
    glEnd();
  }
}


void DODrawing::
DrawObstaclePaths(size_t iqdl, size_t iqdr)
{
  for(size_t ix(0); ix < m_distobj->DimX(); ++ix)
    for(size_t iy = 0; iy < m_distobj->DimY(); ++iy)
      if(m_distobj->CellOccupied(ix, iy)
	 && (DistanceObjective::ZONE == m_distobj->GetRegion(ix, iy)))
	DrawCollisionPrediction(iqdl, iqdr, ix, iy);
}


void DODrawing::
DrawCollisionPrediction(size_t iqdl, size_t iqdr, size_t igx, size_t igy)
{
  const double time(m_distobj->CollisionTime(igx, igy, iqdl, iqdr));
  const bool no_collision(time < 0);
  
  double sd, thetad;
  m_rm->Actuator2Global(m_dwa->Qd(iqdl),
		       m_dwa->Qd(iqdr),
		       sd,
		       thetad);
  double dist(sfl::absval(sd) * time);
  
  if(no_collision)
    glColor3d(0, 0, 1);
  else
    glColor3d(1, 0.5, 0);
  glPointSize(3);
  glBegin(GL_POINTS);
  glVertex2d(m_distobj->FindXlength(igx), m_distobj->FindYlength(igy));
  glEnd();
  glPointSize(1);
  
  if(sfl::absval(1e-9 * sd) < sfl::absval(thetad / 1e-9)){
    // circular equation
    double rCur(sd / thetad);
    double rGir(sqrt(pow(m_distobj->FindXlength(igx)       , 2) +
		     pow(m_distobj->FindYlength(igy) - rCur, 2)));
    double phi0(atan2(m_distobj->FindYlength(igy) - rCur,
		      m_distobj->FindXlength(igx)) * 180 / M_PI);
    
    double phi;
    if(no_collision)
      phi = 360;
    else
      phi = (dist / rCur) * 180 / M_PI;
    
    if(thetad >= 0)
      phi = - phi;

    if(no_collision)
      glColor3d(0, 0, 1);
    else
      if(thetad * sd > 0){
	phi = -phi;
	glColor3d(1, 0, 1);
      }
      else
	glColor3d(1, 1, 0);
    
    glLineWidth(1);
    glBegin(GL_LINE_STRIP);
    for(double a(0); a <= 1; a += 0.01){
      double b((phi0 - a * phi) * M_PI / 180);
      double x(rGir * cos(b));
      double y(rGir * sin(b) + rCur);
      glVertex2d(x, y);
    }
    glEnd();
  }
  else{
    // straight line approximation
    if(dist < 0)
      dist = 100;
    if(sd >= 0)
      dist = - dist;
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex2d(m_distobj->FindXlength(igx), m_distobj->FindYlength(igy));
    glVertex2d(m_distobj->FindXlength(igx)+dist, m_distobj->FindYlength(igy));
    glEnd();
  }
}


void DrawHull(sfl::HullIterator ihull,
	      double red, double green, double blue,
	      bool filled)
{
  glColor3d(red, green, blue);
  if(filled)
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  else
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  
  glLineWidth(1);
  glBegin(GL_POLYGON);
  for(/**/; ihull.IsValid(); ihull.Increment()){
    glVertex2d(ihull.GetP0()->X(), ihull.GetP0()->Y());
    //    glVertex2d(ihull.GetP1()->X(), ihull.GetP1()->Y());
  }
  glEnd();
}


void DODrawing::
DrawPose(const sfl::Frame & pose,
	 double red,
	 double green,
	 double blue)
{
  const double size(m_rm->WheelBase() / 2);
  glColor3d(red, green, blue);

  glLineWidth(1);
  glBegin(GL_LINES);

  double x0(0);
  double y0(0);
  double x1(size);
  double y1(0);
  pose.To(x0, y0);
  pose.To(x1, y1);
  glVertex2d(x0, y0);
  glVertex2d(x1, y1);

  x0 = 0;
  y0 = size;
  x1 = 0;
  y1 = -size;
  pose.To(x0, y0);
  pose.To(x1, y1);
  glVertex2d(x0, y0);
  glVertex2d(x1, y1);

  glEnd();
}


void DODrawing::
DrawPath()
{
  glLineWidth(1);
  glBegin(GL_LINES);
  glColor3d(1, 1, 1);
  glVertex2d(0, 0);
  glVertex2d(m_headobj->local_goal_x, m_headobj->local_goal_y);
  glEnd();
}

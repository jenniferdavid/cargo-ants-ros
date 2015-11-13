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


#include "MotionController.hpp"
#include "Timestamp.hpp"
#include "RobotModel.hpp"
#include <iostream>


using namespace boost;
using namespace std;


namespace sfl {
  
  
  MotionController::
  MotionController(shared_ptr<const RobotModel> robotModel,
		   shared_ptr<DiffDriveChannel> drive)
    : qdMax(robotModel->QdMax()),
      qddMax(robotModel->QddMax()),
      sdMax(robotModel->SdMax()),
      thetadMax(robotModel->ThetadMax()),
      m_robotModel(robotModel),
      m_drive(drive),
      m_proposedQdl(0),
      m_proposedQdr(0),
      m_currentQdl(0),
      m_currentQdr(0),
      m_wantedQdl(0),
      m_wantedQdr(0)
  {
  }
  
  
  int MotionController::
  Update(double timestep, ostream * dbgos)
  {
    double cqdl, cqdr;
    if(!m_drive->GetSpeed(cqdl, cqdr)){
      if(dbgos != 0)
	(*dbgos) << "ERROR in MotionController::Update():\n"
		 << "  m_drive->GetSpeed failed\n";
      return -1;
    }
    if(dbgos != 0){
      (*dbgos) << "INFO from MotionController::Update()\n"
	       << "  timestep: " << timestep << "\n"
	       << "  current:  (" << cqdl << ", " << cqdr << ")\n"
	       << "  proposed: (" << m_proposedQdl << ", "
	       << m_proposedQdr << ")\n";
    }
    
    // limit in actuator speed space
    const double dqd(timestep * qddMax);
    const double qdlMax(boundval(- qdMax, cqdl + dqd, qdMax));
    const double qdlMin(boundval(- qdMax, cqdl - dqd, qdMax));
    const double qdrMax(boundval(- qdMax, cqdr + dqd, qdMax));
    const double qdrMin(boundval(- qdMax, cqdr - dqd, qdMax));
    
    const double pqdl(boundval(qdlMin, m_proposedQdl, qdlMax));
    const double pqdr(boundval(qdrMin, m_proposedQdr, qdrMax));
    
    // limit in global speed space
    double sd, thetad;
    m_robotModel->Actuator2Global(pqdl, pqdr, sd, thetad);
    sd =     boundval( - sdMax,     sd,     sdMax);
    thetad = boundval( - thetadMax, thetad, thetadMax);
    double wqdl, wqdr;
    m_robotModel->Global2Actuator(sd, thetad, wqdl, wqdr);
    
    // send it
    if(dbgos != 0)
      (*dbgos) << "  wanted:   (" << wqdl << ", " << wqdr << ")\n";
    
    // synch cached values before treating status
    m_currentQdl = cqdl;
    m_currentQdr = cqdr;
    m_proposedQdl = pqdl;
    m_proposedQdr = pqdr;
    m_wantedQdl = wqdl;
    m_wantedQdr = wqdr;
    
    if(!m_drive->SetSpeed(wqdl, wqdr)){
      if(dbgos != 0)
	(*dbgos) << "ERROR in MotionController::Update():\n"
		 << "  m_drive->SetSpeed() failed\n";
      return -2;
    }
    return 0;
  }
  
  
  void MotionController::
  ProposeSpeed(double sd, double thetad)
  {
    m_robotModel->Global2Actuator(sd, thetad, m_proposedQdl, m_proposedQdr);
  }
  
  
  void MotionController::
  ProposeActuators(double qdLeft, double qdRight)
  {
    m_proposedQdl = qdLeft;
    m_proposedQdr = qdRight;
  }
  
  
  void MotionController::
  GetCurrentGlob(double & sd, double & thetad) const
  {
    m_robotModel->Actuator2Global(m_currentQdl, m_currentQdr, sd, thetad);
  }
  
  
  void MotionController::
  GetWantedGlob(double & sd, double & thetad) const
  {
    m_robotModel->Actuator2Global(m_wantedQdl, m_wantedQdr, sd, thetad);
  }
  
  
  void MotionController::
  GetProposedGlob(double & sd, double & thetad) const
  {
    m_robotModel->Actuator2Global(m_proposedQdl, m_proposedQdr, sd, thetad);
  }
  
  
  void MotionController::
  GetCurrentAct(double & qdLeft, double & qdRight) const
  {
    qdLeft = m_currentQdl;
    qdRight = m_currentQdr;
  }
  
  
  void MotionController::
  GetWantedAct(double & qdLeft, double & qdRight) const
  {
    qdLeft = m_wantedQdl;
    qdRight = m_wantedQdr;
  }
  
  
  void MotionController::
  GetProposedAct(double & qdLeft, double & qdRight) const
  {
    qdLeft = m_proposedQdl;
    qdRight = m_proposedQdr;
  }
  
}

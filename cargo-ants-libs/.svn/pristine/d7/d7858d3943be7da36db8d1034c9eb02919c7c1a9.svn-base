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


#include "Lidar.hpp"
#include "gfx/ScannerDrawing.hpp"
#include "RobotServer.hpp"
#include "World.hpp"
#include "NoiseModel.hpp"
#include <sfl/util/Ray.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/api/LidarChannel.hpp>
#include <sfl/api/Timestamp.hpp>
#include <iostream>

/// would be better to simulate a clock based on the timestep... ah well
//
#include <sys/time.h>

using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {


  Lidar::
  Lidar(const RobotServer * owner,
	std::string const & _name,
	const Frame & _mount,
	size_t _nscans,
	double _rhomax,
	double _phi0,
	double _phirange,
	shared_ptr<NoiseModel> noise_model)
    : Sensor(owner),
      name(_name),
      nscans(_nscans),
      rhomax(_rhomax),
      phi0(_phi0),
      phirange(_phirange),
      mount(new Frame(_mount)),
      m_noise_model(noise_model),
      m_global_pose(new Frame(_mount)),
      m_true_rho(nscans, _rhomax),
      m_noisy_rho(nscans, _rhomax)
  {
    m_drawing.reset(new ScannerDrawing(this));
  }
  
  
  void Lidar::
  InitUpdate()
  {
    *m_global_pose = *mount;
    owner->GetPose().To(*m_global_pose);
    for(size_t ir(0); ir < nscans; ++ir){
      m_true_rho[ir] = rhomax;
      m_noisy_rho[ir] = rhomax;
    }
  }
  
  
  void Lidar::
  StepUpdate(const Line & line)
  {
    double phi(m_global_pose->Theta() + phi0);
    double const dphi(phirange / (nscans - 1));
    Ray ray(m_global_pose->X(), m_global_pose->Y(), phi);

    for(size_t ir(0); ir < nscans; ++ir){
      
      const double true_rr(ray.Intersect(line));
      if((true_rr > 0) && (true_rr < m_true_rho[ir])){
	m_true_rho[ir] = true_rr;
	if( ! m_noise_model)
	  m_noisy_rho[ir] = true_rr;
      }
      
      if(m_noise_model){
	const double noisy_rr((*m_noise_model)(true_rr));
	if((noisy_rr > 0) && (noisy_rr < m_noisy_rho[ir]))
	  m_noisy_rho[ir] = noisy_rr;
      }
      
      phi += dphi;
      ray.SetAngle(phi);
    }
  }
  
  
  class LidarChannel
    : public sfl::LidarChannel
  {
  public:
    LidarChannel (Lidar const * lidar): m_lidar(lidar) {}
    
    virtual void GetData (std::vector<double> & data,
			  Timestamp & tupper,
			  Timestamp & tlower)
    {
      data.resize(m_lidar->nscans);
      for (size_t ii(0); ii < m_lidar->nscans; ++ii) {
	data[ii] = m_lidar->GetNoisyRho(ii);
      }
      
      struct timeval tv;
      gettimeofday(&tv, 0);
      timespec_t ts;
      TIMEVAL_TO_TIMESPEC(&tv, &ts);
      tupper = ts;
      tlower = tupper;
    }
    
    Lidar const * m_lidar;
  };
  
  
  boost::shared_ptr<sfl::LidarChannel> Lidar::
  CreateChannel() const
  {
    boost::shared_ptr<sfl::LidarChannel> myfavoritelittlelidar(new LidarChannel(this));
    return myfavoritelittlelidar;
  }
  
}

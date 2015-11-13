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


#include "Scanner.hpp"
#include "Scan.hpp"
#include "Timestamp.hpp"
#include "Pose.hpp"
#include <sfl/util/pdebug.hpp>
#include <sfl/util/Frame.hpp>
#include <boost/scoped_array.hpp>
#include <cmath>


using namespace boost;
using namespace std;


namespace sfl {
  
  
  Scanner::
  Scanner(shared_ptr<LocalizationInterface> localization,
	  shared_ptr<LidarChannel> channel, const Frame & _mount,
	  size_t _nscans, double _rhomax, double _phi0, double _phirange)
    : mount(new Frame(_mount)),
      nscans(_nscans),
      rhomax(_rhomax),
      phi0(_phi0),
      phirange(_phirange),
      dphi(_phirange / _nscans),
      m_localization(localization),
      m_channel(channel),
      m_acquisition_ok(false),
      m_cosphi(nscans, 0.0),
      m_sinphi(nscans, 0.0)
  {
    // XXXX to do: two raw pointers would do the job, instead of 4 shared ones
    m_buffer.push_back(shared_ptr<Scan>(new Scan(nscans, Timestamp::First(),
						 Timestamp::Last(),
						 *mount)));
    m_buffer.push_back(shared_ptr<Scan>(new Scan(*m_buffer.back())));
    m_dirty = m_buffer[0];
    m_clean = m_buffer[1];
    for(size_t i(0); i < nscans; ++i){
      m_clean->data[i].phi = phi0 + dphi * i;
      m_dirty->data[i].phi = m_clean->data[i].phi;
      m_cosphi[i] = cos(m_clean->data[i].phi);
      m_sinphi[i] = sin(m_clean->data[i].phi);
      m_clean->data[i].rho = rhomax;
      m_dirty->data[i].rho = rhomax;
      m_clean->data[i].in_range = false;
      m_dirty->data[i].in_range = false;
    }
  }
  
  
  int Scanner::
  Update()
  {
    // We work with the dirty buffer, and at the end do a quick swap
    // that would need to be protected by mutex if we were to support
    // multithreading.
    
    vector<double> rho;
    Timestamp t0, t1;
    m_channel->GetData(rho, t0, t1);
    if (rho.size() != nscans) {
      PDEBUG("nscans mismatch: wanted %zd but got %zd\n",
	     nscans, rho.size());
      m_acquisition_ok = false;
      return -42;
    }
    
    Pose robot_pose;
    m_localization->GetPose(robot_pose);
    
    m_dirty->tlower = t0;
    m_dirty->tupper = t1;
    m_dirty->scanner_pose.Set(*mount);
    robot_pose.To(m_dirty->scanner_pose);
    for(size_t ii(0); ii < nscans; ++ii){
      m_dirty->data[ii].rho = rho[ii];
      m_dirty->data[ii].in_range = rho[ii] < rhomax;
      m_dirty->data[ii].locx = rho[ii] * m_cosphi[ii];
      m_dirty->data[ii].locy = rho[ii] * m_sinphi[ii];
      mount->To(m_dirty->data[ii].locx, m_dirty->data[ii].locy);
      m_dirty->data[ii].globx = m_dirty->data[ii].locx;
      m_dirty->data[ii].globy = m_dirty->data[ii].locy;
      robot_pose.To(m_dirty->data[ii].globx, m_dirty->data[ii].globy);
    }
    m_acquisition_ok = true;
    swap(m_dirty, m_clean);
    
    return 0;
  }
  
  
  Scanner::status_t Scanner::
  GetData(size_t index, scan_data & data) const
  {
    if(index >= nscans)
      return INDEX_ERROR;
    
    data = m_clean->data[index];
    if(data.in_range)
      return SUCCESS;
    return OUT_OF_RANGE;
  }
  
  
  Scanner::status_t Scanner::
  Rho(size_t index, double & rho) const
  {
    if(index >= nscans)
      return INDEX_ERROR;
    rho = m_clean->data[index].rho;
    if(m_clean->data[index].in_range)
      return SUCCESS;
    return OUT_OF_RANGE;
  }
  
  
  Scanner::status_t Scanner::
  Phi(size_t index, double & phi) const
  {
    if(index >= nscans)
      return INDEX_ERROR;
    // even in case we added multithreading elsewhere, here there
    // would be no need for a mutex because phi isn't touched by
    // Update()
    phi = m_clean->data[index].phi;
    return SUCCESS;
  }
  
  
  Scanner::status_t Scanner::
  CosPhi(size_t index, double & cosphi) const
  {
    if(index >= nscans)
      return INDEX_ERROR;
    cosphi = m_cosphi[index];
    return SUCCESS;
  }
  
  
  Scanner::status_t Scanner::
  SinPhi(size_t index, double & sinphi) const
  {
    if(index >= nscans)
      return INDEX_ERROR;
    sinphi = m_sinphi[index];
    return SUCCESS;
  }
  
  
  shared_ptr<Scan> Scanner::
  GetScanCopy() const
  {
    return shared_ptr<Scan>(new Scan(*m_clean));
  }
  
  
  const Timestamp & Scanner::
  Tupper() const
  {
    return m_clean->tupper;
  }
  
  
  const Timestamp & Scanner::
  Tlower() const
  {
    return m_clean->tlower;
  }
  
}

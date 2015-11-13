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


#include "Multiscanner.hpp"
#include "Scanner.hpp"
#include "Scan.hpp"
#include "LocalizationInterface.hpp"
#include "Pose.hpp"
#include "../util/numeric.hpp"
#include "../util/pdebug.hpp"
#include <iostream>
#include <cmath>


using namespace boost;
using namespace std;


namespace sfl {
  
  
  Multiscanner::
  Multiscanner(shared_ptr<LocalizationInterface> localization)
    : m_localization(localization)
  {
  }
  
  
  void Multiscanner::
  Add(shared_ptr<Scanner> scanner)
  {
    m_total_nscans += scanner->nscans;
    m_scanner.push_back(scanner);
  }
  
  
  size_t Multiscanner::
  Nscanners() const
  {
    return m_scanner.size();
  }

  
  shared_ptr<Scanner> Multiscanner::
  GetScanner(size_t i) const
  {
    if(i >= m_scanner.size())
      return shared_ptr<Scanner>();
    return m_scanner[i];
  }
  
  
  shared_ptr<Scan> Multiscanner::
  CollectScans() const
  {
    shared_ptr<Scan> result;
    
    Pose robot_pose;
    m_localization->GetPose(robot_pose);
    
    // initialize to zero size (just add VALID data) and with INVERTED
    // timestamps to detect the min and max actual ones
    result.reset(new Scan(0, Timestamp::Last(), Timestamp::First(), robot_pose));
    
    for(size_t iScanner(0); iScanner < m_scanner.size(); ++iScanner){
      shared_ptr<Scanner> scanner(m_scanner[iScanner]);
      
      if(scanner->Tlower() < result->tlower)
	result->tlower = scanner->Tlower();
      if(scanner->Tupper() > result->tupper)
	result->tupper = scanner->Tupper();
      
      for(size_t iRay(0); iRay < scanner->nscans; ++iRay){
	scan_data data;
	// note: only add valid data, not even OUT_OF_RANGE!
	if(scanner->GetData(iRay, data) == Scanner::SUCCESS){
	  data.phi = atan2(data.locy, data.locx);
	  data.rho = sqrt(sqr(data.locx) + sqr(data.locy));
	  data.in_range = true;
	  result->data.push_back(data);
	}
      }
    }
    
    return result;
  }
  
  
  boost::shared_ptr<Multiscanner::raw_scan_collection_t> Multiscanner::
  CollectRawScans() const
  {
    boost::shared_ptr<raw_scan_collection_t>
      result(new raw_scan_collection_t());
    for(size_t iScanner(0); iScanner < m_scanner.size(); ++iScanner){
      shared_ptr<Scanner> scanner(m_scanner[iScanner]);
      result->push_back(scanner->GetScanCopy());
    }
    return result;
  }


  bool Multiscanner::
  UpdateAll(std::ostream * erros)
  {
    bool ok(true);
    for (size_t ii(0); ii < m_scanner.size(); ++ii) {
      int const status(m_scanner[ii]->Update());
      if (0 > status) {
	ok = false;
	if (erros)
	  *erros << "sfl::Multiscanner::UpdateAll(): update of scanner " << ii
		 << " failed with status " << status << "\n";
      }
    }
    return ok;
  }

}

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


#include "BubbleFactory.hpp"


using std::make_pair;


namespace sfl {
  
  
  BubbleFactory::
  BubbleFactory(size_t _batchsize)
    : batchsize(_batchsize)
  {
    Produce(batchsize);
  }
  
  
  BubbleFactory::
  ~BubbleFactory()
  {
    for(size_t ii(0); ii < m_stock.size(); ++ii)
      delete m_stock[ii];
  }
  
  
  Bubble * BubbleFactory::
  New(double cutoffDistance, double xpos, double ypos)
  {
    if(m_stock.empty())
      Produce(batchsize);
    Bubble * tmp(m_stock.back());
    m_stock.pop_back();
    tmp->Configure(cutoffDistance, make_pair(xpos, ypos));
    return tmp;
  }
  
  
  Bubble* BubbleFactory::
  Clone(Bubble * bubble)
  {
    if(m_stock.empty())
      Produce(batchsize);
    Bubble * tmp(m_stock.back());
    m_stock.pop_back();
    tmp->CopyConstruct(*bubble);
    return tmp;
  }
  
  
  void BubbleFactory::
  Delete(Bubble * bubble)
  {
    m_stock.push_back(bubble);
  }
  
  
  void BubbleFactory::
  Produce(size_t batch)
  {
    for(size_t ii(0); ii < batch; ++ii)
      m_stock.push_back(new Bubble());
  }
  
}

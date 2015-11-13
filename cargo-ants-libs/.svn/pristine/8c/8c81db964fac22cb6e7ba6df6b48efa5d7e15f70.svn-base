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


#ifndef SUNFLOWER_BUBBLELIST_HPP
#define SUNFLOWER_BUBBLELIST_HPP


#include <sfl/bband/Bubble.hpp>


namespace sfl {
  
  
  class BubbleBand;
  class BubbleFactory;
  
  
  class BubbleList
  {
  public:
    /** \todo Remove the (expo-) default parameters. */
    class Parameters {
    public:
      Parameters(double sp = 4, double lp = 8, double mid = 4)
	: shortpath(sp), longpath(lp), maxignoredistance(mid) { }
      const double shortpath;
      const double longpath;
      const double maxignoredistance;
    };
    
    
    BubbleList(BubbleBand & bubble_band,
	       BubbleFactory & bubble_factory,
	       const Parameters & params);
    BubbleList(BubbleList & original);
    
    ~BubbleList();
    
    
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value). */
    bool Update(boost::shared_ptr<const Scan> scan);
    
    /** Appends to the list of bubbles (or initailizes if empty list). */
    void Append(Bubble * bubble);
    
    /** Inserts a fresh bubble after one denoted "current". */
    void InsertAfter(Bubble * current, Bubble * fresh);
    
    /** Remove bubble from list and give it back to the BubbleFactory. */
    void Remove(Bubble * bubble);
    
    /** Calls Remove() on all bubbles in the list. */
    void RemoveAll();
    
    /** \return true if the list is empty. */
    bool Empty() const;
    
    /** \note Used for plotting only. */
    const Bubble * Head() const { return m_head; }
    
    
  private:
    friend class BubbleBand;
    
    
    /** cumulated path length at which the ignore radius heuristic starts */
    const double m_shortpath;// = 4;
    
    /** cumulated path length at which the ignore radius heuristic ends */
    const double m_longpath;//  = 8;
    
    /** cached value of m_longpath - m_shortpath */
    const double m_deltapath;// = m_LONGPATH - m_SHORTPATH;
    
    /** ignore distance heuristic for bubbles at m_longpath */
    const double m_maxignoredistance;// = 0;//4;
    
    BubbleBand & m_bubble_band;
    BubbleFactory & m_bubble_factory;
    
    Bubble * m_head;
    Bubble * m_tail;
    
    
    /** For all bubbles in the list: Reset parameters, update internal
	and external parameters, apply forces, re-compute external
	parameters.
	
	\note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value).

	\todo Remove the re-computating of external parameters! */
    void UpdateBubbles(boost::shared_ptr<const Scan> scan);
    
    /** Copmutes the cumulated path length for each bubble. */
    void UpdatePathLength();
    
    /** Computes the ignore distance for each bubble, based on the
	cumulated path length and some parameters. This is a heuristic
	to keep the band from snapping too frequently in dynamic
	crowded situations where most obstacles are humans. */
    void UpdateBubbleBehaviour();
    
    /** Tests if the bubble after the given one can be removed: If
	bubble and bubble->_next->_next overlap sufficiently, then
	bubble->_next is removed (ans cached values updated
	accordingly).
	
	\return true if the next bubble has been removed. */
    bool CheckRemove(Bubble * bubble);
    
    /** Tests if there is sufficient overlap between the given bubble
	and the next one. If not, then it attempts to fill the gap by
	inserting a new bubble between the two (and recomputing some
	cached values).
	
	\return true if a new bubble has been inserted.
	
	\note
	<ul>
	 <li>
	  Even if this method returns true, that is not a garantee
	  that the overlap is now sufficient. This should be checked
	  in a loop "above" this method.
	 </li>
	 <li>
	  The Scan object should be filtered, ie contain only
	  valid readings. This can be obtained from
	  Multiscanner::CollectScans(), whereas
	  Scanner::GetScanCopy() can still contain readings that are
	  out of range (represented as readings at the maximum rho
	  value).
	 </li>
	</ul>
    */
    bool CheckAdd(Bubble * bubble, boost::shared_ptr<const Scan> scan);
    
    /** Initializes the list of bubbles. */
    void FirstBubble(Bubble * bubble);
  };
  
}

#endif // SUNFLOWER_BUBBLELIST_HPP

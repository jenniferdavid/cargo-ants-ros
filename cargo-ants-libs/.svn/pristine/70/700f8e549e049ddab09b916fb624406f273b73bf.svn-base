/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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

#include <sfl/util/GoalManager.hpp>
#include <sfl/api/Goal.hpp>
#include <iostream>
#include <fstream>
#include <limits>

using namespace sfl;
using namespace boost;
using namespace std;


namespace sfl {
  
  GoalManager::
  GoalManager()
    : m_repeat(LOOP),
      m_current_goal(0)
  {
  }
  
  
  bool GoalManager::
  ParseConfig(const string & filename, ostream * os)
  {
    ifstream config(filename.c_str());
    if( ! config){
      if(os)
	*os << "ERROR opening config file \"" << filename << "\".\n";
      return false;
    }
    return ParseConfig(config, os);
  }
  
  
  bool GoalManager::
  ParseConfig(istream & is, ostream * os)
  {
    string token;
    while(is >> token){
      if(token[0] == '#'){
	is.ignore(numeric_limits<streamsize>::max(), '\n');
	continue;
      }
      if(token == "repeat"){
	string mode;
	if( ! (is >> mode)){
	  if(os)
	    *os << "ERROR reading repeat mode\n";
	  return false;
	}
	if(mode == "none")
	  m_repeat = NONE;
	else if(mode == "loop")
	  m_repeat = LOOP;
	else{
	  if(os)
	    *os << "ERROR invalid repeat mode \"" << mode << "\"\n";
	  return false;
	}
	continue;
      }
      if(token == "Goal"){
	double x;
	if( ! (is >> x)){
	  if(os)
	    *os << "ERROR reading goal #" << m_goal.size() << " x\n";
	  return false;
	}
	double y;
	if( ! (is >> y)){
	  if(os)
	    *os << "ERROR reading goal #" << m_goal.size() << " y\n";
	  return false;
	}
	double theta;
	if( ! (is >> theta)){
	  if(os)
	    *os << "ERROR reading goal #" << m_goal.size() << " theta\n";
	  return false;
	}
	double dr;
	if( ! (is >> dr)){
	  if(os)
	    *os << "ERROR reading goal #" << m_goal.size() << " dr\n";
	  return false;
	}
	double dtheta;
	if( ! (is >> dtheta)){
	  if(os)
	    *os << "ERROR reading goal #" << m_goal.size() << " dtheta\n";
	  return false;
	}
	AddGoal(x, y, theta, dr, dtheta);
	continue;
      }
      if(os)
	*os << "ERROR unknown token \"" << token << "\"\n";
      return false;
    }
    return true;
  }
  

  /* quick & dirty solution to parsing simple waypoints files */
  bool GoalManager::
  ParseConfigSimple(const string & filename, ostream * os,
		    const string & mode, double goal_theta, double goal_radius,
		    double goal_theta_diff)
  {
    ifstream config(filename.c_str());
    if( ! config){
      if(os)
	*os << "ERROR opening config file \"" << filename << "\".\n";
      return false;
    }
    return ParseConfigSimple(config, os, mode, goal_theta, goal_radius, goal_theta_diff);
  }

  /* this function assumes a set of x-y coordinates column-wise
     so, an extremely simple parser */
  bool GoalManager::
  ParseConfigSimple(istream & is, ostream * os, const std::string & mode,
		    double goal_theta, double goal_radius, double goal_theta_diff)
  {
    string token;
    int num_goals(0);
    
    //defining the mode first
    if(mode == "none")
      m_repeat = NONE;
    else if(mode == "loop")
      m_repeat = LOOP;
    else{
      if(os)
	*os << "ERROR invalid repeat mode \"" << mode << "\"\n";
      return false;
    }

    while(1)
      {
	double x;
	if( ! (is >> x)){
	  if(os)
	    *os << "Ending reading goals! at  #" << m_goal.size() << " x\n";
	  break;
	}
	
	double y;
	if( ! (is >> y)){
	  if(os)
	    *os << "Ending reading goals! at  #" << m_goal.size() << " y\n";
	  break;
	}
	num_goals++;
	AddGoal(x, y, goal_theta, goal_radius, goal_theta_diff);
	if(os)
	  *os << "Adding goal "<< num_goals<<":x="<<x<<" y="<<y<<"\n";
      }

    if(os){
      *os << "Ended reading goals\n"<<"The mode is:"<<mode<<"\n";
    }
    return true;
  }

  
  
  shared_ptr<Goal> GoalManager::
  GetCurrentGoal() const
  {
    if(m_goal.empty() || (m_current_goal >= m_goal.size()))
      return shared_ptr<Goal>();
    return m_goal[m_current_goal];
  }
  
  
  void GoalManager::
  AddGoal(double x, double y, double theta, double dr, double dtheta)
  {
    m_goal.push_back(shared_ptr<Goal>(new Goal(x, y, theta, dr, dtheta)));
  }
  
  
  void GoalManager::
  NextGoal()
  {
    if(m_goal.empty())
      return;
    ++m_current_goal;
    if(m_current_goal < m_goal.size())
      return;
    switch(m_repeat){
    case LOOP: m_current_goal = 0; break;
    case NONE: break;
    default:
      cerr << "BUG in sfl::GoalManager::NextGoal(): invalid m_repeat = "
	   << m_repeat << " treated as NONE\n";
    }
  }
  
  
  bool GoalManager::
  GoalReached(double x, double y, double theta, bool go_forward) const
  {
    return GoalReached(Frame(x, y, theta), go_forward);
  }
  
  
  bool GoalManager::
  GoalReached(const Frame & pose, bool go_forward) const
  {
    shared_ptr<Goal> goal(GetCurrentGoal());
    if( ! goal)
      return false;
    return goal->Reached(pose, go_forward);
  }
  
}

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

#include "Simulator.hpp"
#include <npm/World.hpp>
#include <npm/RobotClient.hpp>
#include <npm/RobotServer.hpp>
#include <npm/gfx/PNGImage.hpp>
#include <npm/gfx/View.hpp>
#include <npm/gfx/Drawing.hpp>
#include <npm/gfx/Camera.hpp>
#include <npm/pdebug.hpp>
#include <sfl/util/Frame.hpp>
#include <sfl/api/Pose.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <err.h>
#include <limits>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {


  AppWindow::
  AppWindow(std::string const & _name,
	    int width, int height, Simulator * simul)
    : name(_name),
      m_simul(simul),
      m_width(width),
      m_height(height)
  {
  }
  
  
  Simulator::
  Simulator(shared_ptr<World> world, double timestep,
	    bool _fatal_warnings)
    : fatal_warnings(_fatal_warnings),
      m_world(world),
      m_step(false),
      m_continuous(true),
      m_printscreen(false),
      m_timestep(timestep)
  {
    m_appwin.push_back(shared_ptr<AppWindow>(new AppWindow("nepumuk", 640, 480, this)));
  }


  Simulator::
  ~Simulator()
  {
  }


  bool Simulator::
  Initialize()
  {
    for (size_t ic(0); ic < RobotClient::registry.size(); ++ic) {
      RobotClient *client(RobotClient::registry.at(ic));
      RobotServer *server(new RobotServer(client, *m_world));
      if ( !client->Initialize(*server)) {
	delete server;
	delete client;
	return false;
      }
      m_world->AddRobot(server);
      m_robot.push_back(robot_s(server, client));
      Frame const pose(client->m_initial_pose.x, client->m_initial_pose.y, client->m_initial_pose.theta);
      server->InitializePose(pose);
      client->SetPose(Pose(client->m_initial_pose.x, client->m_initial_pose.y, client->m_initial_pose.theta));
      if (client->m_goals.size() > 0) {
	client->SetGoal(m_timestep, client->m_goals[0]);
      }
      
      // This was a nice feature that might be worth resurrecting using fpplib
      // if ( ! layout_file.empty())
      // 	m_appwin.push_back(shared_ptr<AppWindow>(new AppWindow(rdesc[ii]->name, layout_file,
      // 							       640, 480, this)));
    }
    
    UpdateAllSensors();
    
    return true;
  }
  
  
  void AppWindow::
  Reshape(int width,
	  int height)
  {
    // Do NOT try to be smart and skip if width and height haven't
    // changed, because we do call Reshape() with unchanged sizes when
    // view get initialized.
    
    m_width = width;
    m_height = height;
    
    for (View::registry_t::vector_t::const_iterator iv(View::registry.vector_.begin());
	 iv != View::registry.vector_.end(); ++iv) {
      (*iv)->Reshape(width, height);
    }
  }
  
  
  bool AppWindow::
  rfctDraw()
  {
    for (View::registry_t::vector_t::const_iterator iv(View::registry.vector_.begin());
	 iv != View::registry.vector_.end(); ++iv) {
      if ( !(*iv)->rfctDraw()) {
	return false;
      }
    }
    return true;
  }


  bool Simulator::
  Idle()
  {
    PVDEBUG("Hello!\n");
    bool retval(false);
    if(m_step || m_continuous){
      if(m_step)
	m_step = false;
      UpdateRobots();
      retval = true;
    }
   if(m_printscreen)
     for (size_t ii(0); ii < m_appwin.size(); ++ii)
       m_appwin[ii]->PrintScreen();
    return retval;
  }


  void Simulator::
  UpdateAllSensors()
  {
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir)
      ir->server->UpdateAllSensors();
  }


  void Simulator::
  UpdateRobots()
  {
    PVDEBUG("updating robots...\n");
  
    UpdateAllSensors();
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir){
      ir->runnable = ir->client->PrepareAction(m_timestep);
      if(( ! ir->runnable) && m_continuous){
	m_continuous = false;
	m_step = true;
      }
    }
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir)
      if(ir->runnable)
	ir->server->SimulateAction(m_timestep);
  
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir){
      if( ! ir->runnable)
	continue;
      if( ! ir->client->GoalReached())
	continue;
      if(ir->client->m_goals.size() <= 1)
	continue;
      ir->goalidx = (ir->goalidx + 1) % ir->client->m_goals.size();
      ir->client->SetGoal(m_timestep, ir->client->m_goals[ir->goalidx]);
    }
  }


  void AppWindow::
  Keyboard(unsigned char key,
	   int mousex,
	   int mousey)
  {
    m_simul->m_world->DispatchKey(key);
    switch(key){
    case ' ':
      m_simul->m_step = true;
      m_simul->m_continuous = false;
      m_simul->m_printscreen = false;
      break;
    case 'c':
      m_simul->m_step = false;
      m_simul->m_continuous = true;
      m_simul->m_printscreen = false;
      break;
    case 'p':
      if( ! m_simul->m_printscreen)
	PrintScreen();
      break;
    case 'P':
      m_simul->m_step = false;
      m_simul->m_continuous = true;
      m_simul->m_printscreen = true;
      break;    
    case 'q':
      cout << "\nbye bye!\n";
      exit(EXIT_SUCCESS);
      //    default:
      //// used to have fancy layout switching... that could be
      //// resurected with a little 'layout-key' parameter for each
      //// View.
      //
      // layout_map_t::const_iterator il(m_layout.find(key));
      // if (il != m_layout.end()) {
      // 	if ((m_active_layout == m_default_layout)
      // 	    || (m_active_layout != il->second))
      // 	  m_active_layout = il->second;
      // 	else
      // 	  m_active_layout = m_default_layout;
      // }
    }
  }


  void Simulator::
  SetContinuous(bool printscreen)
  {
    m_step = false;
    m_continuous = true;
    m_printscreen = printscreen;
  }


  void AppWindow::
  PrintScreen()
  {
#ifndef NPM_HAVE_PNG
    std::cerr << __FILE__": " << __func__ << ": PNG not supported in this build\n";
#else
    static unsigned int count(0);
    ostringstream filename;
    filename << "anim/" << name
	     << setw(6) << setfill('0') << count++
	     << ".png";
  
    PNGImage image(m_width, m_height);
    image.read_framebuf(0, 0);
    image.write_png(filename.str());
#endif
  }

}

/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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

#include <stdio.h>
#include <string.h>
#include <npm2/View.hpp>
#include <npm2/Simulator.hpp>
#include <npm2/Factory.hpp>
#include <npm2/Object.hpp>
#include <npm2/gl.hpp>
#include <err.h>
#include <cstdio>
#include <string.h>

using namespace npm2;


static Simulator * simulator (0);
static int window_handle;
static unsigned int const glut_timer_ms (1);


static void parse_cfile (char const * cfname)
{
  npm2::Factory & ff (npm2::Factory::instance());
  if ( ! ff.parseFile (cfname, cerr, &cerr)) {
    errx (EXIT_FAILURE, "%s: parse error (see above messages)", cfname);
  }
}


static void parse_args (int argc, char ** argv)
{
  bool config_ok (false);
  
  for (int ii (1); ii < argc; ++ii) {
    if (0 == strcmp ("-c", argv[ii])) {
      if (++ii >= argc) {
	errx (EXIT_FAILURE, "-c option expects argument");
      }
      parse_cfile (argv[ii]);
      config_ok = true;
    }
    else if (0 == strcmp ("-h", argv[ii])) {
      printf ("%s cfile [-c cfile] [-h]\n"
	      "  Nepumuk Mobile Robot Simulator v2\n"
	      "  Copyrights (C) 2004-2014 by\n"
	      "    Swiss Federal Institute of Technology, Lausanne, and\n"
	      "    Roland Philippsen. All rights reserved.\n"
	      "  Released under the terms of the GNU General Public License.\n"
	      "\n"
	      "  -h        print this help message.\n"
	      "  -c cfile  read additional configuration files.\n",
	      argv[0]);
      exit (EXIT_SUCCESS);
    }
    else {
      parse_cfile (argv[ii]);
      config_ok = true;
    }
  }
  
  printf ("Nepumuk Mobile Robot Simulator v2\n"
	  "  Swiss Federal Institute of Technology, Lausanne, and Roland Philippsen.\n"
	  "  Released under the terms of the GNU General Public License.\n");
  
  if ( ! config_ok) {
    errx (EXIT_FAILURE, "expected configuration file");
  }
}


static void reshape (int width, int height)
{
  for (size_t iv (0); iv < View::registry.size(); ++iv) {
    View::registry.at(iv)->reshape (width, height);
  }
}


static void draw ()
{
  glutSetWindow (window_handle); // not sure this is needed...
  glClear (GL_COLOR_BUFFER_BIT);
  
  for (size_t iv (0); iv < View::registry.size(); ++iv) {
    View::registry.at(iv)->draw();
  }
  
  glFlush ();
  glutSwapBuffers ();
}


static void keyboard (unsigned char key, int mx, int my)
{
  switch (key) {
  case ' ':
    simulator->state_ = Simulator::STEP;
    break;
  case 'c':
    if (simulator->state_ == Simulator::RUN) {
      simulator->state_ = Simulator::PAUSE;
    }
    else {
      simulator->state_ = Simulator::RUN;
    }
    break;
  case 'q':
    errx (EXIT_SUCCESS, "quit");
  }
  glutPostRedisplay ();
}


static void tick ()
{
  simulator->tick (cout);
  
  glutSetWindow (window_handle); // needed?
  glutPostRedisplay ();
}


static void timer (int handle)
{
  switch (simulator->state_) {
  case Simulator::PAUSE:
    break;
  case Simulator::STEP:
    tick();
    simulator->state_ = Simulator::PAUSE;
    break;
  case Simulator::RUN:
  default:
    tick();
  }
  
  glutTimerFunc (glut_timer_ms, timer, handle);
}


static void init_glut (int argc, char ** argv)
{
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
  
  glutInitWindowPosition (simulator->window_posx_, simulator->window_posy_);
  glutInitWindowSize (simulator->window_width_, simulator->window_height_);
  window_handle = glutCreateWindow (simulator->name.c_str());
  if (0 == window_handle) {
    errx(EXIT_FAILURE, "glutCreateWindow() failed");
  }
  
  glutDisplayFunc (draw);
  glutReshapeFunc (reshape);
  glutKeyboardFunc (keyboard);
  glutTimerFunc (glut_timer_ms, timer, window_handle);
}


int main (int argc, char ** argv)
{
  parse_args (argc, argv);
  
  simulator = npm2::Simulator::instance();
  if (simulator->init (cout)) {
    init_glut (argc, argv);
    glutMainLoop();
  }    
}

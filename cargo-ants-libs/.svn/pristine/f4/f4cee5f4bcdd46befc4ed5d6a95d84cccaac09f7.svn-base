/* 
 * Copyright (C) 2005
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


#include "Argtool.hpp"
#include <npm/World.hpp>
#include <iostream>


using namespace npm;
using namespace boost;
using namespace std;



class Parameters
{
public:
  Parameters():
    world_name("stage"),
    windows_eol(false)
  {
  }

  string world_name;
  bool windows_eol;
};


static Parameters params;


static void parse_options(int argc, char ** argv);


int main(int argc, char ** argv)
{
  parse_options(argc, argv);

  shared_ptr<World> world(World::Create(params.world_name));
  if(world == 0){
    cout << "Invalid World \"" << params.world_name << "\"\n";
    return 1;
  }
  
  world->DumpLines(cout, params.windows_eol);
}


void parse_options(int argc,
		   char ** argv)
{
  typedef Argtool::Callback<string> StringCB;

  Argtool atl;
  atl.Add(new StringCB(params.world_name,
			 'w', "world", "Name of the world (expo|mini|tta)."));
  atl.Add(new Argtool::BoolCallback(params.windows_eol,
					'd', "dos",
					"Windows-compatible line endings."));
  
  atl.UsageMessage(cerr, string(argv[0]) + " <options>");
  try {
    atl.Parse(argc, argv, 0);
  }
  catch(runtime_error e){
    cerr << "ERROR in parse_options():\n  " << e.what() << "\n";
    exit(EXIT_FAILURE);
  }
}

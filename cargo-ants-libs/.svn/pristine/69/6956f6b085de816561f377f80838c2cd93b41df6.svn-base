/*
 * fpplib - Factory and Parameter Parsing Library
 *
 * Copyright (c) 2012 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <fpplib/configurable.hpp>
#include <fpplib/factory.hpp>
#include <fpplib/yaml_parser.hpp>
#include <iostream>
#include <err.h>
#include <stdlib.h>

using namespace fpplib;
using namespace std;


namespace {
  
  
  struct Pose {
    double x, y, theta;
  };
  
  
  struct Goal {
    double x, y, theta, dr, dtheta;
  };
  
  
  struct Line {
    double x0, y0, x1, y1;
  };
  
  
  class Robot
    : public Configurable
  {
  public:
    Robot (string const & name)
      : Configurable (name)
    {
      reflectParameter ("pose", &pose_);
      reflectVectorParameter ("goals", &goals_);
      reflectParameter ("camera_zoom", &camera_zoom_);
    }
    
  private:
    Pose pose_;
    vector<Goal> goals_;
    double camera_zoom_;
  };
  
  
  class Robox
    : public Robot
  {
  public:
    Robox (string const & name)
      : Robot (name)
    {
      reflectParameter ("dwa_dimension", &dwa_dimension_);
      reflectParameter ("dwa_grid_resolution", &dwa_grid_resolution_);
      reflectParameter ("model_security_distance", &model_security_distance_);
    }

  private:
    int dwa_dimension_;
    double dwa_grid_resolution_;
    double model_security_distance_;
  };
  
  
  class Visitor
    : public Robot
  {
  public:
    Visitor (string const & name)
      : Robot (name)
    {
    }
  };
  
  
  class World
    : public Configurable
  {
  public:
    World (string const & name)
      : Configurable (name)
    {
      reflectVectorParameter ("lines", &lines_);
    }

  private:
    vector<Line> lines_;
  };
  
  
  class View
    : public Configurable
  {
  public:
    View (string const & name)
      : Configurable (name)
    {
      reflectParameter ("camera", &camera_name_);
      reflectVectorParameter ("drawings", &drawing_names_);
      reflectParameter ("window", &window_);
      reflectParameter ("border", &border_);
    }
  
  private:
    string camera_name_;	// slots would be tricky to marry with existing registries in npm
    vector<string> drawing_names_;
    Line window_;		// reusing x0, y0, x1, y1 here, but it may be x, y, w, h in npm
    int border_;
  };
  
  
  class NPMFactory
    : public Factory
  {
  public:
    NPMFactory()
    {
      declare<World>("world");
      declare<Robox>("robox");
      declare<Visitor>("visitor");
      declare<View>("view");
    }
  };
  
  
  void operator >> (const YAML::Node & node, Line & ll)
  {
    node[0] >> ll.x0;
    node[1] >> ll.y0;
    node[2] >> ll.x1;
    node[3] >> ll.y1;
  }
  
  
  void operator >> (const YAML::Node & node, Goal & gg)
  {
    node[0] >> gg.x;
    node[1] >> gg.y;
    node[2] >> gg.theta;
    node[3] >> gg.dr;
    node[4] >> gg.dtheta;
  }
  
  
  void operator >> (const YAML::Node & node, Pose & pp)
  {
    node[0] >> pp.x;
    node[1] >> pp.y;
    node[2] >> pp.theta;
  }
  
}


namespace std {
  
  
  ostream & operator << (ostream & os, Pose const &pp)
  {
    return os << "(" << pp.x << ", " << pp.y << ", " << pp.theta << ")";
  }
  
  
  ostream & operator << (ostream & os, Goal const &gg)
  {
    return os << "(" << gg.x << ", " << gg.y << ", " << gg.theta << ", "
	      << gg.dr << ", " << gg.dtheta << ")";
  }
  
  
  ostream & operator << (ostream & os, Line const &ll)
  {
    return os << "(" << ll.x0 << ", " << ll.y0 << ", " << ll.x1 << ", " << ll.y1 << ")";
  }
  
}


//// In case we start using yamlcpp-0.3.0 with their nifty new API
//
// namespace YAML {
//   template<>
//   struct convert<Line> {
//     static Node encode(Line const & rhs) {
//       Node node;
//       node.push_back(rhs.x0);
//       node.push_back(rhs.y0);
//       node.push_back(rhs.x1);
//       node.push_back(rhs.y1);
//       return node;
//     }
//     static bool decode(Node const & node, Line & rhs) {
//       if(!node.IsSequence())
// 	return false;
//       if(!node.size() == 4)
// 	return false;
//       rhs.x0 = node[0].as<double>();
//       rhs.y0 = node[1].as<double>();
//       rhs.x1 = node[2].as<double>();
//       rhs.y1 = node[3].as<double>();
//       return true;
//     }
//   };
// }

    
int main (int argc, char **argv)
{
  if (argc < 2)
    errx (EXIT_FAILURE, "Nepumuk YAML file name expected");
  
  NPMFactory ff;
  YamlParser pp (ff);
  pp.dbg = &cout;
  pp.addConverter<Line>();
  pp.addConverter<Goal>();
  pp.addConverter<Pose>();
  if ( ! pp.parseFile (argv[1]))
    errx (EXIT_FAILURE, "%s: %s", argv[1], pp.error.c_str());
  
  ff.dump("  ", cout);
  return 0;
}

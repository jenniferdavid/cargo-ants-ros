/* 
 * Copyright (C) 2012 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#include "Factory.hpp"
#include <npm/World.hpp>
#include <npm/Plugin.hpp>
#include <npm/ext/Zombie.hpp>
#include <npm/ext/expo02/Robox.hpp>
#include <npm/gfx/View.hpp>
#include <sfl/util/Line.hpp>
#include <fpplib/yaml_parser.hpp>
#include <err.h>

#ifdef SFL2_HAVE_ESTAR
# include <opt/npmestar/Esbot.hpp>
#endif // SFL2_HAVE_ESTAR


namespace sfl {
  
  void operator >> (const YAML::Node & node, Line & ll)
  {
    node[0] >> ll.p0._x;
    node[1] >> ll.p0._y;
    node[2] >> ll.p1._x;
    node[3] >> ll.p1._y;
  }

}

namespace npm {
  
  void operator >> (const YAML::Node & node, qhgoal_s & gg)
  {
    node[0] >> gg.x;
    node[1] >> gg.y;
    node[2] >> gg.theta;
    node[3] >> gg.dr;
    node[4] >> gg.dtheta;
  }
  
  
  void operator >> (const YAML::Node & node, qhpose_s & pp)
  {
    node[0] >> pp.x;
    node[1] >> pp.y;
    node[2] >> pp.theta;
  }
  
  
  void operator >> (const YAML::Node & node, qhwin_s & ww)
  {
    node[0] >> ww.x0;
    node[1] >> ww.y0;
    node[2] >> ww.x1;
    node[3] >> ww.y1;
  }
  
  
  void operator >> (const YAML::Node & node, color_s & cc)
  {
    node[0] >> cc.red;
    node[1] >> cc.green;
    node[2] >> cc.blue;
  }
  
  
  Factory::
  Factory()
    : parser_(*this)
  {
    declare<World>("World");
    declare<Robox>("Robox");
    declare<Zombie>("Zombie");
    declare<LidarZombie>("LidarZombie");
    declare<View>("View");
    declare<Plugin>("Plugin");
    
    // should turn this into a plugin now...
#ifdef SFL2_HAVE_ESTAR
    declare<Esbot>("Esbot");
#endif // SFL2_HAVE_ESTAR
    
    parser_.addConverter<sfl::Line>();
    parser_.addConverter<qhgoal_s>();
    parser_.addConverter<qhpose_s>();
    parser_.addConverter<qhwin_s>();
    parser_.addConverter<color_s>();
  }
  
  
  Factory & Factory::
  Instance()
  {
    static Factory * instance (0);
    if ( ! instance) {
      instance = new Factory();
    }
    return *instance;
  }
  
  
  World * Factory::
  GetWorld()
  {
    fpplib::Registry<World> const *wr(findRegistry<World>());
    if ( !wr) {
      warnx (__FILE__": %s: no world registry", __func__);
      return 0;
    }
    if (wr->size() != 1) {
      warnx (__FILE__": %s: expected one world, but got %zu", __func__, wr->size());
      return 0;
    }
    return wr->at(0);
  }
  
  
  fpplib::YamlParser & Factory::
  GetParser()
  {
    return parser_;
  }
  
  
  bool Factory::
  ParseFile(std::string const &yaml_filename, std::ostream & erros, std::ostream *dbgos)
  {
    parser_.dbg = dbgos;
    if ( ! parser_.parseFile(yaml_filename, erros)) {
      return false;
    }
    return true;
  }
  
}

/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2009 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#include <sfl/api/FrameFusion.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/util/numeric.hpp>
#include <vector>
#include <iostream>
#include <sstream>
#include <err.h>

using namespace sfl;
using namespace std;
typedef FrameFusion::speed_t speed_t;
typedef FrameFusion::frame_t frame_t;

static double const tick(0.03);
static size_t iodomlag(3);
static size_t islamlag(11);
static size_t islamskip(5);
static vector<speed_t> vel_cmd;
static vector<frame_t> true_pose;
static vector<frame_t> odom_noise;
static vector<frame_t> raw_odom;
static size_t const buflen(2048);
static char buf[buflen];

static void init();

int main(int argc, char ** argv)
{
  init();
	ostringstream log;
  FrameFusion ff(100, 10, 100, 100, &cout, &cout);
  for (size_t ii(0); ii < odom_noise.size(); ++ii) {
    ff.AddSpeedCommand(vel_cmd[ii].tstamp, vel_cmd[ii].data.sd, vel_cmd[ii].data.thetad);
    if (ii >= iodomlag) {
      size_t const jj(ii - iodomlag);
			ff.AddRawOdometry(raw_odom[jj].tstamp, raw_odom[jj].data);
    }
    if (ii >= islamlag) {
      size_t const jj(ii - islamlag);
      if (0 == jj % islamskip) {
				if ( ! ff.UpdateOdomCorrection(true_pose[jj].tstamp, true_pose[jj].data))
					errx(EXIT_FAILURE, "zonk ff.UpdateOdomCorrection!");
      }
    }
    Frame const ext(ff.Extrapolate(raw_odom[ii].tstamp));
    Frame delta(true_pose[ii].data);
    ext.From(delta);
    double const diff(absval(delta.Theta()) + absval(delta.X()) + absval(delta.Y()));
		char const * prefix("OK  ");
		if (diff >= 0.001)
      prefix = "grr ";
    snprintf(buf, buflen,
						 "%s t %5.2f %5.2f %5.2f"
						 "  x %5.2f %5.2f %5.2f"
						 "  d %5.2f %5.2f %5.2f"
						 "  n %5.2f %5.2f %5.2f\n",
						 prefix,
						 true_pose[ii].data.X(), true_pose[ii].data.Y(), true_pose[ii].data.Theta(),
						 ext.X(), ext.Y(), ext.Theta(),
						 delta.X(), delta.Y(), delta.Theta(),
						 odom_noise[ii].data.X(), odom_noise[ii].data.Y(), odom_noise[ii].data.Theta());
    cout << buf;
		log << buf;
  }
	cout << log.str();
}


void init()
{
  Timestamp timestep, now;
  timestep.FromSeconds(tick);
  
  for (double tt(0); tt < M_PI / 2; tt += 0.01 * M_PI) {
    global_speed_s gs(0.5 * sin(2 * tt), 3 * sin(tt));
    vel_cmd.push_back(speed_t(now, gs));
    now += timestep;
  }
  
  Frame pos(10, 0, 0);
  for (size_t ii(0); ii < vel_cmd.size(); ++ii) {
    double dx, dy, dth;
    RobotModel::LocalKinematics(vel_cmd[ii].data.sd, vel_cmd[ii].data.thetad, tick, dx, dy, dth);
    pos.Add(dx, dy, dth);
    true_pose.push_back(frame_t(vel_cmd[ii].tstamp, pos));
// 		Frame const noise(ii * 0.02,
// 											ii * 0.007 - 0.1,
// 											0.3 - ii * 0.01);
		Frame const noise;
    odom_noise.push_back(frame_t(vel_cmd[ii].tstamp, noise));
  }
  
  for (size_t ii(0); ii < odom_noise.size(); ++ii) {
    Frame raw(true_pose[ii].data);
    odom_noise[ii].data.To(raw);
    raw_odom.push_back(frame_t(vel_cmd[ii].tstamp, raw));
  }
}

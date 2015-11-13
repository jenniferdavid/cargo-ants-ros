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

#include "FrameFusion.hpp"
#include "RobotModel.hpp"
#include <iostream>
#include <cmath>
//#include <iostream>

namespace sfl {


	/** Beware: ringbuf indices go from youngest to oldest! */
	template<typename data_type>
	ssize_t	find_matchindex_recurse(Timestamp const & tstamp,
																	ringbuf<stamped<data_type> > const & buf,
																	ssize_t iyoung, ssize_t iold,
																	std::ostream * debug_os)
	{
		if (debug_os)
			*debug_os << "  iyoung: " << iyoung << "  iold: " << iold << "  t: " << tstamp << "\n";

		if (iyoung == iold) {
			if (debug_os)
				*debug_os << "  iyoung == iold\n";
			return iyoung;
		}

		if (buf[iold].tstamp >= tstamp) {
			if (debug_os)
				*debug_os << "  buf[iold].tstamp == " << buf[iold].tstamp
									<< " >= tstamp == " << tstamp << "\n";
			return iold;
		}

		if (buf[iyoung].tstamp <= tstamp) {
			if (debug_os)
				*debug_os << "  buf[iyoung].tstamp == " << buf[iyoung].tstamp
									<< " <= tstamp == " << tstamp << "\n";
			return iyoung;
		}

		ssize_t const isplit((iyoung + iold) / 2);
		if (debug_os)
			*debug_os << "  isplit: " << isplit << "\n";

		if (buf[isplit].tstamp < tstamp)
			return find_matchindex_recurse(tstamp, buf, iyoung, isplit, debug_os);
		return find_matchindex_recurse(tstamp, buf, isplit, iold, debug_os);
	}


	/** \todo generalize and extract into header */
	template<typename data_type>
	stamped<data_type> const *
	find_matching(Timestamp const & tstamp,
								ringbuf<stamped<data_type> > const & buf,
								size_t * imatch,
								std::ostream * debug_os)
	{
		ssize_t const buflen(buf.size());
		if (debug_os)
			*debug_os << "sfl::FrameFusion: find_matching: " << tstamp << "  buflen: " << buflen << "\n";
		if (0 == buflen)
			return 0;
		if (1 == buflen) {
			if (imatch)
				*imatch = 0;
			return &buf[0];
		}

		//size_t const ii(find_matchindex_recurse(tstamp, buf, 0, buflen - 1, debug_os));
		size_t bestIndex = 0;
		double bestDistance = std::abs((buf[0].tstamp - tstamp).ToSeconds());
		for (size_t i = 1; i < buf.size(); i++) {
			double newDistance = std::abs((buf[i].tstamp - tstamp).ToSeconds());
			if (newDistance < bestDistance) {
				bestDistance = newDistance;
				bestIndex = i;
			}
		}

		//std::cout << "ii = " << ii << ", bestIndex = " << bestIndex << std::endl;

		if (imatch)
			*imatch = bestIndex;
		return &buf[bestIndex];
	}


  FrameFusion::
  FrameFusion(size_t buflen_odom,
							size_t buflen_loc,
							size_t buflen_vel_com,
							size_t buflen_vel_act,
							std::ostream * error_os,
							std::ostream * debug_os)
    : m_error_os(error_os),
			m_debug_os(debug_os),
			m_odom(buflen_odom),
      m_loc(buflen_loc),
      m_corr(buflen_loc),
      m_vel_com(buflen_vel_com),
      m_vel_act(buflen_vel_act)
  {
  }

	Frame FrameFusion::
	GetLastRawOdometry()
	{
		Frame last;

		if (m_odom.empty()) {
			if (m_error_os)
				*m_error_os << "sfl::FrameFusion::GetLastRawOdometry(): no odometry data\n";
			return last;
		}

		last = m_odom[0].data;
		return last;
	}


	Frame FrameFusion::
	GetLastSlamPos()
	{
		Frame last;

		if (m_loc.empty()) {
			if (m_error_os)
				*m_error_os << "sfl::FrameFusion::GetLastSlamPos(): no SLAM data\n";
			return last;
		}

		last = m_loc[0].data;
		return last;
	}


  void FrameFusion::
  AddRawOdometry(Timestamp const & tstamp,
								 Frame const & pos)
  {
		if (m_debug_os)
			*m_debug_os << "sfl::FrameFusion::AddRawOdometry(" << tstamp << "  " << pos << ")\n";
    m_odom.push_back(frame_t(tstamp, pos));
  }


  void FrameFusion::
  AddSpeedCommand(Timestamp const & tstamp,
									double sd, double thetad)
  {
		if (m_debug_os)
			*m_debug_os << "sfl::FrameFusion::AddSpeedCommand(" << tstamp << "  "
									<< sd << " " << thetad << ")\n";
    m_vel_com.push_back(speed_t(tstamp, global_speed_s(sd, thetad)));
  }


  void FrameFusion::
  AddSpeedActual(Timestamp const & tstamp,
								 double sd, double thetad)
  {
		if (m_debug_os)
			*m_debug_os << "sfl::FrameFusion::AddSpeedActual(" << tstamp << "  "
									<< sd << " " << thetad << ")\n";
    m_vel_act.push_back(speed_t(tstamp, global_speed_s(sd, thetad)));
  }


  bool FrameFusion::
  UpdateOdomCorrection(Timestamp const & tstamp,
											 Frame const & slampos)
  {
		if (m_debug_os)
			*m_debug_os << "sfl::FrameFusion::UpdateOdomCorrection(" << tstamp
									<< "  " << slampos << ")\n";

    m_loc.push_back(frame_t(tstamp, slampos));

    frame_t const * match(find_matching<Frame>(tstamp, m_odom, 0, m_debug_os));
		if ( ! match) {
			if (m_error_os)
				*m_error_os << "sfl::FrameFusion::UpdateOdomCorrection(): no odometry for time "
										<< tstamp << "\n";
			return false;
		}
		if (m_debug_os)
			*m_debug_os << "  match: " << match->tstamp << "  " << match->data << "\n";

    Frame const & raw_odom(match->data);
    double const corr_th(slampos.Theta() - raw_odom.Theta());
    double const cos_corr_th(cos(corr_th));
    double const sin_corr_th(sin(corr_th));
    double const corr_x(slampos.X() - cos_corr_th * raw_odom.X() + sin_corr_th * raw_odom.Y());
    double const corr_y(slampos.Y() - sin_corr_th * raw_odom.X() - cos_corr_th * raw_odom.Y());

    m_corr.push_back(frame_t(tstamp, Frame(corr_x, corr_y, corr_th)));

		if (m_debug_os) {
			Frame check(match->data);
			m_corr[0].data.To(check);
			*m_debug_os << "  correction:  " << m_corr[0].data << "\n"
									<< "  check match: " << check << "\n";
		}

		return true;
  }


  Frame FrameFusion::
	Extrapolate(Timestamp const & tstamp)
	{
		Frame ext;

		if (m_odom.empty()) {
			if (m_error_os)
				*m_error_os << "sfl::FrameFusion::Extrapolate(): no odometry data\n";
			return ext;
		}

		frame_t const & latest_odom(m_odom[0]);
		ext.Set(latest_odom.data);

		if (m_corr.empty()) {
			if (m_error_os)
				*m_error_os << "sfl::FrameFusion::Extrapolate(): no correction data\n";
			return ext;
		}

		// apply latest odometry correction
		m_corr[0].data.To(ext);

		if (m_debug_os)
			*m_debug_os << "sfl::FrameFusion::Extrapolate()\n"
									<< "  correction:      " << m_corr[0] << "\n"
									<< "  latest raw odom: " << latest_odom << "\n"
									<< "  corrected odom:  " << ext << "\n";

		size_t i_vel_com;
    speed_t const *
			vel_com(find_matching<global_speed_s>(latest_odom.tstamp, m_vel_com, &i_vel_com, m_debug_os));
		if (vel_com) {
			if (m_debug_os)
				*m_debug_os << "  vel_com matching raw_odom starting at index " << i_vel_com << "\n";
			while (i_vel_com > 0) {
				Timestamp duration(m_vel_com[i_vel_com - 1].tstamp);
				duration -= vel_com->tstamp;
				double dx, dy, dth;
				RobotModel::LocalKinematics(vel_com->data.sd, vel_com->data.thetad, duration.ToSeconds(),
																		dx, dy, dth);
				ext.Add(dx, dy, dth);
				if (m_debug_os)
					*m_debug_os << "    vel_com " << i_vel_com << " " << *vel_com
											<< " during " << duration.ToSeconds() << "s\n"
											<< "      delta " << dx << " " << dy << " " << dth << "\n"
											<< "      extrapolated " << ext << "\n";
				--i_vel_com;
				vel_com = &(m_vel_com[i_vel_com]);
			}
			{
				Timestamp duration(tstamp);
				duration -= vel_com->tstamp;
				double dx, dy, dth;
				RobotModel::LocalKinematics(vel_com->data.sd, vel_com->data.thetad, duration.ToSeconds(),
																		dx, dy, dth);
				ext.Add(dx, dy, dth);
				if (m_debug_os)
					*m_debug_os << "    vel_com " << i_vel_com << " " << *vel_com
											<< " during " << duration.ToSeconds() << "s\n"
											<< "      delta " << dx << " " << dy << "" << dth << "\n"
											<< "      extrapolated " << ext << "\n";
			}
		}

		if (m_debug_os)
			*m_debug_os << "  final extrapolation:  " << ext << "\n";

		return ext;
	}


	Frame FrameFusion::
	GetOdomCorrection() const
	{
		Frame corr;

		if (m_corr.empty()) {
			if (m_error_os)
				*m_error_os << "sfl::FrameFusion::GetOdomCorrection(): no correction data\n";
			return corr;
		}

		corr = m_corr[0].data;
		return corr;
	}

}


namespace std {

	ostream & operator << (ostream & os, sfl::global_speed_s const & rhs) {
		os << "(" << rhs.sd << " " << rhs.thetad << ")";
		return os;
	}

}

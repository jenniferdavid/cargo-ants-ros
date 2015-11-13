/* -*- mode: C++; tab-width: 2 -*- */
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

#ifndef NPM_THREAD_DRAWING_HPP
#define NPM_THREAD_DRAWING_HPP


#include "../common/Drawing.hpp"
#include "../common/Camera.hpp"
#include "../common/wrap_gl.hpp"
#include "../common/View.hpp"
#include "../common/Manager.hpp"


using namespace npm;


namespace foo {
	template<typename status_t>
	void set_bg_color(status_t status) { glColor3d(0, 0, 0); }
	
	template<typename status_t>
	void set_fg_color(status_t status) { glColor3d(1, 1, 1); }
	
	template<typename Thread>
	struct stats_select {
		typedef void stats_t;
	};
}


template<typename Thread>
class ThreadDrawing
	: public npm::Drawing,
		public npm::Camera
{
public:
	typedef typename foo::stats_select<Thread>::stats_t stats_t;
	
	const Thread * m_thread;
	double m_max_delta_ms, m_min_delta_ms;
	
	ThreadDrawing(const std::string & name,
								const Thread * thread)
		: Drawing(name,
							"thread time and status stats",
							Instance<UniqueManager<Drawing> >()),
			Camera(name,
						 "auto zoom and scale on thread time stats",
						 Instance<UniqueManager<Camera> >()),
			m_thread(thread)
	{ }
	
	virtual void Draw() {
		const stats_t & stats(m_thread->stats);
		if(stats.status.empty())
			return;
		if(1 == stats.status.size()){
			foo::set_bg_color(stats.status[0]);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glRectd(0, 0, 1, 1);
		}
		else{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			for(size_t ii(0); ii < stats.status.size(); ++ii){
				foo::set_bg_color(stats.status[ii]);
				glRectd(ii    , 0, ///m_min_delta_ms,
								ii + 1, m_max_delta_ms);
			}
			glBegin(GL_LINE_STRIP);
			for(size_t ii(0); ii < stats.status.size(); ++ii){
				foo::set_fg_color(stats.status[ii]);
				glVertex2d(ii + 0.5, stats.delta_ms[ii]);
			}
			glEnd();
		}
	}
	
	virtual void ConfigureView(npm::View & view) {
		const stats_t & stats(m_thread->stats);
		view.UnlockAspectRatio();
		if(stats.comp_minmax(m_max_delta_ms, m_min_delta_ms))
			view.SetBounds(0, 0, stats.length, m_max_delta_ms);
		//view.SetBounds(0, m_min_delta_ms, stats.length, m_max_delta_ms);
		else
			view.SetBounds(0, 0, 1, 1);
	}
};

#endif // NPM_THREAD_DRAWING_HPP

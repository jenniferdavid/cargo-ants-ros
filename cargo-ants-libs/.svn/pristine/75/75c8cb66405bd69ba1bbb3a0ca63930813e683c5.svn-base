/* 
 * Copyright (C) 2005 Roland Philippsen <roland dot philippsen at gmx net>
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


#ifndef SFL_PDEBUG_HPP
#define SFL_PDEBUG_HPP


#include <boost/current_function.hpp>

#ifndef WIN32
# define PDEBUG_FULL_ERR(fmt, arg...) fprintf(stderr, "%s:\n  " fmt, BOOST_CURRENT_FUNCTION, ## arg)
# define PDEBUG_FULL_OUT(fmt, arg...) fprintf(stdout, "%s:\n  " fmt, BOOST_CURRENT_FUNCTION, ## arg)
# define PDEBUG_ERR(fmt, arg...) fprintf(stderr, "%s(): " fmt, __func__, ## arg)
# define PDEBUG_OUT(fmt, arg...) fprintf(stdout, "%s(): " fmt, __func__, ## arg)
# define PDEBUG_OFF(fmt, arg...)
#else
inline void PDEBUG_FULL_ERR(char const * fmt, ...) {}
inline void PDEBUG_FULL_OUT(char const * fmt, ...) {}
inline void PDEBUG_ERR(char const * fmt, ...) {}
inline void PDEBUG_OUT(char const * fmt, ...) {}
inline void PDEBUG_OFF(char const * fmt, ...) {}
#endif // WIN32

#ifdef SFL_VERBOSE_DEBUG
# define SFL2_VERBOSE_DEBUG
#endif
#ifdef SFL_DEBUG
# define SFL2_DEBUG 1
#endif

#ifdef SFL2_VERBOSE_DEBUG
# define SFL2_DEBUG
# define PVDEBUG PDEBUG_OUT
#else // ! SFL2_VERBOSE_DEBUG
# define PVDEBUG PDEBUG_OFF
#endif // SFL2_VERBOSE_DEBUG

#ifdef SFL2_DEBUG
# include <stdio.h>
# define PDEBUG PDEBUG_OUT
#else // ! SFL2_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // SFL2_DEBUG


#endif // SFL_PDEBUG_HPP

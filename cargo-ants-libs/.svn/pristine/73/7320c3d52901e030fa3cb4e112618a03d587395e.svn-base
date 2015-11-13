/* Minimal Estar implementation for 2D grid with LSM kernel.
 *
 * Copyright (C) 2013 Roland Philippsen. All rights reserved.
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

/**
   \mainpage
   
   This is the second rewrite of the E* Interpolated Graph Replanner.
   Everything is running like a charm, but some utility functions are
   missing for the most common usage scenarios.
   
   The initial (zeroth) version was experimental and relied on a
   polymorphic event mechanism.  Indeed, the E stands for event in
   that version.  It is documented in [Philippsen and Siegwart, "An
   Interpolated Dynamic Navigation Function", ICRA 2005].  Its C++
   implementation was not really fit for publishing.
   
   The first "proper" version was a complete rewrite based on the
   elegance of D*-Lite [Koenig and Likhachev, AAAI 2002].  It also was
   intended to be open sourced from the outset, and achieved a pretty
   decent stability.  However, the interface was bloated due to
   ill-advised support for multi-threading and continued support for
   several interpolation methods.  This is documented in [Philippsen,
   "A Light Formulation of the E* Interpolated Path Replanner",
   Technical Report EPFL 2006].
   
   Finally, summer 2013 saw this second rewrite, called E* v2. It is
   written in pure C (well -- some GNUisms have probably krept in)
   with an emphasis on minimalism.  Gone is the support for multiple
   interpolation kernels and graphs that can grow and shrink on the
   fly.  Gone are also the varied interfaces depending on whether a
   thread requires read or write access to the underlying
   datastructures.  What remains is an implementation that does one
   thing, but does that one thing as properly and minimally as
   possible or reasonable.  It served as demo for an invited talk at
   the 2013 SIAM Conference on Control and Its Application, but has
   not yet been published as a paper (and this is particularly high on
   my priority list).
   
   A good starting point is provided in the gestar.c source file,
   which is a GTK+ based app that demonstrates how E* works.  While it
   is propagating, you can click to add and remove obstacles.  You can
   also reset the computations and then the next click will set a new
   goal.  You can flush the computations to see how fast it actually
   runs (in "play" mode it is rather slow because it draws everything
   at every iteration).
   
   In order to use E* you can probably get by with just the following
   functions, in more or less this order:
   - estar_init()
   - estar_set_speed()
   - estar_set_goal()
   - estar_propagate()
   - estar_grid_at()
   - estar_reset()
   - estar_fini()
   
   What is still lacking from this code is a good (and minimal!) set
   of utility functions to help with the most common use cases.  So,
   for now just assume we want to to propagate everything
   (i.e. "flush" the computation).  This means you need to loop until
   the queue is empty:
   
   \code
   estar_t estar;
   // init etc
   while (estar.pq.len != 0) {
     estar_propagate (&estar);
   }
   \endcode     
   
   To trace back a solution path from a given cell to the goal depends
   on that cell having been visited, have a look at the
   cb_phi_expose() function in the gester.c sources, towards the end
   of the function.
*/


#ifndef ESTAR2_ESTAR_H
#define ESTAR2_ESTAR_H

#include <estar2/grid.h>
#include <estar2/pqueue.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
   E* computes the crossing-time values (called "phi" as this is
   commonly used in the literature about Level-Set and Fast-Marching
   Methods) at the center points of regular cells that are arranged in
   a grid.  Each cell has a "speed" which lies between 0 and 1, and
   which encodes the velocity with which a simulated wavefront should
   traverse that cell.  Thus, a 0 corresponds to obstacles (nothing
   can get through) and a 1 corresponds to free space (you can go at
   full speed).  Anything in between can be used to model areas that
   should be avoided but can be traversed if necessary, or to
   implement soft buffer zones around obstacles.
   
   E* maintains a priority queue of cells that need to get updated in
   order to propagate all information.  When you add goals (i.e. flag
   cells as being goals and setting their value to zero) or when you
   modify a cell's speed information, things get added to the queue.
   When a cell gets updated (during propagation) and its value
   changes, then its neighbors might be added to the queue (i.e. if
   they previously depended on the updated cell, or if the cell's new
   value presents an opportunity to lower the neighbor's value).
   
   You can keep an estar_t as an instance (e.g. statically or on the
   stack), or dynamically allocate it.  In either case, you have to
   call estar_init() before you can use it, and should call
   estar_fini() when you are done with it (so that internally
   allocated memory can be freed).
*/
typedef struct {
  estar_grid_t grid;
  estar_pqueue_t pq;
} estar_t;


/** Initializes the given estar_t instance to represent a grid with
    the given dimensions.  In case you are reusing a previously
    initialized estar_t instance, you should first call estar_fini()
    on it. */
void estar_init (estar_t * estar, size_t dimx, size_t dimy);

/** Clears everything except speed information. You need to
    estar_set_goal() again after calling this function. */
void estar_reset (estar_t * estar);

/** Frees up the memory allocated during estar_init(). */
void estar_fini (estar_t * estar);

/** Designates the given cell (specified by its indices) as being a
    goal cell.  At least one cell must be a goal, but there is no
    upper limit.  Indeed, it is a common usecase to have one E*
    instance where all the obstacle cells are goals, then E* serves as
    a distance transform (each cell will end up with a phi value equal
    to the Euclidean distance to the closest obstacle). */
void estar_set_goal (estar_t * estar, size_t ix, size_t iy);

/** Set the wavefront propagation speed for the given cell.  A speed
    of 0 (zero) means that this cell is an obstacle, and a speed of 1
    (one) means it lies in freespace.  Nothing prevents you from
    setting speeds higher than 1 (or indeed lower than 0), but the
    algorithm is not really designed to handle this properly. So, just
    don't do it. */
void estar_set_speed (estar_t * estar, size_t ix, size_t iy, double speed);

/** Internal function: update a single cell.  There is probably no
    good reason to have this exposed in the interface, except that it
    can help with experimentation and debugging. */
void estar_update (estar_t * estar, estar_cell_t * cell);

/** Perform one wavefront propagation step.  Repeatedly call this
    function in order to run E*. It takes the topmost cell from the
    priority queue, raises or lowers its phi value depending on why it
    had ended up on the queue in the first place, and then potentially
    schedules its neighbors for an update. */
void estar_propagate (estar_t * estar);

/** A debugging function to print the priority queue of cells that are
    pending updates to stdout. */
void estar_dump_queue (estar_t * estar, char const * pfx);

/** A debugging function which performs internal consistency checks.
    It returns 0 when everything is honky dory, and a bitmask
    otherwise.  It also writes human-readbable error messages to
    stdout, so you need not neccessarily worry about the meanings in
    the bitmask. */
int estar_check (estar_t * estar, char const * pfx);

#ifdef __cplusplus
}
#endif

#endif

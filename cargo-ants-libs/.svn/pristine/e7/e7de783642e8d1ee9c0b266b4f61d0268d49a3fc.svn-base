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

#include <estar2/estar.h>

#include <math.h>
#include <stdio.h>


static double interpolate (double cost, double primary, double secondary)
{
  double tmp;
  
  if (cost <= secondary - primary) {
    return primary + cost;
  }
  
  // pow(cost,2) could be cached inside estar_set_speed. And so could
  // the other squared terms. That might speed things up, but it would
  // certainly make hearier caching code.
  
  tmp = primary + secondary;
  return (tmp + sqrt(pow(tmp, 2.0)
		     - 2.0 * (pow(primary, 2.0)
			      + pow(secondary, 2.0)
			      - pow(cost, 2.0)))) / 2.0;
}


static void calc_rhs (estar_cell_t * cell, double phimax)
{
  estar_cell_t ** prop;
  estar_cell_t * primary;
  estar_cell_t * secondary;
  double rr;
  
  cell->rhs = INFINITY;
  prop = cell->prop;
  while (NULL != *prop) {
    
    primary = *(prop++);
    if (primary->rhs <= (*prop)->rhs)  {
      secondary = *(prop++);
    }
    else {
      secondary = primary;
      primary = *(prop++);
    }
    
    // do not propagate from obstacles, queued cells, cells above the
    // wavefront, or cells at infinity
    if (primary->flags & ESTAR_FLAG_OBSTACLE
	|| primary->pqi != 0
	|| primary->phi > phimax
	|| isinf(primary->phi)) {
      continue;
    }
    
    // the same goes from the secondary, but if that fails at least we
    // can fall back to the non-interpolated update equation.
    if (secondary->flags & ESTAR_FLAG_OBSTACLE
	|| secondary->pqi != 0
	|| secondary->phi > phimax
	|| isinf(secondary->phi)) {
      rr = primary->rhs + cell->cost;
    }
    else {
      rr = interpolate (cell->cost, primary->phi, secondary->phi);
    }
    
    if (rr < cell->rhs) {
      cell->rhs = rr;
    }
  }
  
  if (isinf (cell->rhs)) {
    // None of the above worked, we're probably done... but I have
    // lingering doubts about about the effects of in-place primary /
    // secondary sorting above, it could be imagined to create
    // situations where we overlook something. So, just to be on the
    // safe side, let's retry all non-interpolated options.
    for (prop = cell->nbor; *prop != 0; ++prop) {
      rr = (*prop)->phi;
      if (rr < cell->rhs) {
	cell->rhs = rr;
      }
    }
    cell->rhs += cell->cost;
  }
}


void estar_init (estar_t * estar, size_t dimx, size_t dimy)
{
  estar_grid_init (&estar->grid, dimx, dimy);
  estar_pqueue_init (&estar->pq, dimx + dimy);
}


void estar_reset (estar_t * estar)
{
  size_t const ncells = estar->grid.dimx * estar->grid.dimy;
  size_t ii;
  estar_cell_t * cell;
  
  for (ii = 0, cell = estar->grid.cell; ii < ncells; ++ii, ++cell) {
    cell->phi = INFINITY;
    cell->rhs = INFINITY;
    cell->key = INFINITY;
    cell->pqi = 0;
    cell->flags &= ~ESTAR_FLAG_GOAL;
  }
  
  estar->pq.len = 0;
}


void estar_fini (estar_t * estar)
{
  estar_grid_fini (&estar->grid);
  estar_pqueue_fini (&estar->pq);
}


void estar_set_goal (estar_t * estar, size_t ix, size_t iy)
{
  estar_cell_t * goal = estar_grid_at (&estar->grid, ix, iy);
  goal->rhs = 0.0;
  goal->flags |= ESTAR_FLAG_GOAL;
  goal->flags &= ~ESTAR_FLAG_OBSTACLE;
  estar_pqueue_insert_or_update (&estar->pq, goal);
}


void estar_set_speed (estar_t * estar, size_t ix, size_t iy, double speed)
{
  double cost;
  estar_cell_t * cell;
  estar_cell_t ** nbor;

  cell = estar_grid_at (&estar->grid, ix, iy);
  
  // XXXX I'm undecided yet whether this check here makes the most
  // sense. The other option is to make sure that the caller doesn't
  // place obstacles into a goal cell. The latter somehow makes more
  // sense to me at the moment, so in gestar.c there is code to filter
  // goal cells from the obstacle setting routines.
  ////  if (cell->flags & ESTAR_FLAG_GOAL) {
  ////    return;
  ////  }
  
  if (speed <= 0.0) {
    cost = INFINITY;
  }
  else {
    cost = 1.0 / speed;
  }
  if (cost == cell->cost) {
    return;
  }
  
  cell->cost = cost;
  if (speed <= 0.0) {
    cell->phi = INFINITY;
    cell->rhs = INFINITY;
    cell->flags |= ESTAR_FLAG_OBSTACLE;
  }
  else {
    cell->flags &= ~ESTAR_FLAG_OBSTACLE;
  }
  
  estar_update (estar, cell);
  for (nbor = cell->nbor; *nbor != 0; ++nbor) {
    estar_update (estar, *nbor);
  }
}


void estar_update (estar_t * estar, estar_cell_t * cell)
{
  /* XXXX check whether obstacles actually can end up being
     updated. Possibly due to effects of estar_set_speed? */
  if (cell->flags & ESTAR_FLAG_OBSTACLE) {
    estar_pqueue_remove_or_ignore (&estar->pq, cell);
    return;
  }
  
  /* Make sure that goal cells remain at their rhs, which is supposed
     to be fixed and only serve as source for propagation, never as
     sink. */
  if ( ! (cell->flags & ESTAR_FLAG_GOAL)) {
    calc_rhs (cell, estar_pqueue_topkey (&estar->pq));
  }
  
  if (cell->phi != cell->rhs) {
    estar_pqueue_insert_or_update (&estar->pq, cell);
  }
  else {
    estar_pqueue_remove_or_ignore (&estar->pq, cell);
  }
}


void estar_propagate (estar_t * estar)
{
  estar_cell_t * cell;
  estar_cell_t ** nbor;
  
  cell = estar_pqueue_extract (&estar->pq);
  if (NULL == cell) {
    return;
  }
  
  // The chunk below could be placed into a function called expand,
  // but it is not needed anywhere else.
  
  if (cell->phi > cell->rhs) {
    cell->phi = cell->rhs;
    for (nbor = cell->nbor; *nbor != 0; ++nbor) {
      estar_update (estar, *nbor);
    }
  }
  else {
    cell->phi = INFINITY;
    for (nbor = cell->nbor; *nbor != 0; ++nbor) {
      estar_update (estar, *nbor);
    }
    estar_update (estar, cell);
  }
}


int estar_check (estar_t * estar, char const * pfx)
{
  int status;
  size_t ii, jj, kk;
  
  status = 0;
  
  for (ii = 0; ii < estar->grid.dimx; ++ii) {
    for (jj = 0; jj < estar->grid.dimy; ++jj) {
      estar_cell_t * cell;
      cell = estar_grid_at (&estar->grid, ii, jj);
      
      if (cell->rhs == cell->phi) {
	// consistent
	if (0 != cell->pqi) {
	  printf ("%sconsistent cell should not be on queue\n", pfx);
	  status |= 1;
	}
      }
      else {
	// inconsistent
	if (0 == cell->pqi) {
	  printf ("%sinconsistent cell should be on queue\n", pfx);
	  status |= 2;
	}
      }
      
      if (0 == cell->pqi) {
	// not on queue
	for (kk = 1; kk <= estar->pq.len; ++kk) {
	  if (cell == estar->pq.heap[kk]) {
	    printf ("%scell with pqi == 0 should not be on queue\n", pfx);
	    status |= 4;
	    break;
	  }
	}
      }
      else {
	// on queue
	for (kk = 1; kk <= estar->pq.len; ++kk) {
	  if (cell == estar->pq.heap[kk]) {
	    break;
	  }
	}
	if (kk > estar->pq.len) {
	  printf ("%scell with pqi != 0 should be on queue\n", pfx);
	  status |= 8;
	}
      }
    }
  }
  
  for (ii = 1; ii <= estar->pq.len; ++ii) {
    if (estar->pq.heap[ii]->pqi != ii) {
      printf ("%sinconsistent pqi\n", pfx);
      estar_dump_queue (estar, pfx);
      status |= 16;
      break;
    }
  }
  
  return status;
}


void estar_dump_queue (estar_t * estar, char const * pfx)
{
  size_t ii;
  for (ii = 1; ii <= estar->pq.len; ++ii) {
    printf ("%s[%zu %zu]  pqi:  %zu  key: %g  phi: %g  rhs: %g\n",
	    pfx,
	    (estar->pq.heap[ii] - estar->grid.cell) % estar->grid.dimx,
	    (estar->pq.heap[ii] - estar->grid.cell) / estar->grid.dimx,
	    estar->pq.heap[ii]->pqi, estar->pq.heap[ii]->key,
	    estar->pq.heap[ii]->phi, estar->pq.heap[ii]->rhs);
  }
}

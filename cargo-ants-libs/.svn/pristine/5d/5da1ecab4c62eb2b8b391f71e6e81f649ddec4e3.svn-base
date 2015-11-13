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

#include <estar2/grid.h>

#include <stdlib.h>
#include <err.h>
#include <math.h>
#include <stdio.h>


void estar_grid_init (estar_grid_t * grid, size_t dimx, size_t dimy)
{
  size_t ix, iy;
  estar_cell_t * cell;
  estar_cell_t ** nbor;
  
  grid->cell = malloc (sizeof(estar_cell_t) * dimx * dimy);
  if (NULL == grid->cell) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc", __func__);
  }
  grid->dimx = dimx;
  grid->dimy = dimy;
  
  for (ix = 0; ix < dimx; ++ix) {
    for (iy = 0; iy < dimy; ++iy) {
      cell = estar_grid_at(grid, ix, iy);

      cell->cost = 1.0;
      cell->phi = INFINITY;
      cell->rhs = INFINITY;
      cell->key = INFINITY;
      cell->pqi = 0;
      cell->flags = 0;

      nbor = cell->nbor;
      if (ix > 0) {		/* west */
	*(nbor++) = cell - 1;
      }
      if (ix < dimx - 1) {	/* east */
	*(nbor++) = cell + 1;
      }
      if (iy > 0) {		/* south */
	*(nbor++) = cell - dimx;
      }
      if (iy < dimy - 1) {	/* north */
	*(nbor++) = cell + dimx;
      }
      *nbor = 0;
      
      nbor = cell->prop;
      if (ix > 0) {
	if (iy > 0) {		/* south-west */
	  *(nbor++) = cell - 1;
	  *(nbor++) = cell - dimx;
	}
	if (iy < dimy - 1) {	/* north-west */
	  *(nbor++) = cell - 1;
	  *(nbor++) = cell + dimx;
	}
      }
      if (ix < dimx - 1) {
	if (iy > 0) {		/* south-east */
	  *(nbor++) = cell + 1;
	  *(nbor++) = cell - dimx;
	}
	if (iy < dimy - 1) {	/* north-east */
	  *(nbor++) = cell + 1;
	  *(nbor++) = cell + dimx;
	}
      }
      *nbor = 0;
    }
  }
}


void estar_grid_fini (estar_grid_t * grid)
{
  free (grid->cell);
  grid->dimx = 0;
  grid->dimy = 0;
}


void estar_grid_dump_cell (estar_grid_t * grid, estar_cell_t const * cell, char const * pfx)
{
  size_t ix, iy;
  ix = (cell - grid->cell) % grid->dimx;
  iy = (cell - grid->cell) / grid->dimx;
  printf ("%s[%3zu  %3zu]  k: %4g  r: %4g  p: %4g\n",
	  pfx, ix, iy, cell->key, cell->rhs, cell->phi);
}

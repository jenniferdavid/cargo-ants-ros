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

#include <estar2/pqueue.h>
#include <estar2/cell.h>

#include <stdlib.h>
#include <err.h>
#include <stdio.h>
#include <math.h>


#define CALC_KEY(cell) ((cell)->rhs < (cell)->phi ? (cell)->rhs : (cell)->phi)


static void swap (estar_cell_t ** aa, estar_cell_t ** bb)
{
  size_t ti;
  estar_cell_t *tc;
  ti = (*aa)->pqi;
  (*aa)->pqi = (*bb)->pqi;
  (*bb)->pqi = ti;
  tc = (*aa);
  (*aa) = (*bb);
  (*bb) = tc;
}


static void bubble_up (estar_cell_t ** heap, size_t index)
{
  size_t parent;
  parent = index / 2;
  while ((parent > 0) && (heap[index]->key < heap[parent]->key)) {
    swap (&heap[index], &heap[parent]);
    index = parent;
    parent = index / 2;
  }
}


static void bubble_down (estar_cell_t ** heap, size_t len, size_t index)
{
  size_t child, target;
  
  target = index;
  while (1) {
    child = 2 * index;
    if (child <= len && heap[child]->key < heap[target]->key) {
      target = child;
    }
    ++child;
    if (child <= len && heap[child]->key < heap[target]->key) {
      target = child;
    }
    if (index == target) {
      break;
    }
    swap (&heap[target], &heap[index]);
    index = target;
  }
}


void estar_pqueue_init (estar_pqueue_t * pq, size_t cap)
{
  pq->heap = malloc (sizeof(estar_cell_t*) * (cap+1));
  if (NULL == pq->heap) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc", __func__);
  }
  pq->len = 0;
  pq->cap = cap;
}


void estar_pqueue_fini (estar_pqueue_t * pq)
{
  free (pq->heap);
  pq->len = 0;
  pq->cap = 0;
}


double estar_pqueue_topkey (estar_pqueue_t * pq)
{
  if (pq->len > 0) {
    return pq->heap[1]->key;
  }
  return INFINITY;
}


void estar_pqueue_insert_or_update (estar_pqueue_t * pq, estar_cell_t * cell)
{
  size_t len;
  estar_cell_t ** heap;
  
  if (0 != cell->pqi) {
    cell->key = CALC_KEY(cell);
    // could probably make it more efficient by only bubbling down when
    // the bubble up did not change cell->pqi
    bubble_up (pq->heap, cell->pqi);
    bubble_down (pq->heap, pq->len, cell->pqi);
    return;
  }
  
  // grow heap, realloc if necessary
  
  len = pq->len + 1;
  if (len <= pq->cap) {
    heap = pq->heap;
  }
  else {
    size_t cap;
    cap = 2 * pq->cap;
    heap = realloc (pq->heap, sizeof(estar_cell_t*) * (cap+1));
    if (NULL == heap) {
      errx (EXIT_FAILURE, __FILE__": %s: realloc", __func__);
    }
    pq->heap = heap;
    pq->cap = cap;
  }
  pq->len = len;
  
  // append cell to heap and bubble up
  
  cell->key = CALC_KEY(cell);
  heap[len] = cell;
  cell->pqi = len;		/* initialize pqi */
  bubble_up (heap, len);
}


void estar_pqueue_remove_or_ignore (estar_pqueue_t * pq, estar_cell_t * cell)
{
  if (0 == cell->pqi) {
    // This could be done by the caller for efficiency, but it is much
    // more convenient to do it here.
    return;
  }
  
  pq->heap[cell->pqi] = pq->heap[pq->len];
  pq->heap[cell->pqi]->pqi = cell->pqi; /* keep pqi consistent! */
  --pq->len;
  bubble_down (pq->heap, pq->len, cell->pqi);
  cell->pqi = 0;		/* mark cell as not on queue */
}


estar_cell_t * estar_pqueue_extract (estar_pqueue_t * pq)
{
  estar_cell_t * cell;
  
  if (0 == pq->len) {
    return NULL;
  }
  
  cell = pq->heap[1];
  cell->pqi = 0;		/* mark cell as not on queue */
  
  if (1 == pq->len) {
    pq->len = 0;
    return cell;
  }
  
  pq->heap[1] = pq->heap[pq->len];
  pq->heap[1]->pqi = 1;		/* keep pqi consistent */
  --pq->len;
  // here would be a good place to shrink the heap
  
  bubble_down (pq->heap, pq->len, 1);
  
  return cell;
}

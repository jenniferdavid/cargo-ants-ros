/*
 * Uniform Cubic B-Spline example code.
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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

#include <stdio.h>
#include <math.h>


struct control_point {
  double xx, yy;
};


struct path_point {
  double xx, yy, th;
  double dx_dl, dy_dl, dth_dl;
  double d2x_dl2, d2y_dl2, d2th_dl2;
  double d3x_dl3, d3y_dl3;
  double ds_dl, kappa;
};


struct at_params {
  double hitch_offset;
  double trailer_arm;
};


struct at_state {
  double xx, yy, th, phi;
};
  
  
#define NUM_NU 50

/* Feel free to change these. Beware that the time profile
   calculations currently ignore the possibility of triangular
   profiles */

#define A_MAX 0.4
#define V_MAX 5.5

#define DT 0.01


/* Below, you can put as many control points as you like. Well, we
   need at least 4. */

static struct control_point ctrlp[] = {
  {  10.0, 15.0},
  {  10.0,  5.0},
  {  20.0,  0.0},
  {  40.0,  0.0},
  {  60.0, 20.0},
  {  80.0, 20.0},
  {  80.0, 10.0},
  {  60.0,  0.0}
};

#define NUM_SEGMENTS ((sizeof ctrlp) / (sizeof *ctrlp) - 3)


struct arc_length_point {
  double lambda, s;
};


struct time_profile {
  double vmax, amax;
  double t1, t2, t3;
  double s1, s3;
};


void calc_b (double nu, double * bb)
{
  bb[0] = (  -pow(nu,3) + 3*pow(nu,2) - 3*nu + 1) / 6.0;
  bb[1] = ( 3*pow(nu,3) - 6*pow(nu,2)        + 4) / 6.0;
  bb[2] = (-3*pow(nu,3) + 3*pow(nu,2) + 3*nu + 1) / 6.0;
  bb[3] = (   pow(nu,3)                         ) / 6.0;
}


void calc_bd (double nu, double * bb)
{
  bb[0] = (-3*pow(nu,2) +  6*nu - 3) / 6.0;
  bb[1] = ( 9*pow(nu,2) - 12*nu    ) / 6.0;
  bb[2] = (-9*pow(nu,2) +  6*nu + 3) / 6.0;
  bb[3] = ( 3*pow(nu,2)            ) / 6.0;
}


void calc_bdd (double nu, double * bb)
{
  bb[0] = ( -6*nu +  6) / 6.0;
  bb[1] = ( 18*nu - 12) / 6.0;
  bb[2] = (-18*nu +  6) / 6.0;
  bb[3] = (  6*nu     ) / 6.0;
}


void calc_bddd (double nu, double * bb)
{
  bb[0] = -1.0;
  bb[1] =  3.0;
  bb[2] = -3.0;
  bb[3] =  1.0;
}


struct path_point calc_segment_point (struct control_point const * c4, double nu)
{
  int jj;
  double bb[4];
  struct path_point pp;
  
  calc_b (nu, bb);
  pp.xx = 0.0;
  pp.yy = 0.0;
  for (jj = 0; jj < 4; ++jj) {
    pp.xx += c4[jj].xx * bb[jj];
    pp.yy += c4[jj].yy * bb[jj];
  }
  
  calc_bd (nu, bb);
  pp.dx_dl = 0.0;
  pp.dy_dl = 0.0;
  for (jj = 0; jj < 4; ++jj) {
    pp.dx_dl += c4[jj].xx * bb[jj];
    pp.dy_dl += c4[jj].yy * bb[jj];
  }
  
  calc_bdd (nu, bb);
  pp.d2x_dl2 = 0.0;
  pp.d2y_dl2 = 0.0;
  for (jj = 0; jj < 4; ++jj) {
    pp.d2x_dl2 += c4[jj].xx * bb[jj];
    pp.d2y_dl2 += c4[jj].yy * bb[jj];
  }
  
  calc_bddd (nu, bb);
  pp.d3x_dl3 = 0.0;
  pp.d3y_dl3 = 0.0;
  for (jj = 0; jj < 4; ++jj) {
    pp.d3x_dl3 += c4[jj].xx * bb[jj];
    pp.d3y_dl3 += c4[jj].yy * bb[jj];
  }
  
  pp.th = atan2 (pp.dy_dl, pp.dx_dl);

  double const ff = pp.dx_dl * pp.d2y_dl2 - pp.dy_dl * pp.d2x_dl2;
  double const gg = pp.dx_dl * pp.dx_dl + pp.dy_dl * pp.dy_dl;
  pp.dth_dl = ff / gg;
  
  double const fd = pp.dx_dl * pp.d3y_dl3 - pp.dy_dl * pp.d3x_dl3;
  double const gd = 2.0 * (pp.dx_dl * pp.d2x_dl2 + pp.dy_dl * pp.d2y_dl2);
  pp.d2th_dl2 = (gd * ff - fd * gg) / gg * gg;
  
  pp.ds_dl = sqrt (pow(pp.dx_dl,2) + pow(pp.dy_dl,2));
  pp.kappa = (pp.dx_dl * pp.d2y_dl2 - pp.dy_dl * pp.d2x_dl2) / pow(pp.ds_dl, 3);
  
  return pp;
}


struct path_point calc_path_point (struct control_point const * ctrlp,
				   int num_ctrlp,
				   double lambda)
{
  int const nsegs = num_ctrlp - 3;
  int ioff;
  
  ioff = floor (lambda);
  if (ioff < 0) {
    ioff = 0;
    lambda = 0;
  }
  else if (ioff >= nsegs) {
    ioff = nsegs - 1;
    lambda = nsegs;
  }
  
  return calc_segment_point (ctrlp + ioff, lambda - ioff);
}


void dump_path_point (struct path_point const * pp)
{
  printf ("%f  %f  %f  %f  %f  %f  %f  %f  %f %f  %f",
	  pp->xx, pp->yy, pp->th,
	  pp->dx_dl, pp->dy_dl, pp->dth_dl,
	  pp->d2x_dl2, pp->d2y_dl2, pp->d2th_dl2, pp->kappa,
	  pp->ds_dl);
}


void dump_path_point_csv (struct path_point const * pp)
{
  printf ("%f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f, %f,  %f,  %f",
	  pp->xx, pp->yy, pp->th,
	  pp->dx_dl, pp->dy_dl, pp->dth_dl,
	  pp->d2x_dl2, pp->d2y_dl2, pp->d2th_dl2, pp->kappa,
	  pp->ds_dl,
	  pp->kappa);
}


struct time_profile create_profile (double ll, double vmax, double amax)
{
  struct time_profile pp;
  pp.vmax = vmax;
  pp.amax = amax;
  pp.t1 = vmax / amax;
  pp.t2 = ll / vmax;
  pp.t3 = pp.t1 + pp.t2;
  pp.s1 = 0.5 * vmax * pp.t1;
  pp.s3 = vmax * (pp.t2 - 0.5 * pp.t1) + 0.5 * amax * pp.t1 * pp.t1;
  return pp;
}


double calc_sd (struct time_profile const * pp, double tt)
{
  if (tt <= 0 || tt >= pp->t3) {
    return 0.0;
  }
  if (tt < pp->t1) {
    return pp->amax * tt;
  }
  if (tt < pp->t2) {
    return pp->vmax;
  }
  return pp->vmax - pp->amax * (tt - pp->t2);
}


double calc_s (struct time_profile const * pp, double tt)
{
  if (tt <= 0 || tt >= pp->t3) {
    return 0.0;
  }
  if (tt < pp->t1) {
    return 0.5 * pp->amax * tt * tt;
  }
  if (tt < pp->t2) {
    return pp->s1 + pp->vmax * (tt - pp->t1);
  }
  return pp->s3 - 0.5 * pp->amax * pow (tt - pp->t3, 2.0);
}


double invert_sl (struct arc_length_point const * arclen,
		  int n_arclen,
		  double ss)
{
  int const imax = n_arclen - 1;
  double const smin = arclen[0].s;
  double const smax = arclen[imax].s;
  double const lmin = arclen[0].lambda;
  double const lmax = arclen[imax].lambda;
  
  int ilow, iup;
    
  if (ss <= smin) {
    return lmin;
  }
  if (ss >= smax) {
    return lmax;
  }
  
  ilow = floor ((ss - smin) / (smax - smin));
  if (ilow < 0) {
    return lmin;
  }
  if (ilow >= imax) {
    return lmax;
  }
  while (arclen[ilow].s > ss) {
    --ilow;
    if (ilow < 0) {
      return lmin;
    }
  }
  
  iup = ilow + 1;
  while (arclen[iup].s <= ss) {
    ++iup;
    if (iup > imax) {
      return lmax;
    }
  }
  ilow = iup - 1;
  
  return
    arclen[ilow].lambda
    + (ss - arclen[ilow].s)
    * (arclen[iup].lambda - arclen[ilow].lambda)
    / (arclen[iup].s      - arclen[ilow].s);
}


struct at_state calc_at_update (struct at_params const * params,
				struct at_state const * state,
				double vtrans, double vrot, double dt)
{
  double const cth = cos (state->th);
  double const sth = sin (state->th);
  
  double const dx = dt * vtrans * cth;
  double const dy = dt * vtrans * sth;
  
  struct at_state next;
  next.xx = state->xx + dx * cth - dy * sth;
  next.yy = state->yy + dy * cth + dx * sth;
  next.th = state->th + dt * vrot;
  next.phi = state->phi - (dt * (vtrans * sin (state->phi)
				 + vrot * (params->hitch_offset * cos (state->phi) + 1.0))
			   / params->trailer_arm);
  
  return next;
}


int main (int argc, char ** argv)
{
  double tt;
  int ii, ll;
  struct path_point path[NUM_NU * NUM_SEGMENTS];
  struct arc_length_point arclen[NUM_NU * NUM_SEGMENTS];
  struct time_profile prof;
  //  struct path_point tmp;
  
  /* -------------------------------------------------- */
  
  ll = 0;
  for (ii = 0; ii < NUM_SEGMENTS; ++ii) {
    
    double nu;
    
    for (nu = 0.0; nu < 1.0; nu += 1.0 / NUM_NU) {
      
      path[ll] = calc_segment_point (ctrlp + ii, nu);
      
      arclen[ll].lambda = nu + ii;
      if (0 == ll) {
	arclen[ll].s = 0.0;
      }
      else {
	arclen[ll].s = arclen[ll-1].s + path[ll].ds_dl * (1.0 / NUM_NU);
      }

      /* printf ("%f  %f  ", */
      /* 	      arclen[ll].lambda, */
      /* 	      arclen[ll].s); */
      /* dump_path_point (&path[ll]); */
      /* printf ("\n"); */
      
      ++ll;
      
    }
  }
  /* printf ("# 1: lambda\n" */
  /* 	  "# 2: s\n" */
  /* 	  "# 3, 4, 5: x y th\n" */
  /* 	  "# 6, 7, 8: dx_dl dy_dl thd\n" */
  /* 	  "# 9, 10, 11: d2x_dl2 d2y_dl2 thdd\n" */
  /* 	  "# 12: ds_dl\n" */
  /* 	  "# 13: kappa\n"); */
  /* printf ("\n\n"); */
  
  /* -------------------------------------------------- */
  
  prof = create_profile (arclen[ll-1].s, V_MAX, A_MAX);

  struct at_params atp;
  atp.hitch_offset = 2.0;
  atp.trailer_arm = 12.0;
  
  struct at_state ats;
  
  for (tt = 0.0; tt <= prof.t3; tt += DT) {
    
    double const vtrans = calc_sd (&prof, tt);
    double const natparm = calc_s (&prof, tt);
    double const lambda = invert_sl (arclen, NUM_NU * NUM_SEGMENTS, natparm);
    
    //    double v;
    struct path_point const pp = calc_path_point (ctrlp, (sizeof ctrlp) / (sizeof *ctrlp), lambda);
    /* if (0.0 == tt) { */
    /*   v = 0.0; */
    /* } */
    /* else { */
    /*   v = sqrt (pow (pp.xx - tmp.xx, 2.0) + pow (pp.yy - tmp.yy, 2.0)) / DT; */
    /* } */
    /* tmp = pp; */
    
    ats.xx = pp.xx;
    ats.yy = pp.yy;
    ats.th = pp.th;
    if (0.0 == tt) {
      ats.phi = 0.0;
    }
    else {
      // v = Rw = w/k --> w = vk
      double const vrot = vtrans * pp.kappa;
      
      ats = calc_at_update (&atp, &ats, vtrans, vrot, DT);
    }
    
    printf ("%f  ", tt);
    dump_path_point (&pp);
    printf ("  %f  %f  %f  %f  %f\n", ats.xx, ats.yy, ats.th, ats.phi, vtrans);
  }
  printf ("# 1: tt\n"
  	  "# 2, 3 4: x y th\n"
  	  "# 5, 6, 7: dx_dl dy_dl thd\n"
  	  "# 8, 9, 10: d2x_dl2 d2y_dl2 thdd\n"
  	  "# 11: kappa\n"
  	  "# 12: ds_dl\n"
	  "# 13, 14, 15, 16: AT x y th phi\n"
	  "# 14: vtrans\n"
  	  "\n\n");
  
  /* -------------------------------------------------- */
  
  /* for (ii = 1; ii < NUM_NU * NUM_SEGMENTS; ii += 20) { */
  /*   double const stest = 0.5 * (arclen[ii-1].s + arclen[ii].s); */
  /*   double const lapprox = 0.5 * (arclen[ii-1].lambda + arclen[ii].lambda); */
  /*   double const check_l = invert_sl (arclen, NUM_NU * NUM_SEGMENTS, stest); */
  /*   printf ("%f  %f  %f  %f\n", */
  /* 	    stest, lapprox, check_l, check_l - lapprox); */
  /* } */
  
  /* printf ("# stest lapprox check_l err\n" */
  /* 	  "\n\n"); */
  
  /* -------------------------------------------------- */
  
  /* for (ii = 0; ii < (sizeof ctrlp) / (sizeof *ctrlp); ++ii) { */
  /*   printf ("%f  %f\n", ctrlp[ii].xx, ctrlp[ii].yy); */
  /* } */
  
  /* printf ("# x y (control points)\n"); */
  
  return 0;
}

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
  double x, y;
};


struct path_point {
  double x, y;
  double xd, yd;
  double xdd, ydd;
  double xddd, yddd;
  double theta, thetad, thetadd;
  double dsdl, kappa;
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
  { -20.0,  0.0},
  {   0.0,  0.0},
  {  20.0,  0.0},
  {  40.0,  0.0},
  {  60.0, 20.0},
  {  80.0, 20.0},
  { 100.0, 20.0},
  { 120.0, 20.0}
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
  pp.x = 0.0;
  pp.y = 0.0;
  for (jj = 0; jj < 4; ++jj) {
    pp.x += c4[jj].x * bb[jj];
    pp.y += c4[jj].y * bb[jj];
  }
  
  calc_bd (nu, bb);
  pp.xd = 0.0;
  pp.yd = 0.0;
  for (jj = 0; jj < 4; ++jj) {
    pp.xd += c4[jj].x * bb[jj];
    pp.yd += c4[jj].y * bb[jj];
  }
  
  calc_bdd (nu, bb);
  pp.xdd = 0.0;
  pp.ydd = 0.0;
  for (jj = 0; jj < 4; ++jj) {
    pp.xdd += c4[jj].x * bb[jj];
    pp.ydd += c4[jj].y * bb[jj];
  }
  
  calc_bddd (nu, bb);
  pp.xddd = 0.0;
  pp.yddd = 0.0;
  for (jj = 0; jj < 4; ++jj) {
    pp.xddd += c4[jj].x * bb[jj];
    pp.yddd += c4[jj].y * bb[jj];
  }
  
  pp.theta = atan2 (pp.yd, pp.xd);

  double const ff = pp.xd * pp.ydd - pp.yd * pp.xdd;
  double const gg = pp.xd * pp.xd + pp.yd * pp.yd;
  pp.thetad = ff / gg;
  
  double const fd = pp.xd * pp.yddd - pp.yd * pp.xddd;
  double const gd = 2.0 * (pp.xd * pp.xdd + pp.yd * pp.ydd);
  pp.thetadd = (gd * ff - fd * gg) / gg * gg;
  
  pp.dsdl = sqrt (pow(pp.xd,2) + pow(pp.yd,2));
  pp.kappa = (pp.xd * pp.ydd - pp.yd * pp.xdd) / pow(pp.dsdl, 3);
  
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
  printf ("%f  %f  %f  %f  %f  %f  %f  %f  %f %f  %f  %f",
	  pp->x, pp->y, pp->theta,
	  pp->xd, pp->yd, pp->thetad,
	  pp->xdd, pp->ydd, pp->thetadd, pp->kappa,
	  pp->dsdl,
	  pp->kappa);
}


void dump_path_point_csv (struct path_point const * pp)
{
  printf ("%f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f, %f,  %f,  %f",
	  pp->x, pp->y, pp->theta,
	  pp->xd, pp->yd, pp->thetad,
	  pp->xdd, pp->ydd, pp->thetadd, pp->kappa,
	  pp->dsdl,
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
	arclen[ll].s = arclen[ll-1].s + path[ll].dsdl * (1.0 / NUM_NU);
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
  /* 	  "# 6, 7, 8: xd yd thd\n" */
  /* 	  "# 9, 10, 11: xdd ydd thdd\n" */
  /* 	  "# 12: dsdl\n" */
  /* 	  "# 13: kappa\n"); */
  /* printf ("\n\n"); */
  
  /* -------------------------------------------------- */
  
  prof = create_profile (arclen[ll-1].s, V_MAX, A_MAX);
  
  for (tt = 0.0; tt <= prof.t3; tt += DT) {
    
    //    double sd = calc_sd (&prof, tt);
    double s = calc_s (&prof, tt);
    double l = invert_sl (arclen, NUM_NU * NUM_SEGMENTS, s);
    
    //    double v;
    struct path_point const pp = calc_path_point (ctrlp, (sizeof ctrlp) / (sizeof *ctrlp), l);
    /* if (0.0 == tt) { */
    /*   v = 0.0; */
    /* } */
    /* else { */
    /*   v = sqrt (pow (pp.x - tmp.x, 2.0) + pow (pp.y - tmp.y, 2.0)) / DT; */
    /* } */
    /* tmp = pp; */
    
    printf ("%f,  ", tt);
    dump_path_point_csv (&pp);
    printf ("\n");
  }
  /* printf ("# 1: tt\n" */
  /* 	  "# 2, 3 4: x y th\n" */
  /* 	  "# 5, 6, 7: xd yd thd\n" */
  /* 	  "# 8, 9, 10: xdd ydd thdd\n" */
  /* 	  "# 11: dsdl\n" */
  /* 	  "# 12: kappa\n" */
  /* 	  "\n\n"); */
  
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
  /*   printf ("%f  %f\n", ctrlp[ii].x, ctrlp[ii].y); */
  /* } */
  
  /* printf ("# x y (control points)\n"); */
  
  return 0;
}

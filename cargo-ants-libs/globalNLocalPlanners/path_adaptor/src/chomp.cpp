/* CHOMP.
 * 
 * Copyright (C) 2015 Rafael Valencia, Jennifer David. All rights reserved.
 * Based on code by 
 * 
 * This code uses and is based on code from:
 *   
 *   Copyright (C) 2015 Jafar Qutteineh. All rights reserved.
 *   License (3-Cluase BSD): https://github.com/j3sq/ROS-CHOMP/blob/master/LICENSE
 *    
 *   Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *   License (3-Clause BSD) : https://github.com/poftwaresatent/trychomp
 *  
 * 
 * **
 * \file chomp.cpp
 * 
 * CHOMP for point vehicles (x,y) moving holonomously in the plane. It will
 * plan a trajectory (xi) connecting start point (qs) to end point (qe) while
 * avoiding obstacles (obs)
 * 
 * ROS support for chomp path adaptor for Cargo-ANTS project http://cargo-ants.eu/
 *
 *
 */
#include <iostream>
#include <Eigen/Dense>
#include <stdlib.h>
#include <err.h>
#include "chomp.hpp"

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Isometry3d Transform;

using namespace std;

namespace chomp {
//////////////////////////////////////////////////
// trajectory etc


static size_t const nq(20);             // number of q stacked into xi
static size_t const cdim(2);            // dimension of config space
static size_t const xidim(nq *cdim);    // dimension of trajectory, xidim = nq * cdim
static size_t const iteration_limit(10000);
static double const dt(1.0);            // time step
static double const eta(100.0);         // >= 1, regularization factor for gradient descent
static double const lambda(1.0);        // weight of smoothness objective


//////////////////////////////////////////////////
// gradient descent etc

Matrix AA;                      // metric
Vector bb;                      // acceleration bias for start and end config
Matrix Ainv;                    // inverse of AA


static void init_chomp(Vector const &qs, Vector const &qe, Vector  &xi)
{
	//if (xi.rows() == xidim) {
		////do nothing. Use existing trajectory
	//} else {
		////initalize a new trajectory based on a direct line connecting qs to qe
		//xi = Vector::Zero(xidim);
		//Vector dxi(cdim);
		//dxi << (qe(0) - qs(0)) / (nq - 1), (qe(0) - qs(0)) / (nq - 1);
		//for (size_t ii(0); ii < nq; ++ii)
			//xi.block(cdim * ii, 0, cdim, 1) = qs + ii * dxi;

		///*
		 //* //use this instead if you want to inalize all points to qs
		 //* for (size_t ii (0); ii < nq; ++ii) {
		 //* xi.block (cdim * ii, 0, cdim, 1) = qs;
		 //* }
		 //*/
	//}
	AA = Matrix::Zero(xidim, xidim);
	for (size_t ii(0); ii < nq; ++ii) {
		AA.block(cdim * ii, cdim * ii, cdim, cdim) = 2.0 * Matrix::Identity(cdim, cdim);
		if (ii > 0) {
			AA.block(cdim * (ii - 1), cdim * ii, cdim, cdim) = -1.0 * Matrix::Identity(cdim, cdim);
			AA.block(cdim * ii, cdim * (ii - 1), cdim, cdim) = -1.0 * Matrix::Identity(cdim, cdim);
		}
	}
	AA /= dt * dt * (nq + 1);

	bb = Vector::Zero(xidim);
	bb.block(0, 0, cdim, 1) = qs;
	bb.block(xidim - cdim, 0, cdim, 1) = qe;
	bb /= -dt * dt * (nq + 1);

	// not needed anyhow
	// double cc (double (qs.transpose() * qs) + double (qe.transpose() * qe));
	// cc /= dt * dt * (nq + 1);

	Ainv = AA.inverse();
}


static double chomp_iteration(Vector const &qs, Vector const &qe, Vector  &xi, Matrix const &obs)
{
	//////////////////////////////////////////////////
	// beginning of "the" CHOMP iteration
	Vector nabla_smooth(AA * xi + bb);
	Vector const & xidd(nabla_smooth); // indeed, it is the same in this formulation...

	Vector nabla_obs(Vector::Zero(xidim));

	for (size_t iq(0); iq < nq; ++iq) {
		Vector const qq(xi.block(iq * cdim, 0, cdim, 1));
		Vector qd;
		if (0 == iq) {
			qd = 0.5 * (xi.block((iq + 1) * cdim, 0, cdim, 1) - qs);
		} else if (iq == nq - 1) {
			qd = 0.5 * (qe - xi.block((iq - 1) * cdim, 0, cdim, 1));
		} else {
			qd = 0.5 * (xi.block((iq + 1) * cdim, 0, cdim, 1) - xi.block((iq - 1) * cdim, 0, cdim, 1));;
		}

		// In this case, C and W are the same, Jacobian is identity.  We
		// still write more or less the full-fledged CHOMP expressions
		// (but  we only use one body point) to make subsequent extension
		// easier.
		//
		//Vector const & xx(qq);
		Vector const  xx(xi.block(iq * cdim, 0, 2, 1));
		Vector const  xd(qd);
		Matrix const JJ(Matrix::Identity(2, 2));        // a little silly here, as noted above.
		double const vel(xd.norm());
		if (vel < 1.0e-3)                               // avoid div by zero further down
			continue;
		Vector const xdn(xd / vel);
		Vector const xdd(JJ * xidd.block(iq * cdim, 0, cdim, 1));
		Matrix const prj(Matrix::Identity(2, 2) - xdn * xdn.transpose()); // hardcoded planar case
		Vector const kappa(prj * xdd / pow(vel, 2.0));


		for (int ii = 0; ii < obs.cols(); ii++) {
			
			
			//Vector delta(xx - obs.block(0, ii, cdim, 1));
			Vector delta(xx - obs.block(0, ii, 2, 1));
			double const dist(delta.norm());
			
			//if ((dist >= obs(2, ii)) || (dist < 1e-9))
			if ((dist >= 3) || (dist < 1e-9))
				continue;				
		 
			//static double const gain(10.0);                                                 // hardcoded param
			static double const gain(5.0);                                                 // hardcoded param
			double const cost(gain * obs(2, ii) * pow(1.0 - dist / obs(2, ii), 3.0) / 3.0); // hardcoded param
			delta *= -gain *pow(1.0 - dist / obs(2, ii), 2.0) / dist;                       // hardcoded param
			nabla_obs.block(iq * cdim, 0, cdim, 1) += JJ.transpose() * vel * (prj * delta - cost * kappa);
				 
			
		}
	}

	Vector dxi(Ainv * (nabla_obs + lambda * nabla_smooth));
	xi -= dxi / eta;
	//return the error (in Euclidean sense ). Remeber that the difference is -dxi/eta
	return dxi.norm() / eta;

	// end of "the" CHOMP iteration
	//////////////////////////////////////////////////
}

void generatePath(Vector const &qs, Vector const &qe, Vector &xi, Matrix const &obs)
{
	init_chomp(qs, qe, xi);
	double err;
	for (size_t ii = 0; ii < iteration_limit; ii++) {
		err = chomp_iteration(qs, qe, xi, obs);
		if (err < 0.01)
			break;
	}
}
} //namespace

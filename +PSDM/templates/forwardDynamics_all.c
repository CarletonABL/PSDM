/*
% _FUNCTIONNAME_.c Computes the inverse dynamic for a model, as per the E and P
% matrices which were used to generate it.
%
% Calling syntax is:
% 	_FUNCTIONNAME_(Q, Qd, tau, Theta, N, Qdd, tauInd, D);
% where
%   - Q, Qd, Qdd are the joint variables and their first and second
%     derivatives, respectively, in a _1DOF_xN matrix (column major)..
%   - Theta is the regression vector, a _ell_ column vector.
%   - tau is the joint torques, in a _1DOF_xN matrix (column major).
%	- tauInd is the induced joint torqoues (due to gravity, centrifugal and
%	  Coriolis effects). Size is _1DOF_xN (column major).
%	- D is the mass matrix, _1DOF_x_1DOF_xN (column major).
%
% See also PSDM.makeForwardDynamics.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
/*EXTRA_INCLUDES*/

void _FUNCTIONNAME_(const double *Q, const double *Qd, const double *tau, const double *Theta, int N, double *Qdd, double *tauInd, double *D)
{
	int pos;
	char i, j, k;
	int startInd, startIndD;
	double gi[_5DOF_];
	double d[_Dsize_];
	/*NAME_DEF*/

	/* Prep some variables for the matrix inversions */
	double sum;
	char iInd[_1DOF_];
	char dInd[_1DOF_];
	for (k = 0; k < _1DOF_; k++){
		dInd[k] = k*(k+3)/2;
		iInd[k] = k*(k+1)/2;
	}

	/*SETUP1_CODE*/

	for (pos = 0; pos < N; pos++){

		/* Define start index */
		startInd = pos * _1DOF_;
		startIndD = pos * _DOF2_;

		for (k = 0; k < _1DOF_; k++){
			gi[k] = Q[startInd+k];
			gi[k+_1DOF_] = sin(gi[k]);
			gi[k+_2DOF_] = cos(gi[k]);
			gi[k+_3DOF_] = Qd[startInd+k];
			gi[k+_4DOF_] = Qdd[startInd+k];
		}

/*SETUP2_CODE*/

/*TAU_CODE*/

		/* Invert matrix */
		// Copy matrix into D
		for (j = 0; j<_1DOF_; j++){
			for (i=0; i<=j; i++){
				if (i == j){
					D[startIndD + i*(_1DOF_+1)] = 1*d[ dInd[i] ];
				}else{
					D[startIndD + i+j*_1DOF_] = d[ i + iInd[j] ];
					D[startIndD + j+i*_1DOF_] = d[ i + iInd[j] ];
				}
			}
		}

		/* Do a destructive cholesky decomposition on d */
		for (i = 0; i<_1DOF_; i++){
			for (j = i; j<_1DOF_; j++){
				sum = d[ i+iInd[j] ];
				for (k = 0; k < i; k++){
					sum -= d[ k+iInd[i] ] * d[ k+iInd[j] ];
				} 
				if (i == j){
					if (sum <= 0.0){
						__ERROR_FUNCTION__("Cholesky decomposition failed. Results not reliable.");
						return;
					}
					d[ dInd[i] ] = sqrt(sum);
				}else{
					d[ i+iInd[j] ] = sum / d[ dInd[i] ];
				}
			}
		}

		/* Now use this decomposition to solve system of linear equations */
		for (i = 0; i<_1DOF_; i++){
			sum = tau[ startInd+i ] - tauInd[startInd+i];
			// printf("sum = %.5g\n", sum);
			for (k = 0; k < i; k++){
				sum -= d[ k+iInd[i] ] * Qdd[ startInd+k ];
			}
			Qdd[ startInd+i ] = sum/d[ dInd[i] ];
		}
		for (i=_1DOF_-1; i>=0; i--){
			sum = Qdd[ startInd+i ];
			for (k = i+1; k < _1DOF_; k++){
				sum -= d[ i+iInd[k] ] * Qdd[ startInd+k ];
			}
			Qdd[ startInd+i ] = sum/d[ dInd[i] ];
		}

	}
}

/*EXTRA_CODE*/
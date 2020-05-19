/*
% _FUNCTIONNAME_.c Computes the inverse dynamic for a model, as per the E and P
% matrices which were used to generate it.
%
% Calling syntax is:
% 	_FUNCTIONNAME_(Q, Qd, Qdd, Theta, N, tau);
% where
%   - Q, Qd, Qdd are the joint variables and their first and second
%     derivatives, respectively, in a _1DOF_xN matrix (column major)
%   - Theta is the regression vector, a _ell_ column vector.
%	- N is the number of samples given
%   - tau is the joint torques, in a _1DOF_xN matrix (column major)
%
% See also PSDM.makeInverseDynamics.
 */
#include <math.h>

void _FUNCTIONNAME_(const double *Q, const double *Qd, const double *Qdd, const double *Theta, int N, double *tau)
{
	int pos;
	char k;
	int startInd;
	double gi[_5DOF_];
	/*NAME_DEF*/

	/*SETUP1_CODE*/

	for (pos = 0; pos < N; pos++){

		/* Define start index */
		startInd = pos * _1DOF_;

		for (k = 0; k < _1DOF_; k++){
			gi[k] = Q[startInd+k];
			gi[k+_1DOF_] = sin(gi[k]);
			gi[k+_2DOF_] = cos(gi[k]);
			gi[k+_3DOF_] = Qd[startInd+k];
			gi[k+_4DOF_] = Qdd[startInd+k];
		}

/*SETUP2_CODE*/

		/*TAU_CODE*/

	}
}

/*EXTRA_CODE*/
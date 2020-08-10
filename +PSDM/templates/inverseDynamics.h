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

void _FUNCTIONNAME_(const double *Q, const double *Qd, const double *Qdd, const double *Theta, int N, double *tau);
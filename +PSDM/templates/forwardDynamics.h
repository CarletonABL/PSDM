/*
% _FUNCTIONNAME_.c Computes the inverse dynamic for a model, as per the E and P
% matrices which were used to generate it.
%
% Calling syntax is:
% 	_FUNCTIONNAME_(Q, Qd, tau, Theta, N, Qdd);
% where
%   - Q, Qd, Qdd are the joint variables and their first and second
%     derivatives, respectively, in a _1DOF_xN matrix.
%   - Theta is the regression vector, a _ell_ column vector.
%   - tau is the joint torques, in a _1DOF_xN matrix.
%
% See also PSDM.makeForwardDynamics.
 */

void _FUNCTIONNAME_(const double *Q, const double *Qd, const double *tau, const double *Theta, int N, double *Qdd);
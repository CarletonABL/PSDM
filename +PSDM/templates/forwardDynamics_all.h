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

void _FUNCTIONNAME_(const double *Q, const double *Qd, const double *tau, const double *Theta, int N, double *Qdd, double *tauInd, double *D);
% _MEXFUNCTIONNAME_ Computes the forward dynamics for a model, as per the E 
% and P matrices which were used to generate it.
%
% Calling syntax is:
%   [Qdd] = _MEXFUNCTIONNAME_(Q, Qd, tau, Theta)
%   [Qdd, tau_induced, D] = _MEXFUNCTIONNAME_(Q, Qd, tau, Theta)
% where
%   - Q, Qd, Qdd are the joint variables and their first and second
%     derivatives, respectively, in a _1DOF_xN matrix.
%   - Theta is the regression vector, a _ell_ column vector.
%   - tau is the joint torques, in a _1DOF_xN matrix.
%   - tau_induced are the joint torques due to gravitational, centrifugal
%     and Coriolis effects (_1DOF_xN matrix).
%   - D is a page matrix of the mass matrices for each configuration 
%     (_1DOF_x_1DOF_xN).
%
% See also PSDM.makeForwardDynamics.
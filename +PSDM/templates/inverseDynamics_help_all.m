% _MEXFUNCTIONNAME_ Computes the inverse dynamic for a model, as per the E and P
% matrices which were used to generate it.
%
% Calling syntax is:
%   [tau] = _MEXFUNCTIONNAME_(Q, Qd, Qdd, Theta)
%   [tau, Y] = _MEXFUNCTIONNAME_(Q, Qd, Qdd, Theta)
% where
%   - Q, Qd, Qdd are the joint variables and their first and second
%     derivatives, respectively, in a _1DOF_xN matrix.
%   - Theta is the regression vector, a _ell_ column vector.
%   - tau is the joint torques, in a _1DOF_xN matrix.
%   - Y is the regressor matrix (such that tau = Y*Theta), and is a 
%     _1DOF_x_ell_xN page matrix.
%
% See also PSDM.makeInverseDynamics.
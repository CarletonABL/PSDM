%% Two-Link Planar Manipulator PSDM Example
% This example shows how PSDM can be used to derive the regressor for of the 
% dynamics of motion of a simple two-link planar manipulator.
% 
% 
%% Derivation
% First, we define the DH table of the maniuplator. For the input to this toolbox, 
% DH table must be defined as an nx6 double matrix:
% 
% DH = [ a_1    alpha_1     d_1 	theta_1 	t_1 	s_1
%         :         :        :         :         :       :
%        a_n    alpha_n     d_n 	theta_n 	t_n 	s_n];
% 
% where a_i, alpha_i, d_i and theta_i are the normal DH parameters, t_i 
% indicates the joint type (0 for revolute, 1 for prismatic) and s_i is either 
% a 1 or a -1, and indicates the direction of the joint. Internally, they are 
% combined to make the DH table:
%
% Link length: a_i
% Link twist: alpha_i
% Link offset: di + t_i s_i q_i
% Link angle: theta_i + (1-t_i) s_i q_i

DH = [2.1   0	0     0   0   1;
      1.1   0	0     0   0   1];
  
%% 
% We also need a gravity vector. This is a unit vector and is scaled by the 
% gravitational constant internally. Here, gravity is pointing in the negative 
% y direction.

g = [0; 1; 0];

%% 
% Finally, we (optionally) need to define a vector of inertial parameters of 
% the robot, X. This is *not* needed to run the derivation. But, it can be used 
% to specify certain parameters as being zero. Internally, any parameter which 
% is identically zero in the given X matrix is assumed negligible and is not 
% included in the derivation, leading to simpler results.
% 
% X is defined as the concatenation of all the inertial parameters:
% 
%           [m_1  rcx_1  rcy_1  rcz_1  Ixx_1  Iyy_1  Izz_1  Ixy_1  Ixz_1  Iyz_1;
%             :     :      :      :       :     :      :      :     :       :
%           [m_n  rcx_n  rcy_n  rcz_n  Ixx_n  Iyy_n  Izz_n  Ixy_n  Ixz_n  Iyz_n];
% 
% Here, we assume the center of gravities are inline with the linkages, and 
% that all but the I_zz component of the inertias are negligible.

X = [1 -1 0 0 0 0 0.5 0 0 0;
     2 -1 0 0 0 0 0.3 0 0 0];
 
%% 
% We can also specify the tolerance (default is 1e-11) and verbosity (default 
% is true).

tolerance = [];
verbosity = true;

%% 
% Now, we can run the derivation

[E, P] = PSDM.deriveModel(DH, g, X, tolerance, verbosity);

%% 
% E is the exponent matrix which represents the terms of the final result. P 
% is a page array of the two reduction matrices for each joint. Because this is 
% a simple example, we can inspect them:

disp(E)
disp(P)

%% Testing
% The PSDM model, defined by E and P, can be used either through full knowledge 
% of X, the inertial parameters, or through experimental determination of the 
% Theta regression parameters.
% 
% Here, since we do not have experimental data, we can just use the $X$ parameters 
% above to validate the result. Note, this validation has actually already been 
% performed in the derivation above - this is just to illustrate.
% 
% First, use the X2Theta function to convert the X matrix into the appropriate 
% regression vector.

Theta = PSDM.X2Theta(DH, X, g, E, P);

%% 
% Now, define some random poses.

N = 5;
DOF = 2;
Q = (rand(DOF, N)-0.5) * (2*pi);
Qd = (rand(DOF, N)-0.5);
Qdd = (rand(DOF, N)-0.5);

%% 
% We can use the newton-euler algorithm to get the inverse dynamics.

tau_rne = PSDM.inverseDynamicsNewton(DH, X, Q, Qd, Qdd, 2, g)

%% 
% Then use the PSDM model, and check the error:

tau_psdm = PSDM.inverseDynamics(E, P, Theta, Q, Qd, Qdd)
fprintf('Max inverse dynamics error: %.5g\n', max(abs(tau_rne - tau_psdm), [], 'all'));

%% 
% We can also validate the forward dynamic model by ensuring that we get the 
% same joint accelerations we started with.

Qdd_psdm = PSDM.forwardDynamics(E, P, Theta, Q, Qd, tau_psdm)
fprintf('Max forward dynamics error: %.5g\n', max(abs(Qdd_psdm - Qdd), [], 'all'));

%% 
% The errors should be negligible (machine precision levels).

%% Faster real time code
% The default PSDM functions for inverse and forward dynamics are a bit slow 
% since there is some pre-processing to be done at each step, and complex indexing 
% which is slow for matlab.
% 
% However, because the model is quite nicely organized in E and P, we can procedurally 
% create matlab functions which are much faster than the default functions, specifically 
% for a given {E, P, \Theta} tuple.
% 
% Add the temp directory and define some filenames for the functions:

addpath(fullfile(utilities.PSDMDir, 'temp'));
filename_inverse = fullfile( utilities.PSDMDir, 'temp', 'PSDM_inverseDynamics_twolink.m');
filename_forward = fullfile( utilities.PSDMDir, 'temp', 'PSDM_forwardDynamics_twolink.m');

%% 
% Can decide if you want to mex-compile your code or not, as well as whether 
% or not to leverage parallel computing (requires the parallel computing toolbox). 
% Note, MEX compilation can take some time on more complex models.

doMex = true; % Whether or not to mex result for speed
doPar = true; % Whether to evaluate states in parallel

%% 
% Now, derive functions

PSDM.makeInverseDynamics(filename_inverse, E, P, Theta, ...
   'do_mex', doMex, 'parallel', doPar);
PSDM.makeForwardDynamics(filename_forward, E, P, Theta, ...
   'do_mex', doMex, 'parallel', doPar);

%% Test Evaluation Speed
% The code below generations a large number of sample poses and checks the evaluation 
% speed of the forward and inverse dynamics functions.

% Generate samples
N = 10e5;
DOF = 2;
Q = (rand(DOF, N)-0.5) * (2*pi);
Qd = (rand(DOF, N)-0.5) * (10);
Qdd = (rand(DOF, N)-0.5) * (50);

tic1 = tic;
tau_psdm = PSDM.inverseDynamics(E, P, Theta, Q, Qd, Qdd);
t1 = toc(tic1);

fprintf("Default inverse dynamics function took %.3g seconds per state.\n", t1/N);

% Run test. We run the function once first since matlab doesn't run
% functions very quickly the first time.
A = PSDM_forwardDynamics_twolink_mex(Q(:, 1), Qd(:, 1), Qdd(:, 1));
tic2 = tic;
tau_psdm_opt = PSDM_inverseDynamics_twolink_mex(Q, Qd, Qdd);
t2 = toc(tic2);

fprintf("Optimized inverse dynamics function took %.3g seconds per state.\n", t2/N);

fprintf("Maximum error: %.5g\n", max(abs(tau_psdm - tau_psdm_opt), [], 'all'));

tic3 = tic;
Qdd_psdm = PSDM.forwardDynamics(E, P, Theta, Q, Qd, tau_psdm);
t3 = toc(tic3);

fprintf("Default forward dynamics function took %.3g seconds per state.\n", t3/N);

% Run test. We run the function once first since matlab doesn't run
% functions very quickly the first time.
A = PSDM_forwardDynamics_twolink_mex(Q(:, 1), Qd(:, 1), tau_psdm(:, 1));
tic4 = tic;
Qdd_psdm_opt = PSDM_forwardDynamics_twolink_mex(Q, Qd, tau_psdm);
t4 = toc(tic4);

fprintf("Optimized forward dynamics function took %.3g seconds per state.\n", t4/N);

fprintf("Maximum error: %.5g\n", max(abs(Qdd_psdm - Qdd_psdm_opt), [], 'all'));
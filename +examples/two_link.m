%% Perform derivation

% Make DH Table
% Order of terms is:
%   [a_1  alpha_1    d_1   theta_1    lt_1    q_sign_1;
%     :      :        :       :         :         :     
%    an   alpha_n    d_n   theta_n    lt_n    q_sign_n];
DH_ext = [2.1   0	0	0   0   1;
          1.1	0	0	0   0   1];
      
% Make Xlist
% Order of terms are:
%  [m_1  rcx_1  rcy_1  rcz_1  Ixx_1  Iyy_1  Izz_1  Ixy_1  Ixz_1  Iyz_1;
%    :     :      :      :       :     :      :      :     :       :
%   m_n  rcx_n  rcy_n  rcz_n  Ixx_n  Iyy_n  Izz_n  Ixy_n  Ixz_n  Iyz_n];
X = [1 -1 0 0 0 0 0.5 0 0 0;
     2 -1 0 0 0 0 0.3 0 0 0];
 
% Define gravity vector
g = [0; 1; 0];

% Tolerance (empty for default tolerance)
tolerance = [];

% Verbosity
verbosity = true;

% Order of arguments: DHext, X, g, tolerance (empty for default), verbosity
% flag.
[E, P] = PSDM.runID(DH_ext, X, g, tolerance, verbosity);

%% Testing

% Get theta directly from known values of X
Theta = PSDM.getTheta(DH_ext, X, g, E, P);

% Generate some random test poses
N = 5;
DOF = 2;
Q = (rand(DOF, N)-0.5) * (2*pi);
Qd = (rand(DOF, N)-0.5) * (10);
Qdd = (rand(DOF, N)-0.5) * (50);

% Use newton-euler to get joint torques
tau_rne = PSDM.inverseDynamicsNewton(DH_ext, X, Q, Qd, Qdd, 2, g);

% Use PSDM to get torques
tau_psdm = PSDM.inverseDynamics(E, P, Theta, Q, Qd, Qdd);

fprintf('Max inverse dynamics error: %.5g\n', max(abs(tau_rne - tau_psdm), [], 'all'));

% Test forward dynamics
Qdd_psdm = PSDM.forwardDynamics(E, P, Theta, Q, Qd, tau_psdm);
fprintf('Max forward dynamics error: %.5g\n', max(abs(Qdd_psdm - Qdd), [], 'all'));

%% Faster real time code

% Generate dedicated function

addpath(fullfile(utilities.PSDMDir, 'temp'));
filename_inverse = fullfile( utilities.PSDMDir, 'temp', 'PSDM_inverseDynamics_twolink.m');
filename_forward = fullfile( utilities.PSDMDir, 'temp', 'PSDM_forwardDynamics_twolink.m');
doMex = true; % Whether or not to mex result for speed
doPar = true; % Whether to evaluate states in parallel
PSDM.makeInverseDynamics(filename_inverse, E, P, Theta, ...
   'do_mex', doMex, 'parallel', doPar);
PSDM.makeForwardDynamics(filename_forward, E, P, Theta, ...
   'do_mex', doMex, 'parallel', doPar);

%% Test time with real time code
% Note, may need to run this section twice to get good results, since
% matlab takes more time to run new functions the first time.

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

tic2 = tic;
tau_psdm_opt = PSDM_inverseDynamics_twolink_mex(Q, Qd, Qdd);
t2 = toc(tic2);

fprintf("Optimized inverse dynamics function took %.3g seconds per state.\n", t2/N);

tic3 = tic;
Qdd_psdm = PSDM.forwardDynamics(E, P, Theta, Q, Qd, tau_psdm);
t3 = toc(tic3);

fprintf("Default forward dynamics function took %.3g seconds per state.\n", t3/N);

tic4 = tic;
Qdd_psdm_opt = PSDM_forwardDynamics_twolink_mex(Q, Qd, tau_psdm);
t4 = toc(tic4);

fprintf("Optimized forward dynamics function took %.3g seconds per state.\n", t4/N);

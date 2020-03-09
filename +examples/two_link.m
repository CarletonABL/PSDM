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

fprintf('Max error: %.5g\n', max(abs(tau_rne - tau_psdm), [], 'all'));

%% Faster real time code

% Generate some random test states
N = 10e5;
DOF = 2;
Q = (rand(DOF, N)-0.5) * (2*pi);
Qd = (rand(DOF, N)-0.5) * (10);
Qdd = (rand(DOF, N)-0.5) * (50);

tic1 = tic;
tau_psdm = PSDM.inverseDynamics(E, P, Theta, Q, Qd, Qdd);
t1 = toc(tic1);

fprintf("Default function took %.3g seconds per state.\n", t1/N);

%% Generate dedicated function

% Make temporary director

addpath(fullfile(utilities.PSDMDir, 'temp'));
filename = fullfile( utilities.PSDMDir, 'temp', 'PSDM_inverseDynamics_twolink.m');
doMex = true; % Whether or not to mex result for speed
doPar = true; % Whether to evaluate states in parallel
PSDM.makeInverseDynamics(filename, E, P, Theta, ...
    'do_mex', doMex, 'parallel', doPar);


tic2 = tic;
tau_psdm_opt = PSDM_inverseDynamics_twolink_mex(Q, Qd, Qdd);
t2 = toc(tic2);

fprintf("Optimized function took %.3g seconds per state.\n", t2/N);
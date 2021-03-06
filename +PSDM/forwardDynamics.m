function [Qdd, tau_induced, D] = forwardDynamics(E, P, Theta, Q, Qd, tau)
    % FORWARDDYNAMICS Evaluates the forward dynamics for a state Q, Qd and
    % joint torques tau, given a pseudo-symbolic representation E, P and
    % Theta.
    %
    % Qdd = forwardDynamics(E, P, Theta, Q, Qd, tau)
    %
    % Note, this file can be mex'ed for speed. Run PSDM.make with a valid
    % matlab compiler!
    %
    % For a given tuple {E, P, Theta} this function will be fairly slow,
    % even when compiled. To make a faster function, use the function
    %   PSDM.makeForwardDynamics(filename, E, P, Theta).
    %
    % This procedurally generates a function specific to the robot which
    % runs much faster and can also be mex'ed for additional speed.
    %
    % See also: PSDM.inverseDynamics, PSDM.makeForwardDynamics.
    
    %% Parse inputs
    
    DOF = size(Q, 1);
    N = size(Q, 2);
       
    % Input checking
    assert(size(E, 1) == DOF*5, "E and Q are not congruently sized. Q should be a DOF x N matrix.");
    assert(all(size(Q) == size(Qd), 2) && ...
           all(size(Q) == size(tau), 2), ...
           "Q, Qd and tau must all be DOFxN matrices!");
    assert(size(P, 2) == size(Theta, 1), "P and Theta are not congruently sized!");
    assert(size(E, 2) == size(P, 1), "P and E are not congruently sized!");
    
    %% Start function
    
    % Need to extract the acceleration terms from E, into a mask
    accelMask = E((1:DOF)+4*DOF, :) > 0;
    
    % M x 1 vector of which Y functions are acceleration functions
    accelMaskAll = any(accelMask, 1);
    
    % Evaluate all terms with Qdd = ones.
    Qdd_sample = ones(size(Q));
    
    % Get Yp regressor
    Yp = PSDM.generateYp(Q, Qd, Qdd_sample, E);
    
    % Get induced torques from coriolis and gravity by evaluating Yp and P
    % with only the terms NOT associated with acceleration. We call these
    % the "induced" torques.
    
    % Pre-combine Theta with the reduction matrix. First, remove all rows
    % associated with acceleration terms.
    P_induced = P(~accelMaskAll, :, :);
    
    % Pre-multiply Theta
    PTheta_induced = zeros( size(P_induced, 1), DOF );
    for i = 1:DOF
        PTheta_induced(:, i) = P_induced(:, :, i) * Theta;
    end
    Phi_induced = squeeze( utilities.blockprod( P_induced, Theta ) );
    
    % Calculate induced torques
    tau_induced = (Yp(:, ~accelMaskAll) * Phi_induced)';
    
    % Build mass matrix by evaluating torques with Qdd = ones at only the
    % rows of P and columns of Yp associated with acceleration terms.
    Paccel = P(accelMaskAll, :, :);
    Yp_accel = Yp(:, accelMaskAll);
    accelMask_trim = accelMask(:, accelMaskAll);
    
    % Find D (this function runs the id in a loop but will parallelize if
    % the computer supports it).
    D = permute(...
            utilities.iparfor( ...
                @(i) buildDColumn(Paccel(accelMask_trim(i, :), :, :), ...
                          Theta, ...
                          Yp_accel(:, accelMask_trim(i, :))), ...
                DOF, ...
                [DOF, N], false), ...
            [1 3 2]);
    
    % Solve for Qdd
    Qdd = permute( ...
            utilities.iparfor( ...
                @(i) D(:, :, i) \ (tau(:, i) - tau_induced(:, i)), ...
                N, ...
                [DOF, 1], false), ...
            [1 3 2]);
    
end

function Dcol = buildDColumn(Paccel_i, Theta, Yp_accel_i)
    % Builds up the ith column of the mass matrix D

    Phi_accel_i = permute( utilities.blockprod( Paccel_i, Theta ), [1 3 2] );
    % Get torques with Qdd_i = 1, resulting in D.
    Dcol = (Yp_accel_i * Phi_accel_i)';

end
function Qdd = forwardDynamics(E, P, Theta, Q, Qd, tau)
    % FORWARDDYNAMICS Evaluates the forward dynamics for a state Q, Qd and
    % joint torques tau, given a pseudo-symbolic representation E, P and
    % Theta.
    
    DOF = size(Q, 1);
    N = size(Q, 2);
    
    % Input checking
    assert(size(E, 1) == DOF*5, "E and Q are not congruently sized. Q should be a DOF x N matrix.");
    assert(all(size(Q) == size(Qd)) && ...
           all(size(Q) == size(tau)), ...
           "Q, Qd and tau must all be DOFxN matrices!");
    assert(size(P, 2) == size(Theta, 1), "P and Theta are not congruently sized!");
    assert(size(E, 2) == size(P, 1), "P and E are not congruently sized!");
    
    doPar = utilities.autoPar;
    
    % Need to extract the acceleration terms from E, into a mask
    accelMask = E((1:DOF)+4*DOF, :) > 0;
    
    accelMaskAll = any(accelMask, 1);
    
    % Evaluate terms with Qdd = ones.
    Qdd_sample = ones(size(Q));
    
    % Get Yp regressor
    Yp = PSDM.genTermValues(Q, Qd, Qdd_sample, E);
    
    % Get induced torques from coriolis and gravity
    P_induced = P(~accelMaskAll, :, :);
    
    PTheta_induced = zeros( size(P_induced, 1), DOF );
    for i = 1:DOF
        PTheta_induced(:, i) = P_induced(:, :, i) * Theta;
    end
    
    % Calculate induced torques
    tau_induced = (Yp(:, ~accelMaskAll) * PTheta_induced)';
    
    % Build of mass matrix
    Paccel = P(accelMaskAll, :, :);
    Yp_accel = Yp(:, accelMaskAll);
    accelMask_trim = accelMask(:, accelMaskAll);
    
    D = zeros(DOF, DOF, N);
    
    if doPar
        parfor i = 1:DOF
            % Iterate through joint acceleration being considered (columns of D)
            D(:, i, :) = buildDColumn(Paccel(accelMask_trim(i, :), :, :), ...
                                      Theta, ...
                                      Yp_accel(:, accelMask_trim(i, :)), ...
                                      DOF);
        end
    else
        for i = 1:DOF
            % Iterate through joint acceleration being considered (columns of D)
            D(:, i, :) = buildDColumn(Paccel(accelMask_trim(i, :), :, :), ...
                                      Theta, ...
                                      Yp_accel(:, accelMask_trim(i, :)), ...
                                      DOF);
        end
    end
    
    % Solve for Qdd
    Qdd = zeros(DOF, N);
    if doPar
        parfor i = 1:N
            Qdd(:, i) = D(:, :, i) \ (tau(:, i) - tau_induced(:, i));
        end
    else
        for i = 1:N
            Qdd(:, i) = D(:, :, i) \ (tau(:, i) - tau_induced(:, i));
        end
    end   
    
end

function Dcol = buildDColumn(Paccel_i, Theta, Yp_accel_i, DOF)
    % Builds up the ith column of the mass matrix D

    PTheta_accel_i = zeros( size(Paccel_i, 1), DOF );

    for j = 1:DOF
        % Iterate through joint torque being considered (rows of D)
        PTheta_accel_i(:, j) = Paccel_i(:, :, j) * Theta;
    end
    
    Dcol = permute((Yp_accel_i * PTheta_accel_i)', [1 3 2]);

end
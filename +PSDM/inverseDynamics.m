function tau = inverseDynamics(E, P, Theta, Q, Qd, Qdd)
    % INVERSEDYNAMICS Evaluates the inverse dynamics at a state Q, Qd, Qdd
    % from a pseudo-symbolic representation E, P and Theta
    %
    %   tau = inverseDynamics(E, P, Theta, Q, Qd, Qdd)
    %
    % Note, this file can be mex'ed for speed. Run PSDM.make with a valid
    % matlab compiler!
    %
    %
    % For a given tuple {E, P, Theta} this function will be fairly slow,
    % even when compiled. To make a faster function, use the function
    %   PSDM.makeInverseDynamics(filename, E, P, Theta).
    %
    % This procedurally generates a function specific to the robot which
    % runs much faster and can also be mex'ed for additional speed.
    %
    % See also: PSDM.forwardDynamics, PSDM.makeInverseDynamics.
    
    %% Parse inputs
    
    DOF = size(Q, 1);
    
    % Input checking
    assert(size(E, 1) == DOF*5, "E and Q are not congruently sized. Q should be a DOF x N matrix.");
    assert(all(size(Q) == size(Qd)) && ...
           all(size(Q) == size(Qdd)), ...
           "Q, Qd and Qdd must all be DOFxN matrices!");
    assert(size(P, 2) == size(Theta, 1), "P and Theta are not congruently sized!");
    assert(size(E, 2) == size(P, 1), "P and E are not congruently sized!");
    
    %% Run mex, if possible
    
    c = PSDM.config;
    if coder.target('matlab') && c.allow_mex
         try
            tau = PSDM.inverseDynamics_mex(E, P, Theta, Q, Qd, Qdd);
            return; 
         catch
             warning("PSDM is not compiled! This code will run slowly without compilation. Recommend running PSDM.make");
         end
    end
    
    
    %% Start Function
    
    DOF = size(Q, 1);
    
    Yp = PSDM.generateYp(Q, Qd, Qdd, E);
    
    % Reduce theta and P together
    PTheta = zeros( size(P, 1), DOF );
    for i = 1:DOF
        PTheta(:, i) = P(:, :, i) * Theta;
    end
    
    % Solve for torques
    tau = (Yp * PTheta)';
     
end
function [tau, Yb] = inverseDynamics(E, P, Theta, Q, Qd, Qdd)
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
    if coder.target('matlab') && c.use_mex
         try
            [tau, Yb] = PSDM.inverseDynamics_mex(E, P, Theta, Q, Qd, Qdd);
            return; 
         catch e
             warning("PSDM is not compiled! This code will run slowly without compilation. Recommend running PSDM.make");
             disp(e)
         end
    end
    
    
    %% Start Function
    
    % Get Yp
    Yp = PSDM.generateYp(Q, Qd, Qdd, E);
    Yb = permute( utilities.blockprod(Yp, P) , [3 2 1]);
    
    % Phi_b = permute(utilities.blockprod(P, Theta), [1 3 2]);
    
    % Solve for torques
    tau = squeeze( utilities.blockprod( Yb, Theta ) );
     
end
function tau = inverseDynamics(E, P, Theta, Q, Qd, Qdd)
    % INVERSEDYNAMICS Evaluates the inverse dynamics at a state Q, Qd, Qdd
    % from a pseudo-symbolic representation E, P and Theta
    
    % Run mex, if possible
    c = PSDM.config;
    if coder.target('matlab') && c.allow_mex
         try
            tau = PSDM.inverseDynamics_mex(E, P, Theta, Q, Qd, Qdd);
            return; 
         catch
             warning("PSDM is not compiled! This code will run slowly without compilation. Recommend running PSDM.make");
         end
    end
    
    doPar = utilities.autoPar;
    
    DOF = size(Q, 1);
    
    Yp = PSDM.genTermValues(Q, Qd, Qdd, E);
    
    % Reduce theta and P together
    PTheta = zeros( size(P, 1), DOF );
    if doPar
        parfor i = 1:DOF
            PTheta(:, i) = P(:, :, i) * Theta;
        end
    else
        for i = 1:DOF
            PTheta(:, i) = P(:, :, i) * Theta;
        end
    end
    
    % Solve for torques
    tau = (Yp * PTheta)';
     
end
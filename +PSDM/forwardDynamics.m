function tau = forwardDynamics(E, P, Theta, Q, Qd, Qdd)
    % FORWARDDYNAMICS Evaluates the forward dynamics at a state Q, Qd, Qdd
    % from a pseudo-symbolic representation map, P and Theta
    
    % Run mex, if possible
    if coder.target('matlab')
         try
            tau = PSDM.forwardDynamics_mex(E, P, Theta, Q, Qd, Qdd);
            return; 
         catch
             warning("PSDM is not compiled! This code will run slowly without compilation. Recommend running PSDM.make");
         end
    end
    
    DOF = size(Q, 1);
    
    Yp = PSDM.genTermValues(Q, Qd, Qdd, E);
    
    PTheta = zeros( size(P, 1), DOF );
    for i = 1:DOF
        PTheta(:, i) = P(:, :, i) * Theta;
    end
    
    tau = (Yp * PTheta)';
     
end
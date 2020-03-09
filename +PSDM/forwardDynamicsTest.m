function tau = forwardDynamicsTest(Eexp, PTheta, Q, Qd, Qdd)
    % FORWARDDYNAMICS Evaluates the forward dynamics at a state Q, Qd, Qdd
    % from a pseudo-symbolic representation map, P and Theta
    
    % Run mex, if possible
    if coder.target('matlab') && false
         try
            tau = PSDM.forwardDynamicsTest_mex(Enum, P, Theta, Q, Qd, Qdd);
            return; 
         catch
             warning("PSDM is not compiled! This code will run slowly without compilation. Recommend running PSDM.make");
         end
    end
    
    N = size(Q, 2);
    tau = zeros(size(Q));
    
    for i = 1:N
        tau(:, i) = getTorques(Q(:, i), Qd(:, i), Qdd(:, i), PTheta, Eexp);
    end
     
end

function tau = getTorques( Q, Qd, Qdd, PTheta, Eexp )

    cQ = cos(Q);
    
    [N, M] = size(Eexp);
    gamma = repmat(vertcat( sin(Q), cQ, Qd, Qdd, cQ, Qd ), [1, M]);
    o = ones(1, M);
    
    Yp = o;
    
    parfor i = 1:N
        gi = o;
        gi(Eexp(i, :)) = gamma(i);
        Yp = Yp .* gi;
    end
    
%     Yp = prod(g, 1);
    
%     Yp = ones(1, M);
%     
%     parfor i = 1:M
%         Ei = Eexp(:, i);
%         Yp(i) = prod(gamma(Ei));
%     end
%     coder.varsize('Yp', [1 10000], [0 1]);
%     Yp = zeros(1, M);
%     for m = 1:M
%         ind = Eexp(:, m);
%         gammat = gamma .* ind;
%         Yp(1, m) = prod( gammat );
%     end
    
    tau = (Yp * PTheta)';
    
end
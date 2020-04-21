function [Et, Pt, interim] = reduceModelComplexity(DH, X, g, E, P, varargin)
    % REDUCEMODELCOMPLEXITY Reduces the complexity of a model E, P by
    % eliminating terms which contribute minimally to the dynamic model.
    
    %% Parse arguments
    
    DOF = size(DH, 1);
    m = size(E, 2);
    ell = size(P, 2);
    
    p = inputParser;
    p.addOptional('max_relative_error', 0.001); % 1%
    p.addOptional('qlim', repmat([-pi pi], [DOF, 1]));
    p.addOptional('Ntests', m*20);
    p.addOptional('qd_lim', 1);
    p.addOptional('qdd_lim', 10);
    p.addOptional('tol', 1e-9);
    p.addOptional('v', true);
    p.parse(varargin{:});
    opt = p.Results;
    
    
    %% Function start
    
    utils.vprint(opt.v, "Eliminating terms with average effects less than %.4g%%...\n", opt.max_relative_error*100);
    
    Ntests = round(opt.Ntests);
    
    % Generate many terms with typical values
    utils.vprint(opt.v, "\tGenerating test set (N = %d)...\n", int32(Ntests));
    [Q, Qd, Qdd, tau] = ...
        generateSamples(DH, X, g, Ntests, opt.qlim, opt.qd_lim, opt.qdd_lim);
    Yp = PSDM.generateYp(Q, Qd, Qdd, E);

    utils.vprint(opt.v, "\tRegressing terms...\n");
    
    % Do regression (in parts)
    Yb = zeros( Ntests, ell, DOF );
    for i = 1:DOF
        Yb(:, :, i) = Yp * P(:, :, i);
    end
    
    Yb_stack = utilities.vertStack(Yb);
    tau_stack = utilities.vertStack(tau, 2);
    
    % Do regression
    Theta = linsolve(Yb_stack, tau_stack);
    
    PTheta_perm = permute( utils.blockprod( P, Theta ), [2 1 3]);

    % Average contribution of each terms over all terms
    % tau_Y = Yp .* PTheta_perm;
    
    % Average contribution of each terms over all terms
    norm_tauY = zeros(Ntests, m);
    
    for i = 1:m
        norm_tauY(:, i) = vecnorm(Yp(:, i) .* PTheta_perm(:, i, :), 2, 3);
    end
        
    % Get ratios
    ratios = norm_tauY ./ vecnorm(tau, 2, 2);
    rmax = mean( ratios, 1 );
    
    % Get ratios
    % rmax = mean( vecnorm(tau_Y, 2, 3) ./ vecnorm(tau, 2, 2) );
    
    % Sort terms by relative size, then eliminate a cumulative amount equal
    % to the requested relative tolerance.
    [rmax_sort, sortInd] = sort(rmax);
    rmax_cumsum = cumsum(rmax_sort);
    
    maskSort = rmax_cumsum > opt.max_relative_error;
    mask(sortInd) = maskSort;
   
    % Eliminate terms
    Et = E(:, mask);
    Pt1 = P(mask, :, :);
    utils.vprint(opt.v, "\tDone. %d terms eliminated, %d terms remaining.\n", int32(sum(~mask)), int32(sum(mask)));
    
    % Find out if any inertial parameters are redundant
    mask2 = max(abs(Pt1), [], [1 3]) > opt.tol;
    
    Pt = Pt1(:, mask2, :);
    utils.vprint(opt.v && any(~mask2), "\tBase inertial parameter size reduced from %d to %d.\n", int32(size(Pt1, 2)), int32(sum(mask2)));
    
    % Estimate error of new model
    checkModel(DH, X, g, E, P, Et, Pt, Ntests/5, opt);
    
    if nargout > 2
        % Return interim results, if requested
        interim = struct('rmax', rmax, ...
                         'ratios', ratios, ...
                         'tau', tau, ...
                         'Theta', Theta);
    end
                         
    
end

function [Q, Qd, Qdd, tau] = ...
    generateSamples(DH, X, g, Ntests, qlim, qd_lim, qdd_lim)
    % Generates sample points Q, Qd, Qdd and the corresponding torques for
    % random poses in qlim and near qd_lim, qdd_lim.

    DOF = size(DH, 1);
    Ntests = round(Ntests);
    
    Q = RU.randomPose( qlim, Ntests );
    Qd = qd_lim - rand(DOF, Ntests) * 0.1;
    Qdd = qdd_lim - rand(DOF, Ntests) * 0.1;
    
    tau = PSDM.inverseDynamicsNewton(DH, X, Q, Qd, Qdd, int8(2), g)';
    
end


function checkModel(DH, X, g, E, P, Et, Pt, Ntests, opt)

    Ntests = round(Ntests);

    % Generate validation set
    [Q, Qd, Qdd, tau] = ...
        generateSamples(DH, X, g, Ntests, opt.qlim, opt.qd_lim, opt.qdd_lim);
    
    % Find regressors
    Theta = PSDM.X2Theta(DH, X, g, E, P, Ntests);
    Thetat = PSDM.X2Theta(DH, X, g, Et, Pt, Ntests);
    
    % Do inverse dynamics, check error
    tau_1 = PSDM.inverseDynamics(E, P, Theta, Q, Qd, Qdd);
    tau_2 = PSDM.inverseDynamics(Et, Pt, Thetat, Q, Qd, Qdd);
    
    tau_norm = mean(vecnorm(tau', 2, 1));
    
    % Get MSE
    e1 = vecnorm(tau' - tau_1, 2, 1);
    e2 = vecnorm(tau' - tau_2, 2, 1);
    
    utils.vprint(opt.v, "Original model had MSE of %.5g / mean torque norm %.5g (relative %.5g%%).\n", mean(e1), tau_norm, mean(e1)/tau_norm*100);
    utils.vprint(opt.v, "Condense model had MSE of %.5g / mean torque norm %.5g (relative %.5g%%).\n", mean(e2), tau_norm, mean(e2)/tau_norm*100);
    
end
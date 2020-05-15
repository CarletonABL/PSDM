function [Et, Pt, interim] = reduceModelComplexity(E, P, varargin)
    % REDUCEMODELCOMPLEXITY Reduces the complexity of a model E, P by
    % eliminating terms which contribute minimally to the dynamic model.
    
    %% Parse arguments
    
    DOF = size(DH, 1);
    m = size(E, 2);
    ell = size(P, 2);
    
    p = inputParser;
    p.addOptional('mode', 'rel_error')
    p.addOptional('max_relative_error', 0.001); % 1%
    p.addOptional('percent_reduction', 10); % 10
    p.addOptional('qlim', repmat([-pi pi], [DOF, 1]));
    p.addOptional('Ntests', m*2);
    p.addOptional('qd_lim', 1);
    p.addOptional('qdd_lim', 10);
    p.addOptional('tolerance', 1e-9);
    p.addOptional('check_accuracy', true);
    p.addOptional('verbose', true);
    p.addOptional('interim', []); % If given, will skip the initial regression steps and use this instead.
    [robot, opt] = parseArgs(p, varargin{:});
    opt.tol = opt.tolerance;
    opt.verbose = opt.v;
    
    %% Function start
    
    if strcmp(opt.mode, 'rel_error')
        utils.vprint(opt.v, "Eliminating terms with average effects less than %.4g%%...\n", opt.max_relative_error*100);
    else
        utils.vprint(opt.v, "Eliminating terms %d%% of terms...\n", opt.percent_reduction);
    end
    Ntests = round(opt.Ntests);
    
    % Generate many terms with typical values
    if isempty(opt.interim)
        
        utils.vprint(opt.v, "\tGenerating test set (N = %d)...\n", int32(Ntests));
        
        % Generate samples
        [Q, Qd, Qdd, tau] = generateSamples(robot, Ntests, opt.qlim, opt.qd_lim, opt.qdd_lim);
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
        norm_tauY = zeros(Ntests, m);
        for i = 1:m
            norm_tauY(:, i) = vecnorm(Yp(:, i) .* PTheta_perm(:, i, :), 2, 3);
        end

        % Get ratios
        ratios = norm_tauY ./ vecnorm(tau, 2, 2);
        rmax = mean( ratios, 1 );
        
    else
        % If interim is defined, just use that instead
        rmax = opt.interim.rmax;
        ratios = opt.interim.ratios;
        tau = opt.interim.tau;
        Theta = opt.interim.Theta;
    end
    
    % Get ratios
    % rmax = mean( vecnorm(tau_Y, 2, 3) ./ vecnorm(tau, 2, 2) );
    [rmax_sort, sortInd] = sort(rmax);
    
    if strcmp(opt.mode, 'rel_error')
        % Sort terms by relative size, then eliminate a cumulative amount equal
        % to the requested relative tolerance.
        rmax_cumsum = cumsum(rmax_sort);

        maskSort = rmax_cumsum > opt.max_relative_error;
        mask(sortInd) = maskSort;
        
    elseif strcmp(opt.mode, 'percent')
        
        p = size(E, 2);
        p2 = round(p * (opt.percent_reduction/100));
        mask = zeros(1, p, 'logical');
        mask(sortInd) = (1:p)>=p2;
        
    end
   
    % Eliminate terms
    Et = E(:, mask);
    Pt1 = P(mask, :, :);
    utils.vprint(opt.v, "\tDone. %d terms eliminated, %d terms remaining.\n", int32(sum(~mask)), int32(sum(mask)));
    
    % Find out if any inertial parameters are redundant
    mask2 = max(abs(Pt1), [], [1 3]) > opt.tol;
    
    Pt = Pt1(:, mask2, :);
    utils.vprint(opt.v && any(~mask2), "\tBase inertial parameter size reduced from %d to %d.\n", int32(size(Pt1, 2)), int32(sum(mask2)));
    
    if opt.check_accuracy
        % Estimate error of new model
        checkModel(DH, X, g, E, P, Et, Pt, Ntests/5, opt);
    end
    
    if nargout > 2
        % Return interim results, if requested
        interim = struct('rmax', rmax, ...
                         'ratios', ratios, ...
                         'tau', tau, ...
                         'Theta', Theta);
    end
                         
    
end

function [Q, Qd, Qdd, tau] = ...
    generateSamples(robot, Ntests, qlim, qd_lim, qdd_lim)
    % Generates sample points Q, Qd, Qdd and the corresponding torques for
    % random poses in qlim and near qd_lim, qdd_lim.

    DOF = size(DH, 1);
    Ntests = round(Ntests);
    
    Q = RU.randomPose( qlim, Ntests );
    Qd = mean(qd_lim) + (rand(DOF, Ntests)-.5) * diff(qd_lim);
    Qdd = mean(qdd_lim) + (rand(DOF, Ntests)-.5) * diff(qdd_lim);
    
    tau = robot.IDfunc(Q, Qd, Qdd, robot.X);
        
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
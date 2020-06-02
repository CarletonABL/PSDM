function [Et, Pt, interim] = reduceModelComplexity(E, P, varargin)
    % REDUCEMODELCOMPLEXITY Takes a model E, P and reduces its complexity
    % through a significance analysis.
    %
    % [E, P] = PSDM.deriveModel(E, P, DH, g, X): Performs complexity reduction
    %   on a model E, P, for a robot with DH parameters, gravity vector,
    %   and X as defined.
    %
    % [E, P] = PSDM.deriveModel(E, P, inverseDynamicsFunc, jointTypes, X):
    %   If the third input to the function is an anonymous function handle,
    %   then reduction is done using this as the sampling function.
    %
    % [E, P] = PSDM.deriveModel(____, varargin) use any of the calling
    %   syntaxes above, but also specify additional model parameters as
    %   defined below.
    %   
    % INPUTS:
    %   - E: A p x 5*DOF uint8 exponent matrix of the PSDM model.
    %   - P: A p x ell x DOF page matrix of the DOF reduction matrices Pi
    %       for the model.
    %   - DH: a DOFx4-6 array of the DH params in the following order: 
    %             [a_1  alpha_1    d_1   theta_1    lt_1    q_sign_1;
    %              :       :        :       :         :         :     
    %              an   alpha_n    d_n   theta_n    lt_n    q_sign_n];
    %           lt stands for "Link types", and should be true if joint is a
    %           prismatic joint, false otherwise. If column 5 is missing, will
    %           assume revolute (0).
    %           q_sign is a link direction number (-1 or 1). Allows you to
    %           change the sign of the joint variable. If column 6 is
    %           missing, will assume 1.
    %
    %   - g: A 3 x 1 gravity vector (must be unit size!!). Points upwards
    %        against gravity. Default: [0;0;1] (z axis points upwards from
    %        gravity).
    %
    %   - X: A DOF x 10 matrix of the mass properties of the
    %        manipulator:
    %        [m_1  rcx_1  rcy_1  rcz_1  Ixx_1  Iyy_1  Izz_1  Ixy_1  Ixz_1  Iyz_1;
    %          :     :      :      :       :     :      :      :     :       :
    %         m_n  rcx_n  rcy_n  rcz_n  Ixx_n  Iyy_n  Izz_n  Ixy_n  Ixz_n  Iyz_n]
    %        To successfully derive the PSDM model, this does not need to
    %        be correct. It is only used to determine which elements of the
    %        model are identically zero, and can be ignored.
    %        Default: ones(DOF, 10)
    %
    %   - inverseDynamicsFunc: An anonymous function which can be called
    %        as:
    %           tau = inverseDynamicsFunc(Q, Qd, Qdd, X, g_scale)
    %        where Q, Qd, Qdd and tau are DOFxN matrices (N = number of
    %        samples), and X is a DOFx10 matrix as defined above. g_scale
    %        is a scalar which "scales" gravity. If g_scale = 0, then the
    %        model should act as if there was no gravity, and if 
    %        g_scale = 1 then the model should act as if there is full
    %        gravity.
    %
    %   - jointTypes: a DOFx1 logical vector of the types of joints, with
    %        with the following mapping: false => revolute, 
    %        true => prismatic.
    %
    % OUPUTS:
    %   - Ehat: A reduced model E.
    %   - Phat: The corresponding page matrix
    %   - interim: A structure with some interim variables, if required.
    %
    % Name-Value pairs:
    %   - mode: The reduction mode. Options :
    %       * 'rel_error' (default): Functions are eliminated based on a
    %           given requirement of maximum relative error (the
    %           'max_relative_error' option).
    %       * 'percent': A fixed percentage of the functions are
    %           eliminated, as given by the 'percent' argument.
    %   - max_relative_error: A number from 0 to 1, indicating the
    %       acceptable amount of relative error in the reduced model. Only
    %       used for the 'rel_error' mode. Default: 0.001.
    %   - percent_reduction: A number from 0 to 100 indicating the required
    %       reduction in size (only used in percent mode). Default: 10.
    %   - qlim: The limits of the joint variables q of the robot, for the
    %       application, in rad for revolute joints.
    %       Default: repmat([-pi pi], [DOF, 1])
    %   - qd_lim: The limits of the joint speeds of the robot, in rad/s for
    %       revolute joints. Default: 1.
    %   - qdd_lim: The limits of the joint accelerations of the robot, in
    %       rad/s^2 for revolute joints. Default: 10.
    %   - Ntests: The number of tests to run. Default: size(E, 2)*2
    %   - tolerance: The tolerance used in reducing the Theta vector.
    %       Default: 1e-10.
    %   - verbose: Whether or not to output information about the
    %       algorithm. Default: true.
    %   - check_accuracy: If set to true, will double check the reduction
    %       with a secondary analysis, after removing functions. 
    %       Default: true. 
    %   - interim: Can feed back the interim output a run to perform
    %       another reduction with the same analysis. Only useful for
    %       repeated analyses – ignore this otherwise.
    
    
    %% Parse arguments
    DOF = size(E, 1)/5;
    m = size(E, 2);
    ell = size(P, 2);
    
    p = inputParser;
    p.addOptional('mode', 'rel_error')
    p.addOptional('max_relative_error', 0.001); % 1%
    p.addOptional('percent_reduction', 10); % 10
    p.addOptional('Ntests', m*2);
    p.addOptional('qlim', repmat([-pi pi], [DOF, 1]));
    p.addOptional('qd_lim', 1);
    p.addOptional('qdd_lim', 10);
    p.addOptional('tolerance', 1e-10);
    p.addOptional('check_accuracy', true);
    p.addOptional('verbose', true);
    p.addOptional('interim', []); % If given, will skip the initial regression steps and use this instead.
    [robot, opt] = parseArgs(p, varargin{:});
    opt.tol = opt.tolerance;
    opt.v = opt.verbose;
    
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
        checkModel(robot, E, P, Et, Pt, Ntests/5, opt);
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

    DOF = robot.DOF;
    Ntests = round(Ntests);
    
    Q = RU.randomPose( qlim, Ntests );
    Qd = mean(qd_lim) + (rand(DOF, Ntests)-.5) * diff(qd_lim);
    Qdd = mean(qdd_lim) + (rand(DOF, Ntests)-.5) * diff(qdd_lim);
    
    tau = robot.IDfunc(Q, Qd, Qdd, robot.X, 1);
        
end


function checkModel(robot, E, P, Et, Pt, Ntests, opt)

    Ntests = round(Ntests);

    % Generate validation set
    [Q, Qd, Qdd, tau] = ...
        generateSamples(robot, Ntests, opt.qlim, opt.qd_lim, opt.qdd_lim);
    
    % Find regressors
    Theta = PSDM.X2Theta(E, P, robot.DH,  robot.g, robot.X, 'Ntests', Ntests);
    Thetat = PSDM.X2Theta(Et, Pt, robot.DH, robot.g, robot.X,  'Ntests', Ntests);
    
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
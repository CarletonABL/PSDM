function [E, P] = deriveModel(varargin)
    % DERIVEMODEL Performs a dynamic ID using PSDM (Pseudo-Symbolic Dynamic
    % Modelling).
    %
    % [E, P] = PSDM.deriveModel(DH, g): Performs an ID on a manilator with 
    %   DH parameters and gravity vector defined by DH and g, respectively.
    %
    % [E, P] = PSDM.deriveModel(DH, g, X): Performs an ID and uses a matrix
    %   X of inertial parameters (defined below) to implicitely simplify the
    %   dynamics. Note, the exact values of X are not used here, but any
    %   element with a value equal identically to zero will be ignored in
    %   the derivation, resulting in simpler final equations.
    %
    % [E, P] = PSDM.deriveModel(inverseDynamicsFunc, jointTypes): If the
    %   first input to the function is an anonymous function handle, then
    %   the derivation will be done using this function as the sampling
    %   function. The second argument given must then by a list of joint
    %   types, as defined below.
    %
    % [E, P] = PSDM.deriveModel(@inverseDynamicsFunc, jointTypes, X): Same
    %   as the previous calling syntax, but also does implicit
    %   simplification based on the numerical values of X.
    %
    % [E, P] = PSDM.deriveModel(____, varargin) use any of the calling
    %   syntaxes above, but also specify additional model parameters as
    %   defined below.
    %   
    % INPUTS:
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
    %        Optionally, an 11th column may be added to represent the joint
    %        drive inertias/masses (units of kg or kgm^2).
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
    %   - E: A p x 5*DOF uint8 exponent matrix of the PSDM model.
    %   - P: A p x ell x DOF page matrix of the DOF reduction matrices Pi
    %       for the model.
    %
    % Name-Value pairs:
    %   - tolerance: A tolerance used in the ID. Any factors below this
    %           tolerance (in estimation of the torques) is ignored.
    %           Defaut: 1e-10.
    %   - verbose: A verbosity flag. Set to false to suppress output.
    %           Default: true.    
    %   - gravity_only: If true, the algorithm will ignore all acceleration
    %           and velocity effects.
    %           Default: false
    
    %% Parse arguments
    if coder.target('matlab')
        p = inputParser;
        p.addOptional('tolerance', 1e-10);
        p.addOptional('verbose', true);
        p.addOptional('gravity_only', false);
        [robot, opt] = parseArgs(p, varargin{:});
        opt.v = opt.verbose; opt.tol = opt.tolerance;
    else
        DH = varargin{1};
        g = varargin{2};
        DOF = size(DH, 1);
        if ~isempty(varargin{3})
            X = varargin{3}(1:DOF, 1:10);
        else
            X = ones(DOF, 10) + rand(DOF, 10)*0.2 - 0.1;
        end
        if ~isempty(varargin{4})
            tolerance = varargin{4}(1);
        else
            tolerance = 1e-10;
        end
        if ~isempty(varargin{5})
            verbose = varargin{5}(1);
        else
            verbose = true;
        end
        if ~isempty(varargin{6})
            gravity_only = varargin{6}(1);
        else
            gravity_only = false;
        end
        robot = struct('IDfunc', @(Qf, Qdf, Qddf, Xf, g_scale) PSDM.inverseDynamicsNewton(DH, Xf, Qf, Qdf, Qddf, int8(2), g*g_scale)', ...
                       'DOF', DOF, ...
                       'X', X, ...
                       'lt', logical(DH(:, 5)));
        opt.tolerance = tolerance;
        opt.verbose = verbose;
        opt.gravity_only = gravity_only;
        opt.v = verbose; opt.tol = tolerance;
    end          
    
    %% Start function
            
    % Start timeing
    coder.extrinsic('tic', 'toc')
    t = tic;
    
    % Derive gravity model
    [E_grav, P_grav] = PSDM.deriveGravityModel(robot, opt);
    
    if opt.gravity_only
        % If gravity can just simplify and exit here
        
        P = PSDM.simplifyReductionMatrix(P_grav, opt.tol);
        E = E_grav;
        return;
        
    end

    % Derive accel model
    [Ei_accel, Pi_accel] = PSDM.deriveAccelModel(robot, opt);

    % Derive velocity model
    [Ei_vel, Pi_vel] = PSDM.deriveVelocityModel(robot, Ei_accel, opt);
        
    % Output information, if required.
    utilities.vprint(opt.v, "Running final combining of terms:\n");
    
    % Combine models into a single term
    [E, P] = PSDM.combineModels(robot, ...
                {{E_grav}, Ei_accel, Ei_vel}, ...
                {{P_grav}, Pi_accel, Pi_vel}, ...
                'all', opt);

    % Simplify P matrices by normalizing each column by the value of the
    % first nonzero value in P.
    P = PSDM.simplifyReductionMatrix(P, opt.tol);
       
    % Output information, if required.
    utilities.vprint(opt.v, "\nRobot matching done. Took %.3g s total.\n", toc(t));

end



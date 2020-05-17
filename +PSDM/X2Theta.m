function Theta = X2Theta(E, P, varargin)
    % X2Theta For a given robot defined by DH_ext and with known 
    % inertial properties X, calculates the Theta
    % matrix appropriate for the map and P given, based on the kinematic
    % and inertial properties of the robot.
    %
    % Theta = PSDM.X2Theta(E, P, DH, g, X) Calculates Theta based on 10*ell
    %   tests, where ell = size(P, 2). Inverse dynamic sampling is done
    %   using Newton-Euler algorithm and the model defined by DH and g.
    %
    % 
    % Theta = PSDM.X2Theta(E, P, inverseDynamicsFunc, jointTypes, X) Calculates 
    %   Theta as above, but using the anonymous inverse dynamics function
    %   defined to sample the inverse dynamics. The second argument is a
    %   list of joint types, as defined below.
    %
    % Theta = PSDM.X2Theta(____, varargin) use either of the calling
    %   syntaxes above with the name value pairs below to tweak the
    %   behaviour.
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
    %   - Theta: A parameter vector theta which can be used to evaluate the
    %       dynamic model.
    %
    % Name-Value pairs:
    %   - Ntests: The number of samples to take to evaluate the parameter
    %       vector. More tests increases accuracy but takes more time.
    %       Default: 10 * ell = 10 * size(P, 2)
    
    % Parse arguments
    p = inputParser;
    p.addOptional('Ntests', round(size(P, 2) * 10));
    [robot, opt] = parseArgs(p, varargin{:});

    % Make some samples to test
    [Q, Qd, Qdd, tau] = PSDM.generateSamples(robot, opt.Ntests, 1);
    
    % Generate Yp and Yb
    Yp = PSDM.generateYp(Q, Qd, Qdd, E);
    Yb = utilities.blockprod(Yp, P);
    
    % Stack DOF of robot along regression dimension, then solve for theta
    Yb_stack = utilities.vertStack(Yb);
    tau_stack = utilities.vertStack(tau, 2);
    Theta = utilities.linsolve(Yb_stack, tau_stack);
    
end
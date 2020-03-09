function [E, P] = runID(DH_ext, X, g, tol, v, no_mex)
    % RUNID Performs a dynamic ID using PSDM (Pseudo-Symbolic Dynamic
    % Modelling).
    %
    % Inputs
    %   - DH_ext: a DOFx4-6 array of the DH params in the following order: 
    %             [a_1  alpha_1    d_1   theta_1    lt_1    q_sign_1;
    %              :       :        :       :         :         :     
    %           an   alpha_n    d_n   theta_n    lt_n    q_sign_n];
    %           lt stands for "Link types", and should be true if joint is a
    %           prismatic joint, false otherwise. If column 5 is missing, will
    %           assume revolute (0).
    %           q_sign is a link direction number (-1 or 1). Allows you to
    %           change the sign of the joint variable. If column 6 is
    %           missing, will assume 1.
    %   - X: A DOF x 10 matrix of the mass properties of the
    %        manipulator:
    %        [m_1  rcx_1  rcy_1  rcz_1  Ixx_1  Iyy_1  Izz_1  Ixy_1  Ixz_1  Iyz_1;
    %          :     :      :      :       :     :      :      :     :       :
    %         m_n  rcx_n  rcy_n  rcz_n  Ixx_n  Iyy_n  Izz_n  Ixy_n  Ixz_n  Iyz_n];
    %   - g: A 3 x 1 gravity vector (must be unit size!!). Points upwards
    %        against gravity. Default: [0;0;1] (z axis points upwards from
    %        gravity).
    %   - tol: A tolerance used in the ID. Any factors below this tolerance
    %          (in estimation of the torques) is ignored. Defaut: 1e-12.
    %   - v: A verbosity flag. Set to false to suppress output. Default:
    %           true.
    %
    %   See also PSDM.runID()

    if nargin < 3 || isempty(g)
        g = [0 0 1]';
        warning("Assuming g = [0;0;1]. Supply a proper g vector to suppress this warning.");
    end
    
    % Fill in inputs
    if nargin < 4 || isempty(tol)
        tol = 1e-12;
    end
    if nargin < 5 || isempty(v)
        v = true;
    end
    if nargin < 6 || isempty(no_mex)
        no_mex = false;
    end
    
    % Run mex, if possible
    if coder.target('matlab') && ~no_mex
        try
            [E, P] = PSDM.runID_mex(DH_ext, X, g, tol, v);
            return; 
        catch
           warning("PSDM is not compiled! This code will run slowly without compilation. Recommend running PSDM.make");
        end
    end
        
    % Check inputs
    assert(size(DH_ext, 2) == 6, "Invalid DH table");
    assert(all(DH_ext(:, 5) == 0), "This function does not currently work on prismatic joint robots");
    assert(all(abs(DH_ext(:, 6)) == 1), "Link sign column appears invalid. All numbers must be -1 or 1!");
    assert(size(X, 2) == 10 && size(X, 1) == size(DH_ext, 1), "X appears to be the wrong size!");
    assert(all(X(:, [1, 5, 6, 7]) >= 0, 'all'), "Negative masses and principle inertias Ixx Iyy Izz are not possible!")
    assert(sum(g.^2)==1, "Gravity vector is not a unit vector.")
    
    % Start timeing
    t = tic;
    
    % Gravity
    [Egrav, Pgrav] = PSDM.runIDGravity(DH_ext, X, g, tol, v);

    % Accel
    [Eaccel, Paccel] = PSDM.runIDAccel(DH_ext, X, tol, v);

    % Velocity
    [Evel, Pvel] = PSDM.runIDVel(DH_ext, X, Eaccel, tol, v);

    % Final combinations
    utilities.vprint(v, "Running final combining of terms:\n");
    
    % Do final combination of regression terms
    [E, P] = PSDM.combineTerms(DH_ext, X, g, ...
        {Egrav, Eaccel, Evel}, ...
        {Pgrav, Paccel, Pvel}, ...
        'all', tol, v);
    
    utilities.vprint(v, "\nRobot matching done. Took %.3g s total.\n", toc(t));

end
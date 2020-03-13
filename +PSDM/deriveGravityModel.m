function [Ep_grav, P_grav] = deriveGravityModel(DH_ext, g_in, X_in, tol_in, v_in)
    % RUNIDGRAVITY Performs a dynamic ID using PSDM (Pseudo-Symbolic Dynamic
    % Modelling), but only for the gravity terms.
    %
    % [Ep_grav, P_grav] = deriveGravityModel(DH_ext, g, X, tol, v)
    %
    % Inputs
    %   - DH_ext: a DOFx4-6 array of the DH params in the following order: 
    %             [a_1  alpha_1    d_1   theta_1    lt_1    q_sign_1;
    %               :      :        :       :         :         :     
    %              a_n  alpha_n    d_n   theta_n    lt_n    q_sign_n];
    %           lt stands for "Link types", and should be true if joint is a
    %           prismatic joint, false otherwise. If column 5 is missing, will
    %           assume revolute (0).
    %           q_sign is a link direction number (-1 or 1). Allows you to
    %           change the sign of the joint variable. If column 6 is
    %           missing, will assume 1.
    %   - g: A 3 x 1 gravity vector (must be unit size!!). Points upwards
    %        against gravity. Default: [0;0;1] (z axis points upwards from
    %        gravity).
    %   - X: A DOF x 10 matrix of the mass properties of the
    %        manipulator:
    %        [m_1  rcx_1  rcy_1  rcz_1  Ixx_1  Iyy_1  Izz_1  Ixy_1  Ixz_1  Iyz_1;
    %          :     :      :      :       :     :      :      :     :       :
    %         m_n  rcx_n  rcy_n  rcz_n  Ixx_n  Iyy_n  Izz_n  Ixy_n  Ixz_n  Iyz_n]
    %        To successfully derive the PSDM model, this does not need to
    %        be correct. It is only used to determine which elements of the
    %        model are identically zero, and can be ignored.
    %        Default: ones(DOF, 10)
    %   - tol: A tolerance used in the ID. Any factors below this tolerance
    %          (in estimation of the torques) is ignored. Defaut: 1e-12.
    %   - v: A verbosity flag. Set to false to suppress output. Default:
    %           true.
    %
    % OUTPUTS:
    %   - E: THe 4DOF x M PSDM exponent matrix associated with gravity.
    %   - P: The reduction matrix P
    %
    %   See also PSDM.deriveModel()
    
    %% Parse arguments
    
    DOF = size(DH_ext, 1);

    % Parse g
    if nargin < 2 || isempty(g_in)
        g = [0 0 1]';
        if coder.target('matlab')
            warning("Assuming g = [0;0;1]. Supply a proper g vector to suppress this warning.");
        end
    else
        g = g_in;
    end
    
    % Parse X
    if nargin < 3 || isempty(X_in)
        X = ones(DOF, 10);
    else
        X = X_in;
    end
    
    % parse tolerance
    if nargin < 4 || isempty(tol_in)
        tol = 1e-11;
    else
        tol = tol_in;
    end
    
    % Parse verbosity
    if nargin < 5 || isempty(v_in)
        v = true;
    else
        v = v_in;
    end
    
    % If possible, run mex file
    c = PSDM.config;
    if coder.target('matlab') && c.allow_mex
        try
            [Ep_grav, P_grav] = PSDM.deriveGravityModel_mex(DH_ext, g, X, tol, v);
            return; 
        catch
            warning("PSDM is not compiled! PSDM.deriveGravityModel will run slowly without compilation. Recommend running PSDM.make");
        end
    end
    
    % Check inputs
    assert(size(DH_ext, 2) == 6, "Invalid DH table");
    assert(all(abs(DH_ext(:, 6)) == 1), "Link sign column appears invalid. All numbers must be -1 or 1!");
    assert(size(X, 2) == 10 && size(X, 1) == size(DH_ext, 1), "X appears to be the wrong size!");
    assert(all(X(:, [1, 5, 6, 7]) >= 0, 'all'), "Negative masses and principle inertias Ixx Iyy Izz are not possible!")
    assert(sum(g.^2)==1, "Gravity vector is not a unit vector.")

    %% Function start
    
    time = tic;
    
    % Get candidate Y functions for gravity (as exponent matrix).
    Em_grav = PSDM.getSetYm(DH_ext, 'gravity');
    Nterms = size(Em_grav, 2);
    
    % Output information, if required.
    utilities.vprint(v, '\nRunning gravity derivation (%d search terms).\n', int32(Nterms));

    % Get mask of correlated terms.
    mask = PSDM.findYpMask(DH_ext, X, g, Em_grav, 'gravity', tol, v);

    % Reduce terms with mask
    Ep_grav = Em_grav(:, mask);
    
    % Get reduction matrix
    P_grav = PSDM.findReductionMatrix(DH_ext, X, g, Ep_grav, 'gravity', [], tol, v);

    % Output information, if required.
    utilities.vprint(v, '\tGravity matching done. %d terms remaining (took %.3g sec total).\n\n', int32(size(P_grav, 2)), toc(time));
    
end
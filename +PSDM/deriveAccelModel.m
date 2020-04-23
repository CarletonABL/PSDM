function [Ei, Pi] = deriveAccelModel(DH_ext, X, tol, v)
    % RUNIDACCEL Performs a dynamic ID using PSDM (Pseudo-Symbolic Dynamic
    % Modelling), but only for the acceleration terms.
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
    %   - tol: A tolerance used in the ID. Any factors below this tolerance
    %          (in estimation of the torques) is ignored. Defaut: 1e-12.
    %   - v: A verbosity flag. Set to false to suppress output. Default:
    %           true.
    %
    % OUTPUTS:
    %   - E: THe 4DOF x M PSDM exponent matrix associated with gravity.
    %   - P: The reduction matrix P
    %
    %   See also PSDM.runID(), PSDM.runIDGravity()

    %% Parse arguments
    
    if nargin < 3 || isempty(tol)
        tol = 1e-11;
    end
    
    if nargin < 4 || isempty(v)
        v = true;
    end

    % Check inputs
    assert(size(DH_ext, 2) == 6, "Invalid DH table");
    assert(all(abs(DH_ext(:, 6)) == 1), "Link sign column appears invalid. All numbers must be -1 or 1!");
    assert(size(X, 2) == 10 && size(X, 1) == size(DH_ext, 1), "X appears to be the wrong size!");
    assert(all(X(:, [1, 5, 6, 7]) >= 0, 'all'), "Negative masses and principle inertias Ixx Iyy Izz are not possible!")
    
    %% Function Start
    t = tic;
    DOF = size(DH_ext, 1);
    
    % Get general exponent matrix
    Em_acc = PSDM.getSetYm(DH_ext, 'joint');
    Nterms = size(Em_acc, 2);
    
    utilities.vprint(v, '\nRunning accel derivation (%d search terms).\n\n', int32(Nterms));
    
    % Define some helpers
    accelTerms = Em_acc((4*DOF+1):(5*DOF), :);
    
    % Preallocation some variables
    mask = ones(1, size(Em_acc, 2), 'logical');
    Ei = cell(DOF, 1);
    Pi = cell(DOF, 1);
    
    
    % Loop through each joint
    for j = 1:DOF
            
        % Get a mask of just terms relating to this joint
        jointMask = any(accelTerms(j, :) > 0, 1);
        
        % Combine with existing mast
        maskCombined = all([mask; jointMask], 1);
        
        % Get just the terms associated with this joint.
        Em_joint = Em_acc(:, maskCombined);
        
        % Output information, if required.
        utilities.vprint(v, '\tSearching in joint %d (%d terms)...\n', int32(j), int32(sum(maskCombined)));  
        
        % Get mask of correlated terms (Yp for this joint).
        maskCorr = PSDM.findYpMask(DH_ext, X, [], Em_joint, {'accel', j}, tol, v);
        
        % Combine with running mask
        mask(maskCombined) = all( vertcat( mask(maskCombined), maskCorr ) );    
        
        % Build up E matrix for each joint
        Ei{j} = Em_joint(:, maskCorr);
        Pi{j} = PSDM.findReductionMatrix(DH_ext, X, [], Em_joint(:, maskCorr), {'accel', j}, [], tol, v);

    end
    
    % Determine size of P
    p_acc = 0; % Keep track of number of terms in final result
    for i = 1:numel(Pi)
        p_acc = p_acc + size(Pi{i}, 2);
    end
    
    % Output information, if required.
    utilities.vprint(v, '\tAccel matching done. %d terms remaining (took %.3g sec total).\n\n', int32(p_acc), toc(t));
    
end

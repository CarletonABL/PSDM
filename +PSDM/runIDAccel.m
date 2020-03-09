function [E, P] = runIDAccel(DH_ext, X, tol, v)
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

    % Fill in imcomplete arguments
    if nargin < 3 || isempty(tol)
        tol = 1e-12;
    end
    
    if nargin < 4 || isempty(v)
        v = true;
    end

    % Check inputs
    assert(size(DH_ext, 2) == 6, "Invalid DH table");
    assert(all(DH_ext(:, 5) == 0), "This function does not currently work on prismatic joint robots");
    assert(all(abs(DH_ext(:, 6)) == 1), "Link sign column appears invalid. All numbers must be -1 or 1!");
    assert(size(X, 2) == 10 && size(X, 1) == size(DH_ext, 1), "X appears to be the wrong size!");
    assert(all(X(:, [1, 5, 6, 7]) >= 0, 'all'), "Negative masses and principle inertias Ixx Iyy Izz are not possible!")
    
    t = tic;
    DOF = size(DH_ext, 1);
    
    % Get general term map
    E1 = PSDM.genSearchTerms(DOF, 'accel');
    Nterms = size(E1, 2);
    
    utils.vprint(v, '\nRunning accel derivation (%d search terms).\n\n', int32(Nterms));
    
    % Define some helpers
    accelTerms = E1((3*DOF+1):(4*DOF), :);
    
    % Preallocation some variables
    mask = ones(1, size(E1, 2), 'logical');
    Ei = cell(DOF, 1);
    Pi = cell(DOF, 1);
    
    % Loop through each joint
    for j = 1:DOF
            
        % Get a mask of just terms relating to this joint
        jointMask = any(accelTerms(j, :) > 0, 1);
        
        % Combine with existing mast
        maskCombined = all([mask; jointMask], 1);
        
        Ejoint = E1(:, maskCombined);
        
        utils.vprint(v, '\tSearching in joint %d (%d terms)...\n', int32(j), int32(sum(maskCombined)));  
        
        maskCorr = PSDM.findCorrelationMask(DH_ext, X, [], Ejoint, {'accel', j}, tol, v);
        
        % Combine with running mask
        mask(maskCombined) = all( vertcat( mask(maskCombined), maskCorr ) );    
        
        % Build up E matrix for each joint
        Ei{j} = Ejoint(:, maskCorr);
        Pi{j} = PSDM.findBaseParams(DH_ext, X, [], Ejoint(:, maskCorr), {'accel', j}, [], tol, v);
        
    end
    
    % Solve for masks
    utils.vprint(v, "\tCombining terms:\n");
    [E, P] = PSDM.combineTerms(DH_ext, X, [], Ei, Pi, 'accel', tol, v);

    utils.vprint(v, '\tAccel matching done. %d terms remaining (took %.3g sec total).\n\n', int32(size(P, 2)), toc(t));
    
end

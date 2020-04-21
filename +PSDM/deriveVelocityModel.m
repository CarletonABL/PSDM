function [Ei, Pi] = deriveVelocityModel(DH_ext, X, Ep_accel, tol_in, v_in)
    % RUNIDVEL Performs a dynamic ID using PSDM (Pseudo-Symbolic Dynamic
    % Modelling), but only for the velocity terms.
    %
    % [Ep_vel, P_vel] = deriveVelocityModel(DH_ext, X, Ep_accel, tol_in, v_in)
    %
    % Should be run through PSDM.deriveModel command.
    %
    % Inputs:
    %   - DH_ext, X: As per runID.
    %   - E_accel: The accel map of the robot.
    %   - tol: Tolerance
    %   - v: Verbosity flag. Default: true.
    %
    % Ouputs:
    %   - Ep_vel: The exponent matrix representing the Y terms for
    %       velocity.
    %   - P_vel: The reduction matrix.
    
    %% Parse arguments
    
    % Fill in imcomplete arguments
    if nargin < 4 || isempty(tol_in)
        tol = 1e-12;
    else
        tol = tol_in;
    end
    
    if nargin < 5 || isempty(v_in)
        v = true;
    else
        v = v_in;
    end
    
    % Check inputs
    assert(size(DH_ext, 2) == 6, "Invalid DH table");
    assert(all(abs(DH_ext(:, 6)) == 1), "Link sign column appears invalid. All numbers must be -1 or 1!");
    assert(size(X, 2) == 10 && size(X, 1) == size(DH_ext, 1), "X appears to be the wrong size!");
    assert(all(X(:, [1, 5, 6, 7]) >= 0, 'all'), "Negative masses and principle inertias Ixx Iyy Izz are not possible!")
    
    
    %% Function Start
    
    % Init some variables
    DOF = size(DH_ext, 1);
    
    % Start timer
    time = tic;
    
    % Get candidate functions from accel terms
    Em_vel = PSDM.getSetYm(DH_ext, 'velocity', Ep_accel);
    
    Nterms = size(Em_vel, 2);
    Nterms_test = Nterms / (DOF + nchoosek(DOF, 2));
    
    % Initialize some terms
    velTerms = Em_vel((3*DOF+1):(4*DOF), :);
    
    % Preallocation some variables
    jointCombinations = nchoosek(1:DOF, 2); % Generate possible coriolis combinations
    mask = ones(1, Nterms, 'logical');
    Naccels = DOF + size(jointCombinations, 1);
    Ei = cell(Naccels, 1);
    Pi = cell(Naccels, 1);
    
    % Pre-allocate variables    
    if DOF < 4 || ~coder.target('matlab')
        for i = 1:Naccels
            Ei{i} = zeros(4*DOF, 1, 'uint8');
            Pi{i} = zeros(1,1);
        end
    end
    
    % Output information if required.
    utilities.vprint(v, '\nRunning centripital derivation (%d search terms).\n\n', int32(Nterms_test * DOF));
    
    p_vel = 0; % Keep track of number of functions
    
    % Loop through each centrifugal acceleration term
    for j = 1:DOF
            
        % Get a mask of just terms relating to this joint
        jointMask = all( vertcat( velTerms(1:DOF~=j, :) == 0, ...
                                  velTerms(j, :) == 2), 1 );
        
        % Combine with existing mask
        maskCombined = all([mask; jointMask], 1);
        
        % Get only terms associated with centrifugal accelerations of the
        % current joint
        Ep_cent_i = Em_vel(:,maskCombined);
        
        % Output information, if required.
        utilities.vprint(v, '\tSearching in joint %d (%d terms)...\n', int32(j), int32(sum(maskCombined)));  
        
        % Get correlation mask
        maskCorr = PSDM.findYpMask(DH_ext, X, [], Ep_cent_i, {'centripital', j}, tol, v);
        
        % Combine masks.
        mask(maskCombined) = all( vertcat( mask(maskCombined), maskCorr ) );   
        
        % Store results, and find reduction matrix
        Ei{j} = Ep_cent_i(:, maskCorr);
        Pi{j} = PSDM.findReductionMatrix(DH_ext, X, [], Ep_cent_i(:, maskCorr), {'centripital', j}, [], tol, v);
        p_vel = p_vel + size(Pi{j}, 2);
    end
    
    % Output information, if required
    utilities.vprint(v, '\nRunning coriolis derivation (%d search terms).\n\n', int32(Nterms_test * DOF));
    
    % Loop through and ID all coriolis terms
    for i = 1:size(jointCombinations, 1)
        
        % Joint angles of interest
        ij = jointCombinations(i, :);
            
        % Get a mask of just terms relating to this joint
        jointMask = all( vertcat( velTerms(ij(1), :) > 0, ...
                                  velTerms(ij(2), :) > 0 ), 1);
                              
        % Combine with existing mask
        maskCombined = all([mask; jointMask], 1);
        
        % Get exponent map of terms just associated with the current
        % coriolis combinations.
        Ep_cor_ij = Em_vel(:, maskCombined);

        % Output information, if required.
        utilities.vprint(v, '\tSearching in joints %d/%d (%d terms)...\n', int32(ij(1)), int32(ij(2)), int32(sum(maskCombined)));  
        
        % Get correlation mask
        maskCorr = PSDM.findYpMask(DH_ext, X, [], Ep_cor_ij, {'coriolis', ij}, tol, v);
        
        % Combine masks
        mask(maskCombined) = all( vertcat( mask(maskCombined), maskCorr ) );    
        
        % Store results, find reduction matrix
        Ei{DOF+i} = Ep_cor_ij(:, maskCorr);
        Pi{DOF+i} = PSDM.findReductionMatrix(DH_ext, X, [], Ep_cor_ij(:, maskCorr), {'coriolis', ij}, [], tol, v);
        p_vel = p_vel + size(Pi{DOF+i}, 2);
    end
    
    % Output information, if required.
    utilities.vprint(v, '\tVelocity matching done. %d terms remaining (took %.3g sec total).\n\n', int32(p_vel), toc(time));
   
end

function [E, Pi] = deriveVelocityModel(robot, Ep_accel, opt)
    % RUNIDVEL Performs a dynamic ID using PSDM (Pseudo-Symbolic Dynamic
    % Modelling), but only for the velocity terms.
    %
    % [E, Pi] = deriveVelocityModel(robot, Ep_accel, opt);
    %
    % This function is not intended for use by public, refer to
    % PSDM.deriveModel.
    %
    % See also PSDM.deriveModel
    
    % Init some variables
    DOF = robot.DOF;
    
    % Start timer
    time = tic;
    
    % Get candidate functions from accel terms
    Em_vel = PSDM.getSetYm(robot.lt, 'velocity', Ep_accel);
    
    Nterms = size(Em_vel, 2);
    Nterms_test = Nterms / (DOF + nchoosek(DOF, 2));
    
    % Initialize some terms
    velTerms = Em_vel((3*DOF+1):(4*DOF), :);
    
    % Preallocation some variables
    jointCombinations = nchoosek(1:DOF, 2); % Generate possible coriolis combinations
    mask = ones(1, Nterms, 'logical');
    Naccels = DOF + size(jointCombinations, 1);
    assert(Naccels < 1000);

    E = cell(Naccels, 1);
    if ~coder.target('matlab')
        for i = 1:Naccels
            E{i} = zeros(5*DOF, 1, 'uint8');
        end
    end
    Pi = cell(Naccels, 1);
    if ~coder.target('matlab')
        for i = 1:Naccels
            Pi{i} = zeros(1,1);
        end
    end
    
    % Output information if required.
    utilities.vprint(opt.v, '\nRunning centripital derivation (%d search terms).\n\n', ...
        int32(Nterms_test * DOF));
        
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
        utilities.vprint(opt.v, '\tSearching in joint %d (%d terms)...\n', ...
            int32(j), int32(sum(maskCombined)));  
        
        % Get correlation mask
        maskCorr = PSDM.findYpMask(robot, Ep_cent_i, {'centripital', j}, opt);
        
        % Combine masks.
        mask(maskCombined) = all( vertcat( mask(maskCombined), maskCorr ) );   
        
        % Store results, and find reduction matrix
        E{j} = Ep_cent_i(:, maskCorr);
        Pi{j} = PSDM.findReductionMatrix(robot, Ep_cent_i(:, maskCorr), {'centripital', j}, [], opt);
    end
    
    % Output information, if required
    utilities.vprint(opt.v, '\nRunning coriolis derivation (%d search terms).\n\n', ...
        int32(Nterms_test * DOF));
    
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
        utilities.vprint(opt.v, '\tSearching in joints %d/%d (%d terms)...\n', ...
            int32(ij(1)), int32(ij(2)), int32(sum(maskCombined)));  
        
        % Get correlation mask
        maskCorr = PSDM.findYpMask(robot, Ep_cor_ij, {'coriolis', ij}, opt);
        
        % Combine masks
        mask(maskCombined) = all( vertcat( mask(maskCombined), maskCorr ) );    
        
        % Store results, find reduction matrix
        E{DOF+i} = Ep_cor_ij(:, maskCorr);
        Pi{DOF+i} = PSDM.findReductionMatrix(robot, Ep_cor_ij(:, maskCorr), {'coriolis', ij}, [], opt);
    end
    
    % Get number of functions
    p_vel = 0; % Keep track of number of terms in final result
    for i = 1:numel(E)
        p_vel = p_vel + size(E{i}, 2);
    end
    % Get number of functions
    ell_vel = 0; % Keep track of number of terms in final result
    for i = 1:numel(Pi)
        ell_vel = ell_vel + size(Pi{i}, 2);
    end
    
    % Output information, if required.
    utilities.vprint(opt.v, '\tVelocity matching done. %d / %d terms remaining (took %.3g sec total).\n\n', ...
        int32(p_vel), int32(ell_vel), toc(time));
   
end

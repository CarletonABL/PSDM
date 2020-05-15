function [E, Pi] = deriveAccelModel(robot, opt)
    % RUNIDACCEL Performs a dynamic ID using PSDM (Pseudo-Symbolic Dynamic
    % Modelling), but only for the acceleration terms.
    %
    % [E, Pi] = PSDM.deriveAccelModel(robot, opt)
    %
    % This function is not intended for use by public, refer to
    % PSDM.deriveModel.
    %
    % See also PSDM.deriveModel

    %% Function Start
    t = tic;
    DOF = robot.DOF;
    
    % Get general exponent matrix
    Em_acc = PSDM.getSetYm(robot.lt, 'joint');
    Nterms = size(Em_acc, 2);
    
    utilities.vprint(opt.v, '\nRunning accel derivation (%d search terms).\n\n', int32(Nterms));
    
    % Define some helpers
    accelTerms = Em_acc((4*DOF+1):(5*DOF), :);
    
    % Preallocation some variables
    mask = ones(1, size(Em_acc, 2), 'logical');
    E = cell(DOF, 1);
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
        utilities.vprint(opt.v, '\tSearching in joint %d (%d terms)...\n', int32(j), int32(sum(maskCombined)));  
        
        % Get mask of correlated terms (Yp for this joint).
        maskCorr = PSDM.findYpMask(robot, Em_joint, {'accel', j}, opt);
        
        % Combine with running mask
        mask(maskCombined) = all( vertcat( mask(maskCombined), maskCorr ) );    
        
        % Build up E matrix for each joint
        E{j} = Em_joint(:, maskCorr);
        Pi{j} = PSDM.findReductionMatrix(robot, Em_joint(:, maskCorr), {'accel', j}, [], opt);

    end
    
    % Determine size of P
    p_acc = 0; % Keep track of number of terms in final result
    for i = 1:numel(Pi)
        p_acc = p_acc + size(Pi{i}, 2);
    end
    
    % Output information, if required.
    utilities.vprint(opt.v, '\tAccel matching done. %d terms remaining (took %.3g sec total).\n\n', int32(p_acc), toc(t));
    
end

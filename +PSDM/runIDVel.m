function [E, P] = runIDVel(DH_ext, X, E_accel, tol_in, v_in)
    % RUNIDVEL Performs a dynamic ID using PSDM (Pseudo-Symbolic Dynamic
    % Modelling), but only for the velocity terms.
    %
    % Should be run through PSDM.runID command. Inputs:
    %
    %   - DH_ext, X: As per runID.
    %   - E_accel: The accel map of the robot.
    %   - tol: Tolerance
    %   - v: Verbosity flag. Default: true.
    
    
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
    assert(all(DH_ext(:, 5) == 0), "This function does not currently work on prismatic joint robots");
    assert(all(abs(DH_ext(:, 6)) == 1), "Link sign column appears invalid. All numbers must be -1 or 1!");
    assert(size(X, 2) == 10 && size(X, 1) == size(DH_ext, 1), "X appears to be the wrong size!");
    assert(all(X(:, [1, 5, 6, 7]) >= 0, 'all'), "Negative masses and principle inertias Ixx Iyy Izz are not possible!")
    
    % Init some variables
    DOF = size(DH_ext, 1);
    
    % Start timer
    t = tic;
    
    % Get general term map
    E1 = PSDM.genSearchTerms(DOF, 'velocity', E_accel);
    
    Nterms = size(E1, 2);
    Nterms_test = Nterms / (DOF + nchoosek(DOF, 2));
    
    % Print, if required
    utils.vprint(v, '\nRunning centripital derivation (%d search terms).\n\n', int32(Nterms_test * DOF));
    
    % Initialize some terms
    velTerms = E1((2*DOF+1):(3*DOF), :);
    
    % Preallocation some variables
    jointCombinations = nchoosek(1:DOF, 2); % Generate possible coriolis combinations
    mask = ones(1, Nterms, 'logical');
    Naccels = DOF + size(jointCombinations, 1);
    Ei = cell(Naccels, 1);
    Pi = cell(Naccels, 1);
    
    if DOF < 4 || ~coder.target('matlab')
        for i = 1:Naccels
            Ei{i} = zeros(4*DOF, 1, 'uint8');
            Pi{i} = zeros(1,1);
        end
    end
    
    for j = 1:DOF
            
        % Get a mask of just terms relating to this joint
        jointMask = all( vertcat( velTerms(1:DOF~=j, :) == 0, ...
                                  velTerms(j, :) == 2), 1 );
        
        % Combine with existing mast
        maskCombined = all([mask; jointMask], 1);
        
        Ejoint = E1(:,maskCombined);
        
        utils.vprint(v, '\tSearching in joint %d (%d terms)...\n', int32(j), int32(sum(maskCombined)));  
        
        maskCorr = PSDM.findCorrelationMask(DH_ext, X, [], Ejoint, {'centripital', j}, tol, v);
        
        mask(maskCombined) = all( vertcat( mask(maskCombined), maskCorr ) );   
        
        Ei{j} = Ejoint(:, maskCorr);
        Pi{j} = PSDM.findBaseParams(DH_ext, X, [], Ejoint(:, maskCorr), {'centripital', j}, [], tol, v);
        
    end
    
    utils.vprint(v, '\nRunning coriolis derivation (%d search terms).\n\n', int32(Nterms_test * DOF));
    
    % Loop through and ID
    for i = 1:size(jointCombinations, 1)
        
        % Joint angles of interest
        j = jointCombinations(i, :);
            
        % Get a mask of just terms relating to this joint
        jointMask = all( vertcat( velTerms(j(1), :) > 0, ...
                                  velTerms(j(2), :) > 0 ), 1);
                              
        % Combine with existing mask
        maskCombined = all([mask; jointMask], 1);
        
        Ejoint = E1(:, maskCombined);

        utils.vprint(v, '\tSearching in joints %d/%d (%d terms)...\n', int32(j(1)), int32(j(2)), int32(sum(maskCombined)));  
        
        maskCorr = PSDM.findCorrelationMask(DH_ext, X, [], Ejoint, {'coriolis', j}, tol, v);
        
        mask(maskCombined) = all( vertcat( mask(maskCombined), maskCorr ) );    
        
        Ei{DOF+i} = Ejoint(:, maskCorr);
        Pi{DOF+i} = PSDM.findBaseParams(DH_ext, X, [], Ejoint(:, maskCorr), {'coriolis', j}, [], tol, v);
        
    end
    
    % Solve for masks
    utils.vprint(v, "\tCombining terms:\n");
    [E, P] = PSDM.combineTerms(DH_ext, X, [], Ei, Pi, 'velocity', tol, v);

    utils.vprint(v, '\tVelocity matching done. %d terms remaining (took %.3g sec total).\n\n', int32(sum(mask)), toc(t));
   
end

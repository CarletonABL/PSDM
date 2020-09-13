function [E, Pi] = deriveGravityModel(robot, opt)
    % RUNIDGRAVITY Performs a dynamic ID using PSDM (Pseudo-Symbolic Dynamic
    % Modelling), but only for the gravity terms.
    %
    % [E_grav, P_grav] = PSDM.deriveGravityModel(robot, opt)
    %
    % This function is not intended for use by public, refer to
    % PSDM.deriveModel.
    %
    % See also PSDM.deriveModel
    
    % Start timer
    coder.extrinsic('tic', 'toc')
    time = tic;
    
    % Get candidate Y functions for gravity (as exponent matrix).
    Em_grav = PSDM.getSetYm(robot.lt, 'gravity');
    Nterms = size(Em_grav, 2);
    
    % Output information, if required.
    utilities.vprint(opt.v, '\nRunning gravity derivation (%d search terms).\n', int32(Nterms));

    % Get mask of correlated terms.
    mask = PSDM.findYpMask(robot, Em_grav, 'gravity', opt);

    % Reduce terms with mask
    E = Em_grav(:, mask);
    
    % Get reduction matrix
    Pi = PSDM.findReductionMatrix(robot, E, 'gravity', [], opt);
    
    % Simplify P matrices by normalizing each column by the value of the
    % first nonzero value in P.
    Pi = PSDM.simplifyReductionMatrix(Pi, opt.tol);

    % Output information, if required.
    utilities.vprint(opt.v, '\tGravity matching done. %d terms remaining (took %.3g sec total).\n\n', int32(size(Pi, 2)), toc(time));
    
end
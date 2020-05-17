function [vars, names, code] = makePCode(vars, names, code, opt)
    % MAKEPTHETACODE Generates the code for defining PTheta in a function

    tol = 1e-11;
    
    % Get vars
    DOF = vars.DOF;
    P = vars.P;
    E = vars.E;
    p = size(E, 2);

    % Setup P. Multiply with gravity
    gravMask = ~any( E( (1:(2*DOF)) + 3*DOF, :) > 0, 1);
    P(gravMask, :, :) = P(gravMask, :, :) .* utilities.g;
    
    % Round out small terms
    P( abs(P) < tol ) = 0;
    P( abs(P - 1) < tol) = 1;
    
    % Do a rational rounding
    P = utilities.ratRound(P, tol);
    
    % Define mask
    P_mask = abs(P) > tol;
    vars.P = P;
    vars.P_mask = P_mask;

    %% Extra processing for FD
    if strcmp(opt.alg, 'FD')        
        % Forward dynamics
        
        % Need to calculate P for induced torques and each individual
        % torque
        induced_mask = ~any( E( (1:DOF) + 4*DOF, :) > 0, 1 );
        P_induced = P( induced_mask, :, :);
        P_induced_mask = abs(P_induced) > 0;
        E_induced = E( :, induced_mask);
        
        % Get accel specific variables
        accel_masks = zeros( DOF, p, 'logical');
        P_accel = cell(DOF, 1); P_accel_mask = cell(DOF, 1); E_accel = cell(DOF, 1);
        for i = 1:DOF
            accel_masks( i, : ) = E( 4*DOF + i, :) > 0;
            P_accel{i} = P( accel_masks( i, : ), :, :);
            P_accel_mask{i} = abs(P_accel{i}) > 0;
            E_accel{i} = E( 1:(3*DOF), accel_masks( i, : ));
        end
        
        % Store in vars variable
        vars.induced_mask = induced_mask;
        vars.P_induced = P_induced;
        vars.P_induced_mask = P_induced_mask;
        vars.E_induced = E_induced;
        
        vars.accel_masks = accel_masks;
        vars.P_accel = P_accel;
        vars.P_accel_mask = P_accel;
        vars.E_accel = E_accel;
        
    end
        
end
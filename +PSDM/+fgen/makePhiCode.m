function [vars, names, code] = makePhiCode(vars, names, code, PhiName)
    % MAKEPTHETACODE Generates the code for defining PTheta in a function

    tol = 1e-10;
    
    % Get dof
    DOF = vars.DOF;
    P = vars.P;
    Theta = vars.Theta;

    % Calculate Phi_b
    Phi = zeros( size(P, 1), DOF );
    for i = 1:DOF
        Phi(:, i) = P(:, :, i) * Theta;
    end
    
    % Pre-multiply in gravity terms
    gravMask = ~any( vars.E( (1:(2*DOF)) + 3*DOF, :) > 0, 1);
    Phi(gravMask, :) = Phi(gravMask, :) .* utilities.g;
    
    % Round away small numbers
    Phi( abs(Phi) < tol ) = 0;
    Phi( abs(Phi - 1) < tol) = 1;
    
    % Convert to string
    Phi_str = mat2str(Phi);
    
    % Convert to code
    code.Phi = sprintf('%s = coder.const(%s);', PhiName, Phi_str);
    names.Phi = PhiName;
    vars.Phi = Phi;
    
end
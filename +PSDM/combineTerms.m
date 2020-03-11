function [E, P] = combineTerms(DH_ext, X, g, Ei, Pi, idType, tol, v)
    % COMBINETERMS Combines multiple exponent maps Ei (cell array) and
    % reduction matrices (Pi) into a single, Ei and Pi.
    %
    % idType specifies what type of combinations to test for, possible
    % options are 'accel' or 'velocity'.

    if nargin < 6 || isempty(idType)
        idType = 'all';
    end
    if nargin < 7 || isempty(tol)
        tol = 1e-14;
    end
    if nargin < 8 || isempty(v)
        v = true;
    end

    % Define some variables
    N = numel(Pi);
    DOF = size(DH_ext, 1);
    
    % Get combined sizes
    n = 0; m = 0; 
    for i = 1:N
        [ni, mi, ~] = size(Pi{i});
        n = n + ni;
        m = m + mi;
    end
    
    % Initialize a "combined" P and E matrix and populate it in a loop
    P_combined = zeros(n, m, DOF);
    E = zeros(5*DOF, n, 'uint8');
    n = 0; m = 0;
    for i = 1:N
        [ni,mi,~] = size(Pi{i});
        P_combined( (n+1):(n+ni), (m+1):(m+mi), : ) = Pi{i};
        E( :, (n+1):(n+ni) ) = Ei{i};
        n = n + ni;
        m = m + mi;
    end
    
    % Find base parameters of "combined" system
    P = PSDM.findBaseParams(DH_ext, X, g, E, idType, P_combined, tol, v);
    
end
function [baseTerms, inertiaTerms] = splitRegressorTerms(tau, vars)

    % Gets a symbolic vector of all the unique terms in the dynamics
    % equation
    DOF = size(tau, 1);
    
    Q = vars.Q;
    Qd = vars.Qd;
    Qdd = vars.Qdd;
    
    % Get list of all coefficient terms
    tau = expand(tau);
    
    varsTau = symvar(tau);
    assumeAlso(varsTau, 'real');
    
    % Replace sin^2 with (1-cos^2)
    tau = subs(tau, sym2cell(sin(Q).^2), sym2cell(1-cos(Q).^2));
    tau = expand(tau);
        
    for i = 1:DOF
        
        fprintf("Splitting terms (%d / %d)...\n", i, DOF);

        termsDOF = children(tau(i))';

        varsNonInertial = symvar( vertcat(Q, Qd, Qdd) );

        [baseTerms{i}, inertiaTerms{i}] = splitTermsDOF(termsDOF, varsNonInertial);
        
    end
    
end


function [baseTermsDOF, inertiaTermsDOF] = splitTermsDOF(terms, nonConstantTerms)
    % Returns the list of terms provided with all constants removed.

    N = numel(terms);
    
    baseTermsDOF = zeros(size(terms), 'sym');
    inertiaTermsDOF = zeros(size(terms), 'sym');
        
    utils.parfor_progress(  round(N/50)  );
    
    % Loop through all terms
    parfor i = 1:N
        
        if mod(i, 50) == 0
            utils.parfor_progress;
        end
        
        term = terms(i);
        
        if isequal(term, sym(0))
            continue;
        end
        
        % Get term children
        c = sort(children(term));
        M = numel(c);
        isInertiaEl = zeros(size(c), 'logical');
        
        % Loop through each one to determine if its constant
        for j = 1:M
            varsEl = symvar(c(j));
            isInertiaEl(j) = isempty(varsEl) || ~all(ismember(symvar(c(j)), nonConstantTerms));
        end
        
        % Reproduce the "clean" terms
        baseTermsDOF(i) = prod(c(~isInertiaEl));
        inertiaTermsDOF(i) = prod(c(isInertiaEl));
        
    end
    
    baseTermsDOF = baseTermsDOF(baseTermsDOF~=0);
    inertiaTermsDOF = inertiaTermsDOF(inertiaTermsDOF~=0);
        
    utils.parfor_progress(0);

end
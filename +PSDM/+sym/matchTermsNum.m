function terms1Ind = matchTermsNum(terms1, terms2)

    assert(size(terms1, 2) == 1 && size(terms2, 2) == 1, "This function only works on column vectors!");

    if nargin < 3
        Ntests = 10;
    end
    
    if nargin < 4
        tol = 1e-6;
    end

    vars = symvar(vertcat(terms1, terms2));

    varsExp1Tmp = symvar(terms1);
    varsExp2Tmp = symvar(terms2);
    
    varsExp1Ind = ismember(vars, varsExp1Tmp);
    varsExp2Ind = ismember(vars, varsExp2Tmp);
    
    h1 = matlabFunction(terms1, 'vars', {vars(varsExp1Ind)});
    h2 = matlabFunction(terms2, 'vars', {vars(varsExp2Ind)});
    
    V1 = repmat(zeros(size(terms1)), [1 Ntests]);
    V2 = repmat(zeros(size(terms2)), [1 Ntests]);
    
    p = gcp('nocreate');
    if ~isempty(p)
        parfor i = 1:Ntests
            varsNum = rand(size(vars))*2-1;
            V1(:, i) = h1(varsNum(varsExp1Ind));
            V2(:, i) = h2(varsNum(varsExp2Ind));
        end
    else
        for i = 1:Ntests
            varsNum = rand(size(vars))*2-1;
            V1(:, i) = h1(varsNum(varsExp1Ind));
            V2(:, i) = h2(varsNum(varsExp2Ind));
        end
    end
    
    N1 = size(terms1, 1);
    N2 = size(terms2, 1);
    terms1Ind = zeros(size(terms1));
    
    for i = 1:N1
        E = abs(V1(i, :) - V2);
        match = find( all(E < tol, 2) , 1);
        if ~isempty(match)
            terms1Ind(i) = match;
        end
    end
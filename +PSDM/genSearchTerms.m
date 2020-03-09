function map = genSearchTerms(DOF, type, terms_acc)
    % Generates the elements to build a generic function map for a robot
    % manipulator.
    %
    %   INPUTS:
    %       - DOF: The number of degrees of freedom of the robot.
    %       - type: Can be 'accel', 'gravity', 'velocity' or 'all'. Will
    %           return all the search terms for that type of ID.
    %           Default: 'all'.
    %       - terms_acc: If type=='velocity', you must provide a reduced
    %           set of acceleration terms to base the calculation off of.
    %           Ignored for other cases.
    %
    %   -map: is a MxN logical matrix which indicates how to form the N'th
    %       term of the summation. 
    %
    %       The order of map is:
    %           sin(q)
    %           cos(q)
    %           qd
    %           qdd
    
    if nargin < 2 || isempty(type)
        type = 'all';
    end
    
    % Get ratio combinations
    if strcmp(type, 'accel')

        % Every possible combination of cos(qi), sin(qi)
        r_comb_all = uint8(de2bi(0:(2^(3*DOF)))');
        r_comb_all = r_comb_all(1:(end-1), :);

        % Separate them
        r_comb = r_comb_all(1:(2*DOF), :);
        r_comb_cos2 = r_comb_all((2*DOF+1):end, :);

        % Add in cos2 cases
        r_comb((1+DOF):end, 1:end) = r_comb((1+DOF):end, 1:end) + r_comb_cos2;  

        % Remove dulicate rows
        r_comb = unique(r_comb', 'rows')';

    elseif strcmp(type, 'velocity')

        % Need to use the accel terms to derive vel terms
        assert(nargin == 3 && ~isempty(terms_acc), "Must specify acceleration terms!");
        
        % Extract geometric terms, make unique
        r_comb_acc = terms_acc(1:(2*DOF), :);
        r_comb_acc = unique(r_comb_acc', 'rows')';

        N = size(r_comb_acc, 2);
        
        % Take derivative of each ratio term
        % Preallocate and loop
        r_comb = zeros(size(r_comb_acc, 1), N * 2 * DOF, 'uint8');
        c = 0;
        for i = 1:N
            for j = 1:DOF
                [t, n] = diffTerm(r_comb_acc(:, i), j);
                c = c + n;
                r_comb(:, (c-n+1):(c)) = t;
            end
        end
        
        % Trim and make unique
        r_comb = uint8(r_comb(:, 1:c));
        r_comb = unique(r_comb', 'rows')';

    elseif strcmp(type, 'gravity')

        % Just linear combinations of rows
        r_comb = uint8(de2bi(0:(2^(2*DOF)))');
        r_comb = r_comb(1:(end-1), 1:(end-1));
        N = size(r_comb, 2);
        
        % Remove any term which isn't monolinear
        sterms = r_comb(1:DOF, :);
        cterms = r_comb((1+DOF):(2*DOF), :);
        mask = ~any( all( cat(3, sterms, cterms), 3 ), 1 );
        
        r_comb = r_comb(:, mask);

    else
        
        error("Unknown type!");
        
    end

    Nr_comb = size(r_comb, 2);
    
    % Eliminate some of the combinations that aren't possible.
    r_comb_elim = zeros(1, Nr_comb, 'logical');
    for i = 1:Nr_comb
        sinTab = r_comb(1:DOF, i);
        cosTab = r_comb((1+DOF):end, i);
        
        r_comb_elim(i) = any( all( [cosTab == 2, sinTab == 1], 2 ), 1);
    end
    
    r_comb = r_comb(:, ~r_comb_elim);

    if strcmp(type, 'gravity')
        
        nExtraCols = 1;
        t_combs = zeros(DOF*2, 1);
                              
    elseif strcmp(type, 'velocity')
        
        % v_comb combinations Every possible combination of 2 choice of 1:DOF.
        v_comb_ind = nchoosek(1:DOF, 2);
        Nvcombs = size(v_comb_ind, 1);
        v_comb = zeros(DOF, Nvcombs, 'uint8');
        indTrue = sub2ind([DOF, Nvcombs], v_comb_ind, repmat((1:Nvcombs)', [1 2]));
        v_comb(indTrue(:)) = true;

        % v2 comb
        v_comb = horzcat(v_comb,  2*eye(DOF, 'uint8'));
        Nvcols = size(v_comb, 2);
        
        nExtraCols = Nvcols;
        t_combs = vertcat(v_comb, zeros(size(v_comb)));
                              
    elseif strcmp(type, 'accel')
        
        % a comb
        a_comb = eye(DOF, 'uint8');
        Nacols = size(a_comb, 2);
        Narows = size(a_comb, 1);
        
        nExtraCols = Nacols;
        t_combs = vertcat( zeros(size(a_comb)), a_comb);
        
    end
    
    % Concat everything together
    map_top = repmat(r_comb, [1, nExtraCols]);
    map_bottom = repelem(t_combs, 1, size(r_comb, 2));
    map = vertcat(map_top, map_bottom);
    
end



function [t, n] = diffTerm(term, i)
    % For a term in the termsMap, takes the derivative with respect to i
        
    DOF = size(term, 1)/2;
    
    % Extract sin and cosin terms
    s = term(1:DOF);
    c = term((1+DOF):(2*DOF));

    % Get logical terms of whether the term corresponding to joint i is
    % sin, cosine or cosine squre (or combination
    si = term(i) == 1;
    ci = term(i+DOF) == 1;
    c2i = term(i+DOF) == 2;
    
    % Initialize the output
    sd = repmat(s, [1 2]);
    cd = repmat(c, [1 2]);
    n = 1;
    
    % Go through all the cases
    if si && ~ci && ~c2i
        
        sd(i, 1) = 0;
        cd(i, 1) = 1;
        
    elseif ~si && ci && ~c2i
        
        sd(i, 1) = 1;
        cd(i, 1) = 0;
        
    elseif si && ci && ~c2i
        
        sd(i, :) = [0 0];
        cd(i, :) = [2 0];
        n = 2;
        
    elseif ~si && ~ci && c2i
        
        sd(i, 1) = 1;
        cd(i, 1) = 1;
        
    elseif ~si && ~ci && ~c2i
        % nothing changes, its constant

    end
    
    t = uint8(vertcat(sd(:, 1:n), cd(:, 1:n)));
    
end
function Upd = getSetUpsilon_diff(Up_in, DH_ext)
    % GETSETUPSILON_DIFF Returns the minimal set of functions which are all
    % possible derivatives of a set of functions Up. Used when doing the
    % velocity model id.
    %
    % Upd = getSetUpsilon_diff(Up_in, DH_ext)
    
    %% Parse arguments
    
    DOF = size(DH_ext, 1);
    
    % If Up was given as a Y instead, need to extract Upsilon
    if size(Up_in, 1)/DOF > 3
        Up_all = Up_in(1:(3*DOF), :);
        Up = unique(Up_all', 'rows')';
    else
        Up = Up_in;
    end
    
    M = size(Up, 2);
    
    %% Start Function
    
    % Preallocate Upd to maximum possible size
    Upd_all = zeros(size(Up, 1), M*2*DOF, 'uint8');
    
    % Allocate in terms in a loop
    c = 0;
    for i = 1:M
        for j = 1:DOF
            % For each term, and each joint, take derivative and populate
            % array
            [t, n] = diffUp(Up(:, i), j, logical(DH_ext(j, 5)));
            c = c + n;
            Upd_all(:, (c-n+1):(c)) = t;
            
        end
    end
    
    % Trim and condense terms
    Upd_trim1 = Upd_all(:, 1:c);
    Upd = unique(Upd_trim1', 'rows')';
    
end
    
function [t, n] = diffUp(up, i, lt)
    % DIFFUP for a term up in Upsilon, differentiates it with respects to
    % the ith joint.
    %
    % INPUTS:
    %   -up: The single upsilon term.
    %   -i: The joint variable which is being derived by.
    %   -lt: The link type (false for revolute, true for prismatic).
    %
    % OUTPUTS:
    %   -t: the derivative terms, of the same column dimension of up, and
    %       either 1 or 2 columns wide.
    %   -n: The number of derivative terms (equal to the column width of
    %       t).
    
    % Figure out the DOF of the robot
    DOF = size(up, 1)/3;
    
    % Extract sin and cosin terms
    l = up(1:DOF);
    s = up((1:DOF)+DOF);
    c = up((1:DOF)+2*DOF);
    
    % Initialize the output
    ld = repmat(l, [1 2]);
    sd = repmat(s, [1 2]);
    cd = repmat(c, [1 2]);
    n = 1;
    
    % Go through all the cases
    if lt
        % Prismatic joint
        ld(i, 1) = max(l(i, 1) - 1, uint8(0));
        n = 1;
        
    else
        % Revolute
        
        % Get logical terms of whether the term corresponding to joint i is
        % sin, cosine or cosine squre (or combination
        si = up(i+DOF) == 1;
        ci = up(i+2*DOF) == 1;
        c2i = up(i+2*DOF) == 2;
    
        if si && ~ci && ~c2i

            sd(i, 1) = uint8(0);
            cd(i, 1) = uint8(1);

        elseif ~si && ci && ~c2i

            sd(i, 1) = uint8(1);
            cd(i, 1) = uint8(0);

        elseif si && ci && ~c2i

            sd(i, :) = uint8([0 0]);
            cd(i, :) = uint8([2 0]);
            n = 2;

        elseif ~si && ~ci && c2i

            sd(i, 1) = uint8(1);
            cd(i, 1) = uint8(1);

        elseif ~si && ~ci && ~c2i
            % nothing changes, its constant

        end
        
    end
    
    % Concat all options together   
    t = uint8(vertcat(ld(:, 1:n), sd(:, 1:n), cd(:, 1:n)));
    
end
function [E, P] = combineModels(robot, Ei_in, Pi_in, idType, opt)
    % COMBINEMODELS Combines multiple dynamic model components (specified
    % by a cell array of Ei and Pi exponent and reduction matrices,
    % respectively.
    %
    % [E, P] = combineModels(robot, Ei_in, Pi_in, idType, opt)
    %
    % idType specifies what type of combinations to test for, possible
    % options are 'accel' or 'velocity'.
    
    %% Parse Inputs    
    DOF = robot.DOF;
    N = 1 + DOF*2 + nchoosek(DOF, 2);
    assert(N< 1000);
    
    % Parse inputs if a cell array of cells is given
    if iscell(Pi_in) && iscell(Pi_in{1})
        % We've been given a cell array of cell arrays
        if coder.target('matlab')
            Ei = vertcat( Ei_in{:} );
            Pi = vertcat( Pi_in{:} );
        else
            % So matlab is bad at handling cell arrays for codegen.
            % Additionally, I can't move this into a subfunction because
            % then matlab doesn't recognize the code patterns. So it stays.
            
            % Initialize the cell array and clearly show that each element
            % is assigned a value (in a loop)
            Ei = cell(N, 1);
            for i = 1:N
                Ei{i} = uint8(0);
            end
            Pi = cell(N, 1);
            for i = 1:N
                Pi{i} = 0;
            end
            
            % Now assign the actual value into the cell array.
            c = 1;
            for i = 1:numel(Ei_in)
                for j = 1:numel(Ei_in{i})
                    Ei{c} = Ei_in{i}{j};
                    c = c+1;
                end
            end
            c = 1;
            for i = 1:numel(Pi_in)
                for j = 1:numel(Pi_in{i})
                    Pi{c} = Pi_in{i}{j};
                    c = c+1;
                end
            end
        end
    else
        Ei = Ei_in;
        Pi = Pi_in;
    end

    % Check P
    for i = 1:N
        if nargin < 3 || numel(Pi) < i || isempty(Pi{i})
            Pi{i} = repmat( eye( size(Ei{i}, 2) ), [1 1 DOF]);
        end
    end
    
    if nargin < 4 || isempty(idType)
        idType = 'all';
    end
    
    %% Function start
        
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
        [ni, mi, ~] = size(Pi{i});
        P_combined( (n+1):(n+ni), (m+1):(m+mi), : ) = Pi{i};
        E( :, (n+1):(n+ni) ) = Ei{i};
        n = n + ni;
        m = m + mi;
    end
    
    % Find base parameters of "combined" system
    P = PSDM.findReductionMatrix(robot, E, idType, P_combined, opt);
    
end
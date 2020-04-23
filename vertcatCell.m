function C = vertcatCell(varargin)

    N = nargin;
    
    % Get size
    c = 0;
    for i = 1:N
        c = c + size(varargin{i}, 1);
    end
    
    C = cell(c, 1);
    
    % Assign in loop
    c = 1;
    for i = 1:N
        for j = 1:size(varargin{i}, 1)
            C{c} = varargin{i}{j};
            c = c+1;
        end
    end

end

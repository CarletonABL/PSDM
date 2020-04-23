function C = vertcatCell(varargin)

    N = nargin;
    
    % Get size
    c = 0;
    for i = 1:N
        c = c + size(varargin{i}, 1);
    end
    
    C = initCell(1, c, class(varargin{1}{1}));
    
    %assert(c < 1000);
    %C = cell(1, c);
    %for i = 1:c
    %    C{i} = cast(0, class(varargin{1}{1}));
    %end
    
    % Assign in loop
    c = 1;
    for i = 1:N
        for j = 1:size(varargin{i}, 1)
            C{c} = varargin{i}{j};
            c = c+1;
        end
    end

end

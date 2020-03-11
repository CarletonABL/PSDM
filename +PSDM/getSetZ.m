function Z = getSetZ(order, jointType)
    % GETZ Gets a list of all functions Z of order k, for a joint type.
    %
    % Inputs:
    %   order: The order of the set requested
    %   jointType => true for prismatic, false for revolute.
    % 
    % Output:
    %   e: 3 x M integer matrix of the exponents of q, sin(q) and cos(q),
    %   respectively.
    
    if ~jointType
        % Revolute
        
        switch order
            case uint8(1)
                Z = uint8([0 0 0;
                           0 1 0;
                           0 0 1]);
            case uint8(2)
                Z = uint8([0 0 0 0 0;
                           0 1 0 1 0;
                           0 0 1 1 2]);
            otherwise
                error("This function hasn't implemented orders greater than 2 currently");
        end
        
    else
        % Prismatic
        
        switch order
            case uint8(1)
                Z = uint8([0 1;
                           0 0;
                           0 0]);
            case uint8(2)
                Z = uint8([0 1 2;
                           0 0 0;
                           0 0 0]);
            otherwise
                error("This function hasn't implemented orders greater than 2 currently");
        end
        
    end
    
end
function vprint(verbose, varargin)
    % VPRINT functions exactly like fprintf except the first argument is a
    % verbosity flag. If true, it will run as if it were fprintf.
    % Otherwise, nothing happens.
    %
    % Useful for cleaning up code.
    
    if verbose
        fprintf(varargin{:});
    end
end
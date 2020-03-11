function doPar = autoPar
    % Returns a flag, doPar, which is true if the matlab coder is a c
    % compiler, OR if the parallel pool is already enabled.

    % Run loop in parallel if possible.
    % Use parallel if in matlab and parallel pool open, or if using matlab
    % coder
    if coder.target('matlab')
        doPar = ~isempty(gcp('nocreate'));
    else
        doPar = true;
    end
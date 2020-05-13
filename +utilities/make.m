function make(functions)
    % UTILS.MAKE Compiles some of the expensive functions into mex-files
    % for faster computation.
    
    if nargin < 1 || isempty(functions)
        functions = 'all';
    end
    
    path = fullfile( fileparts(mfilename('fullpath')), '..');
    
    %% Compile blockprod function
    if any(strcmp(functions, 'blockprod')) || any(strcmp(functions, 'all'))
    
        fprintf("Compiling utilities.blockprod into mex file... ");
        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ReportPotentialDifferences = false;

        % Define argument types for entry-point 'blockprod_private'.
        ARGS = cell(1,1);
        ARGS{1} = cell(2,1);
        ARGS{1}{1} = coder.typeof(0,[Inf Inf Inf],[1 1 1]);
        ARGS{1}{2} = coder.typeof(0,[Inf Inf Inf],[1 1 1]);

        % Invoke MATLAB Coder.
        cd(fullfile(path, '+utilities'));

        codegen -config cfg -I path +utilities/blockprod -args ARGS{1}
        fprintf("Done!\n");
        
    end
    
    %% Compile residual3p function
    if any(strcmp(functions, 'residual3p')) || any(strcmp(functions, 'all'))
        fprintf("Compiling utilities.residual3p into mex file... ");
        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ReportPotentialDifferences = false;

        % Define argument types for entry-point 'blockprod_private'.
        ARGS = cell(1,1);
        ARGS{1} = cell(3,1);
        ARGS{1}{1} = coder.typeof(0,[Inf Inf],[1 1]); % A
        ARGS{1}{2} = coder.typeof(0,[Inf Inf],[1 1]); % x
        ARGS{1}{3} = coder.typeof(0,[Inf Inf],[1 1]); % b

        % Invoke MATLAB Coder.
        cd(fullfile(path, '+utilities'));

        codegen -config cfg -I path +utilities/residual3p -args ARGS{1}
        fprintf("Done!\n");
    end
            
    %% Cleanup
    
    fprintf("Done code generation for utilities package.\n");
    
    cd(path);
 
end
function make(functions)
    % MAKE Compiles the functions in PSDM to make them run faster.
    
    filename = mfilename('fullpath');
    [pathPSDM, ~, ~] = fileparts(filename);
    path = fullfile(pathPSDM, '..');
    
    if nargin < 1 || isempty(functions)
        functions = 'all';
    end
    
    %% RUNID
    
    if any(strcmp(functions, 'deriveModel')) || any(strcmp(functions, 'all'))
        
        fprintf("Compiling PSDM.deriveModel into mex file... ");

        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ExtrinsicCalls = false;
        cfg.ReportPotentialDifferences = false;

        % Define argument types for entry-point 'runID'.
        ARGS = cell(1,1);
        ARGS{1} = cell(6,1);
        ARGS{1}{1} = coder.typeof(0,[10  6],[1 0]); % DH
        ARGS{1}{2} = coder.typeof(0,[3 1]);         % g
        ARGS{1}{3} = coder.typeof(0,[10 10],[1 1]); % X
        ARGS{1}{4} = coder.typeof(0, [1 1], [1 1]); % tolerance
        ARGS{1}{5} = coder.typeof(false, [1 1], [1 1]);           % verbose
        ARGS{1}{6} = coder.typeof(false, [1 1], [1 1]);           % gravity_only
        

        cd(fullfile(path, '+PSDM'));

        codegen -config cfg -I path +PSDM/deriveModel -args ARGS{1}

        cd(path);
        fprintf("Done!\n");
    end
    
    %% generateYp
    
    if any(strcmp(functions, 'generateYp')) || any(strcmp(functions, 'all'))
        fprintf("Compiling PSDM.generateYp into mex file... ");

        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ExtrinsicCalls = false;
        cfg.ReportPotentialDifferences = false;

        % Define argument types for entry-point 'genTermValues'.
        ARGS = cell(1,1);
        ARGS{1} = cell(4,1);
        ARGS{1}{1} = coder.typeof(0,[10 Inf],[1 1]);
        ARGS{1}{2} = coder.typeof(0,[10 Inf],[1 1]);
        ARGS{1}{3} = coder.typeof(0,[10 Inf],[1 1]);
        ARGS{1}{4} = coder.typeof(uint8(0),[40 Inf],[1 1]);

        cd(fullfile(path, '+PSDM'));

        codegen -config cfg -I path +PSDM/generateYp -args ARGS{1}

        cd(path);
        fprintf("Done!\n");
    end
    
    %% InverseDynamicsNewton
    
    if any(strcmp(functions, 'inverseDynamicsNewton')) || any(strcmp(functions, 'all'))

        disp("Compiling PSDM.inverseDynamicsNewton function into mex...");

        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ReportPotentialDifferences = false;

        % Define argument types for entry-point 'inverseDynamicsNewton'.
        ARGS = cell(1,1);
        ARGS{1} = cell(8,1);
        ARGS{1}{1} = coder.typeof(0,[10  6],[1 1]);
        ARGS{1}{2} = coder.typeof(0,[10 10],[1 0]);
        ARGS{1}{3} = coder.typeof(0,[10 Inf],[1 1]);
        ARGS{1}{4} = coder.typeof(0,[10 Inf],[1 1]);
        ARGS{1}{5} = coder.typeof(0,[10 Inf],[1 1]);
        ARGS{1}{6} = coder.typeof(int8(0));
        ARGS{1}{7} = coder.typeof(0,[3 1]);
        ARGS{1}{8} = coder.typeof(0,[3 3]);

        % Invoke MATLAB Coder.
        cd( fullfile( path, '+PSDM') );
        codegen -config cfg -I roboDir +PSDM/inverseDynamicsNewton -args ARGS{1}
        cd( path );

        disp("Done.");
        
    end
    
    disp("Code generation done for PSDM package.");
    
    if any([strcmp(functions, 'all'), strcmp(functions, 'residual3p'), strcmp(functions, 'blockprod')])
        utilities.make(functions);
    end


end
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
        ARGS{1} = cell(5,1);
        ARGS{1}{1} = coder.typeof(0,[10  6],[1 0]);
        ARGS{1}{2} = coder.typeof(0,[3 1]);
        ARGS{1}{3} = coder.typeof(0,[10 10],[1 0]);
        ARGS{1}{4} = coder.typeof(0);
        ARGS{1}{5} = coder.typeof(false);

        cd(fullfile(path, '+PSDM'));

        codegen -config cfg -I path +PSDM/deriveModel -args ARGS{1}

        cd(path);
        fprintf("Done!\n");
    end
    
    %% RUNIDGRAVITY
    
    if any(strcmp(functions, 'deriveGravityModel')) || any(strcmp(functions, 'all'))
        fprintf("Compiling PSDM.deriveGravityModel into mex file... ");

        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ExtrinsicCalls = false;
        cfg.ReportPotentialDifferences = false;

        % Define argument types for entry-point 'runID'.
        ARGS = cell(1,1);
        ARGS{1} = cell(5,1);
        ARGS{1}{1} = coder.typeof(0,[10  6],[1 0]);
        ARGS{1}{2} = coder.typeof(0,[3 1]);
        ARGS{1}{3} = coder.typeof(0,[10 10],[1 0]);
        ARGS{1}{4} = coder.typeof(0);
        ARGS{1}{5} = coder.typeof(false);

        cd(fullfile(path, '+PSDM'));

        codegen -config cfg -I path +PSDM/deriveGravityModel -args ARGS{1}

        cd(path);
        fprintf("Done!\n");
    end
    
    %% GENTERMVALUES
    
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
    
    %% GENTESTPOSES
    
    if any(strcmp(functions, 'generateSamples')) || any(strcmp(functions, 'all'))
    
        fprintf("Compiling PSDM.generateSamples into mex file... ");

        % Create configuration object of class 'coder.MexCodeConfig'.
        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ReportPotentialDifferences = false;
        cfg.ExtrinsicCalls = false;

        % Define argument types for entry-point 'genTestPoses'.
        ARGS = cell(1,1);
        ARGS{1} = cell(6,1);
        ARGS{1}{1} = coder.typeof(0,[10  6],[1 0]);
        ARGS{1}{2} = coder.typeof(0,[10 10],[1 0]);
        ARGS{1}{3} = coder.typeof(0,[3 1], [1 1]);
        ARGS{1}{4} = coder.typeof(0);
        ARGS{1}{5} = coder.typeof(0);
        ARGS_1_6 = cell([1 2]);
        ARGS_1_6{1} = coder.typeof('X',[1 20],[1 1]);
        ARGS_1_6{2} = coder.typeof(0,[1 10],[1 1]);
        ARGS{1}{6} = coder.typeof(ARGS_1_6,[1 2]);
        ARGS{1}{6} = ARGS{1}{6}.makeHeterogeneous();

        cd(fullfile(path, '+PSDM'));
        codegen -config cfg -I path +PSDM/generateSamples -args ARGS{1}

        cd(path);
        fprintf("Done!\n");
        
    end
    
    if any(strcmp(functions, 'generateSamples')) || any(strcmp(functions, 'all'))
    
        fprintf("Compiling PSDM.generateSamples into mex file... ");

        % Create configuration object of class 'coder.MexCodeConfig'.
        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ReportPotentialDifferences = false;
        cfg.ExtrinsicCalls = false;

        % Define argument types for entry-point 'genTestPoses'.
        ARGS = cell(1,1);
        ARGS{1} = cell(6,1);
        ARGS{1}{1} = coder.typeof(0,[10  6],[1 0]);
        ARGS{1}{2} = coder.typeof(0,[10 10],[1 0]);
        ARGS{1}{3} = coder.typeof(0,[3 1], [1 1]);
        ARGS{1}{4} = coder.typeof(0);
        ARGS{1}{5} = coder.typeof(0);
        ARGS_1_6 = cell([1 2]);
        ARGS_1_6{1} = coder.typeof('X',[1 20],[1 1]);
        ARGS_1_6{2} = coder.typeof(0,[1 10],[1 1]);
        ARGS{1}{6} = coder.typeof(ARGS_1_6,[1 2]);
        ARGS{1}{6} = ARGS{1}{6}.makeHeterogeneous();

        cd(fullfile(path, '+PSDM'));
        codegen -config cfg -I path +PSDM/generateSamples -args ARGS{1}

        cd(path);
        fprintf("Done!\n");
        
    end
    
    
    %% forwardDynamics
    
    if any(strcmp(functions, 'forwardDynamics')) || any(strcmp(functions, 'all'))
    
        fprintf("Compiling PSDM.forwardDynamics into mex file... ");

        % Create configuration object of class 'coder.MexCodeConfig'.
        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ReportPotentialDifferences = false;
        cfg.IntegrityChecks = false;
        cfg.ResponsivenessChecks = false;
        cfg.ExtrinsicCalls = false;

        % Define argument types for entry-point 'genTestPoses'.
        ARGS = cell(1,1);
        ARGS{1} = cell(6,1);
        ARGS{1}{1} = coder.typeof(uint8(0),[40 10000],[1 1]); % E
        ARGS{1}{2} = coder.typeof(0,[10000  100   10],[1 1 1]); % P
        ARGS{1}{3} = coder.typeof(0,[100  1],[1 0]); % Theta
        ARGS{1}{4} = coder.typeof(0,[10 Inf],[1 1]); % Q
        ARGS{1}{5} = coder.typeof(0,[10 Inf],[1 1]); % Qd
        ARGS{1}{6} = coder.typeof(0,[10 Inf],[1 1]); % tau

        cd(fullfile(path, '+PSDM'));
        codegen -config cfg -I path +PSDM/forwardDynamics -args ARGS{1}

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


end
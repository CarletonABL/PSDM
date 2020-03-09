function make
    % UTILS.MAKE Compiles some of the expensive functions into mex-files
    % for faster computation.
    
    path = fullfile( fileparts(mfilename('fullpath')), '..');
    
    %% Compile blockprod function
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
    
    %% Compile rref function
    fprintf("Compiling utilities.rref into mex file... ");
    cfg = coder.config('mex');
    cfg.GenerateReport = true;
    cfg.ReportPotentialDifferences = false;

    % Define argument types for entry-point 'blockprod_private'.
    ARGS = cell(1,1);
    ARGS{1} = cell(2,1);
    ARGS{1}{1} = coder.typeof(0,[Inf Inf],[1 1]);
    ARGS{1}{2} = coder.typeof(0,[1 1],[0 0]);
    ARGS{1}{3} = coder.typeof(false,[1 1],[0 0]);

    % Invoke MATLAB Coder.
    cd(fullfile(path, '+utilities'));
    
    codegen -config cfg -I path +utilities/rref -args ARGS{1}
    fprintf("Done!\n");
    
    %% Cleanup
    
    fprintf("Done code generation for utilities package.\n");
    
    cd(path);
 
end
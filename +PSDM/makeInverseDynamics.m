function makeInverseDynamics(filename, E, P, Theta, varargin)

    p = inputParser;
    p.addOptional('do_mex', false);
    p.addOptional('parallel', false);
    p.parse(varargin{:});
    opt = p.Results;

    DOF = size(P, 3);
    
    % Make base code
    [Up1, A1, setupCode, Up1names, Up1code, A1names, A1code] = makeBaseCode(E, P);
    
    % Build up Y matrix   
    Ycode = makeYMatrixCode(E, 'Ypi', Up1, Up1names, A1, A1names);
    
    % Build up PTheta code
    PThetaCode = makePThetaCode(P, Theta, 'PTheta');  
    
    %% Make function
    
    [funcDir, funcName, ~] = fileparts(filename);
    
    dir = fileparts(mfilename('fullpath'));
    funcText = fileread( fullfile(dir, 'templates', 'inverseDynamics.m') );
    
    funcText = strrep(funcText, '%SETUPCODE%', setupCode);
    funcText = strrep(funcText, '%UPCODE%', Up1code);
    funcText = strrep(funcText, '%ACODE%', A1code);
    funcText = strrep(funcText, '%YCODE%', Ycode);
    funcText = strrep(funcText, '%PTHETACODE%', PThetaCode);
    funcText = strrep(funcText, 'function tau = inverseDynamics(Q, Qd, Qdd)', ...
        sprintf('function tau = %s(Q, Qd, Qdd)', funcName));
    funcText = strrep(funcText, '%p%', ...
        sprintf('%d', size(E,2)));
    
    if opt.parallel
        % Change loop to a parallel loop
        funcText = strrep(funcText, "for i = 1:N", "parfor i = 1:N");
    end
        
    
    %% Write file
    
    fid = fopen(filename, 'wt');
    fprintf(fid, '%s', funcText);
    fclose(fid);
    
    %% Mex, if desired
    
    if opt.do_mex
        
        fprintf("Compiling into mex file... ");

        % Create configuration object of class 'coder.MexCodeConfig'.
        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ReportPotentialDifferences = false;
        cfg.IntegrityChecks = false;
        cfg.ResponsivenessChecks = false;
        cfg.ExtrinsicCalls = false;

        % Define argument types for entry-point 'genTestPoses'.
        ARGS = cell(1,1);
        ARGS{1} = cell(3,1);
        ARGS{1}{1} = coder.typeof(0,[DOF Inf],[0 1]);
        ARGS{1}{2} = coder.typeof(0,[DOF Inf],[0 1]);
        ARGS{1}{3} = coder.typeof(0,[DOF Inf],[0 1]);
        
        cdir = pwd;
        idir = fullfile(dir, '..');
        compileName = fullfile(funcDir, funcName);
        cd(funcDir)
        codegen(compileName,'-config', cfg, '-args', ARGS{1})
        fprintf(" Done!\n");
        cd(cdir);
        
    end
    
end
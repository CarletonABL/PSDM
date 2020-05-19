function makeForwardDynamics(filename, E, P, varargin)
    % Parse inputs

    p = inputParser;
    p.addOptional('do_mex', false);
    p.addOptional('parallel', false);
    p.addOptional('code_style', 'linear');
    p.addOptional('language', 'matlab');
    p.addOptional('return_all', false);
    p.parse(varargin{:});
    opt = p.Results;
    
    opt.alg = 'FD';
    opt.return_Y = false;
    opt.assign_type = 'newline';
    opt.individual_threshold = 0;
    DOF = size(P, 3);
    
    %% Make base code
    [vars, names, code] = PSDM.fgen.makeSetupCode(E, P, opt);
    [vars, names, code] = PSDM.fgen.makePCode(vars, names, code, opt);
    [vars, names, code] = PSDM.fgen.makeFDCode(vars, names, code, opt);
    
    %% Make function
    
    [funcDir, funcName, ~] = fileparts(filename);
    
    dir = fileparts(mfilename('fullpath'));
    
    extraName = '';
    if opt.return_all
        extraName = '_all';
    end
    
    if strcmp(opt.language, 'matlab')
        funcText = fileread( fullfile(dir, 'templates', opt.language, sprintf('forwardDynamics.%s', opt.language(1)) ));
    else
        funcText = fileread( fullfile(dir, 'templates', opt.language, sprintf('forwardDynamics%s.%s', extraName, opt.language(1)) ));
    end
    
    funcText = strrep(funcText, PSDM.fgen.getTag('SETUP1_CODE', opt), code.setup1);
    funcText = strrep(funcText, PSDM.fgen.getTag('SETUP2_CODE', opt), code.setup2);
    funcText = strrep(funcText, PSDM.fgen.getTag('EXTRA_CODE', opt), code.extra);
    funcText = strrep(funcText, PSDM.fgen.getTag('TAU_CODE', opt), code.tau);
    funcText = strrep(funcText, PSDM.fgen.getTag('NAME_DEF', opt), ...
        sprintf('double %s;\n', strjoin(unique(names.def), ', ')));
    
    funcText = strrep(funcText, 'function Qdd = forwardDynamics(Q, Qd, tau, Theta)', ...
        sprintf('function Qdd = %s(Q, Qd, tau, Theta)', funcName));
    funcText = strrep(funcText, '%p%', ...
        sprintf('%d', size(E,2)));
    
    % Replace other terms
    for i = 1:5
        funcText = strrep(funcText, sprintf('_%dDOF_', i), mat2str(i*DOF));
    end
    funcText = strrep(funcText, '_ell_', mat2str(size(P, 2)));
    funcText = strrep(funcText, '_Dsize_', mat2str(DOF*(DOF+1)/2));
        
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
        ARGS{1} = cell(4,1);
        ARGS{1}{1} = coder.typeof(0,[DOF Inf],[0 1]);
        ARGS{1}{2} = coder.typeof(0,[DOF Inf],[0 1]);
        ARGS{1}{3} = coder.typeof(0,[DOF Inf],[0 1]);
        ARGS{1}{4} = coder.typeof(0, [size(P, 2), 1], [0 0]);
        
        cdir = pwd;
        compileName = fullfile(funcDir, funcName);
        cd(funcDir)
        codegen(compileName,'-config', cfg, '-args', ARGS{1})
        fprintf(" Done!\n");
        cd(cdir);
        
    end
    
end
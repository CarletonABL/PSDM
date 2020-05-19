function makeInverseDynamics(filename, E, P, varargin)
    % MAKEINVERSEDYNAMICS Writes a procedural, faster real-time function to
    % evaluate a model E, P.
    %
    % PSDM.makeInverseDynamics(filename, E, P, varargin)
    %
    % INPUTS:
    %   - filename: The location where the file will be stored. This also
    %       determines the functions name. Name should have a .m extension.
    %       Example: fast_inverse_dynamics.m
    %   - E: A p x 5*DOF uint8 exponent matrix of the PSDM model.
    %   - P: A p x ell x DOF page matrix of the DOF reduction matrices Pi
    %       for the model.
    %
    % This function has no outputs.
    %
    % After generation, the new function can be called as:
    %
    %   tau = fast_inverse_dynamics(Q, Qd, Qdd, Theta)
    %
    % where tau is a DOFxN matrix of joint torques, Q, Qd, Qdd are DOFxN
    % matrices of joint states, and Theta is the theta vector.
    %
    % Name/Value Pairs:
    %   Modify the functioning of the program with these name/value pairs.
    %   - do_mex: If true, after generating the matlab code, the file will
    %       undergo code-generation using matlab's codegen to generate a
    %       mex file. Default: false.
    %   - parallel: If true, the program will allow parallelization of the
    %       program across joint samples. Default: false.
    %   - return_Y: If true, then the function calling syntax becomes
    %       either
    %
    %           [Y] = fast_inverse_dynamics(Q, Qd, Qdd)
    %           [Y, tau] = fast_inverse_dynamics(Q, Qd, Qdd, Theta)
    %
    %       as required.
    %       Default: false

    p = inputParser;
    p.addOptional('do_mex', false);
    p.addOptional('parallel', false);
    p.addOptional('return_Y', false);
    p.addOptional('code_style', 'linear');
    p.addOptional('language', 'matlab');

    p.parse(varargin{:});
    opt = p.Results;
    opt.alg = 'ID';
    opt.assign_type = 'newline';
    opt.individual_threshold = 0;
    
    DOF = size(P, 3);
    
    %% Make base code
    
    [vars, names, code] = PSDM.fgen.makeSetupCode(E, P, opt);
    [vars, names, code] = PSDM.fgen.makePCode(vars, names, code, opt);
    
    tauName = arrayfun(@(i) sprintf('tau[startInd+%d]', i), 0:DOF-1, 'UniformOutput', false);
    [tauCode, names, code] = PSDM.fgen.makeYbCode(vars.E, vars.P, tauName, 'Y_i', names, code, opt);
    code.tau = tauCode;
        
    %% Make function
    
    [funcDir, funcName, ~] = fileparts(filename);
    
    dir = fileparts(mfilename('fullpath'));
    funcText = fileread( fullfile(dir, 'templates', opt.language, sprintf('inverseDynamics.%s', opt.language(1)) ) );
    
    funcText = strrep(funcText, PSDM.fgen.getTag('SETUP1_CODE', opt), code.setup1);
    funcText = strrep(funcText, PSDM.fgen.getTag('SETUP2_CODE', opt), code.setup2);
    funcText = strrep(funcText, PSDM.fgen.getTag('EXTRA_CODE', opt), code.extra);
    funcText = strrep(funcText, PSDM.fgen.getTag('TAU_CODE', opt), code.tau);
    funcText = strrep(funcText, PSDM.fgen.getTag('NAME_DEF', opt), ...
        sprintf('double %s;\n', strjoin(unique(names.def), ', ')));
    
    out_args = 'tau';
    if opt.return_Y
        out_args = '[Y, tau]';
    end
    
    funcText = strrep(funcText, 'function tau = inverseDynamics(Q, Qd, Qdd, Theta)', ...
        sprintf('function %s = %s(Q, Qd, Qdd, Theta)', out_args, funcName));
    funcText = strrep(funcText, '%p%', ...
        sprintf('%d', size(E,2)));
    
    % Replace other terms
    for i = 1:5
        funcText = strrep(funcText, sprintf('_%dDOF_', i), mat2str(i*DOF));
    end
    funcText = strrep(funcText, '_ell_', mat2str(size(P, 2)));
    
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
        % cfg = coder.config('lib');
        cfg.GenerateReport = false;
        cfg.ReportPotentialDifferences = false;
        cfg.IntegrityChecks = false;
        cfg.ResponsivenessChecks = false;
        cfg.ExtrinsicCalls = false;
        cfg.EnableOpenMP = true;

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
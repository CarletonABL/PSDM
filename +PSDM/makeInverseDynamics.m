function makeInverseDynamics(filename, E, P, Theta, varargin)

    p = inputParser;
    p.addOptional('do_mex', false);
    p.addOptional('parallel', false);
    p.addOptional('tau_type', 'vector');
    p.addOptional('mult_type', 'individual');
    p.addOptional('assign_type', 'newline');
    p.parse(varargin{:});
    opt = p.Results;

    DOF = size(P, 3);
    
    % Make base code
    [vars, names, code] = PSDM.fgen.makeSetupCode(E, P, Theta);
    [vars, names, code] = PSDM.fgen.makePhiCode(vars, names, code, 'Phi_b');
    [vars, names, code] = PSDM.fgen.makeUpsilonCode2(vars, names, code, opt);
    [vars, names, code] = PSDM.fgen.makeAccelCode(vars, names, code, opt);
    [vars, names, code] = PSDM.fgen.makeYMatrixCode(vars, names, code, 'Ypi', opt);
    [vars, names, code] = PSDM.fgen.makeTauCode(vars, names, code, opt);
    
    %% Make function
    
    [funcDir, funcName, ~] = fileparts(filename);
    
    dir = fileparts(mfilename('fullpath'));
    funcText = fileread( fullfile(dir, 'templates', 'inverseDynamics.m') );
    
    funcText = strrep(funcText, '%SETUPCODE%', code.setup);
    funcText = strrep(funcText, '%UPCODE%', code.Up);
    funcText = strrep(funcText, '%ACODE%', code.A);
    funcText = strrep(funcText, '%YCODE%', code.Y);
    funcText = strrep(funcText, '%PTHETACODE%', code.Phi);
    funcText = strrep(funcText, '%TAUCODE%', code.tau);
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
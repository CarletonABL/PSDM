function makeForwardDynamics(filename, E, P, Theta, varargin)
    % Parse inputs

    p = inputParser;
    p.addOptional('do_mex', false);
    p.addOptional('parallel', false);
    p.addOptional('mult_type', 'individual');
    p.addOptional('assign_type', 'vector');
    p.addOptional('allow_open_mp', true);
    p.parse(varargin{:});
    opt = p.Results;
    opt.alg = 'FD';
    
    DOF = size(P, 3);
    
    %% Make base code
    [vars, names, code] = PSDM.fgen.makeSetupCode(E, P, Theta, opt);
    [vars, names, code] = PSDM.fgen.makePhiCode(vars, names, code, 'Phi_b', opt);
    [vars, names, code] = PSDM.fgen.makeUpsilonCode(vars, names, code, opt);
    [vars, names, code] = PSDM.fgen.makeAccelCode(vars, names, code, opt);
    [vars, names, code] = PSDM.fgen.makeYMatrixCode(vars, names, code, opt);
    [vars, names, code] = PSDM.fgen.makeTauCode(vars, names, code, opt);
    
    %% Make function
    
    [funcDir, funcName, ~] = fileparts(filename);
    
    dir = fileparts(mfilename('fullpath'));
    funcText = fileread( fullfile(dir, 'templates', 'forwardDynamics.m') );
    
    funcText = strrep(funcText, '%SETUP1_CODE%', code.setup1);
    funcText = strrep(funcText, '%SETUP2_CODE%', code.setup2);
    funcText = strrep(funcText, '%UP_CODE%', code.Up);
    funcText = strrep(funcText, '%A_CODE%', code.A);
    funcText = strrep(funcText, '%Y_CODE%', code.Y);
    funcText = strrep(funcText, '%PHI_CODE%', code.Phi);
    funcText = strrep(funcText, '%TAU_CODE%', code.tau);
    funcText = strrep(funcText, 'DOF', sprintf('%d', DOF));
    funcText = strrep(funcText, 'function Qdd = forwardDynamics(Q, Qd, tau)', ...
        sprintf('function Qdd = %s(Q, Qd, tau)', funcName));
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
        cfg.MATLABSourceComments = true;
        cfg.PreserveVariableNames = 'UserNames';
        cfg.EnableOpenMP = opt.allow_open_mp;

        % Define argument types for entry-point 'genTestPoses'.
        ARGS = cell(1,1);
        ARGS{1} = cell(3,1);
        ARGS{1}{1} = coder.typeof(0,[DOF Inf],[0 1]);
        ARGS{1}{2} = coder.typeof(0,[DOF Inf],[0 1]);
        ARGS{1}{3} = coder.typeof(0,[DOF Inf],[0 1]);
        
        cdir = pwd;
        compileName = fullfile(funcDir, funcName);
        cd(funcDir)
        codegen(compileName,'-config', cfg, '-args', ARGS{1})
        fprintf(" Done!\n");
        cd(cdir);
        
    end
    
end
function makeInverseDynamics(filename, E, P, Theta, varargin)

    p = inputParser;
    p.addOptional('do_mex', false);
    p.addOptional('parallel', false);
    p.addOptional('tau_type', 'vector');
    p.addOptional('mult_type', 'individual');
    p.addOptional('assign_type', 'vector');
    p.addOptional('allow_open_mp', true);
    p.addOptional('combine_type', 'gradual'); % Set to gradual (multiplies in Phi gradually)
    
    % Only applies if combine type is gradual
    % linear: Computes each joint separately, linearly
    % parallel: computes each joint separately but in parallel
    % vectorized: computes each joint vectorized
    p.addOptional('gradual_joint_treatment', 'parallel');   
    p.addOptional('explicite_Phi', false);   
    p.addOptional('explicit_regressor', true);   

    p.addOptional('pre_multiply', true); % If true, pre-multiplies parts of Phi in early to minimize multiplications
    p.parse(varargin{:});
    opt = p.Results;
    opt.alg = 'ID';
    
    if opt.explicit_regressor
        opt.explicite_Phi = true;
    end

    DOF = size(P, 3);
    
    %% Make base code
    
    [vars, names, code] = PSDM.fgen.makeSetupCode(E, P, Theta, opt);
    [vars, names, code] = PSDM.fgen.makePhiCode(vars, names, code, 'Phi_b', opt);
    if opt.explicit_regressor
        [vars, names, code] = PSDM.fgen.makeUpsilonCode_grad_reg(vars, names, code, opt);
    elseif strcmp(opt.combine_type, 'gradual')
        [vars, names, code] = PSDM.fgen.makeUpsilonCode_grad(vars, names, code, opt);
    else
        [vars, names, code] = PSDM.fgen.makeUpsilonCode(vars, names, code, opt);
        [vars, names, code] = PSDM.fgen.makeAccelCode(vars, names, code, opt);
        [vars, names, code] = PSDM.fgen.makeYMatrixCode(vars, names, code, opt);
        [vars, names, code] = PSDM.fgen.makeTauCode(vars, names, code, opt);
        code.extra = '';
    end
    
    %% Make function
    
    [funcDir, funcName, ~] = fileparts(filename);
    
    dir = fileparts(mfilename('fullpath'));
    funcText = fileread( fullfile(dir, 'templates', 'inverseDynamics.m') );
    
    funcText = strrep(funcText, '%SETUP1_CODE%', code.setup1);
    funcText = strrep(funcText, '%SETUP2_CODE%', code.setup2);
    funcText = strrep(funcText, '%UP_CODE%', code.Up);
    funcText = strrep(funcText, '%A_CODE%', code.A);
    funcText = strrep(funcText, '%Y_CODE%', code.Y);
    funcText = strrep(funcText, '%EXTRA_CODE%', code.extra);
    funcText = strrep(funcText, '%PHI_CODE%', code.Phi);
    funcText = strrep(funcText, '%TAU_CODE%', code.tau);
    
    theta_arg = '';
    if opt.explicit_regressor
        theta_arg = ', Theta';
    end
    funcText = strrep(funcText, 'function tau = inverseDynamics(Q, Qd, Qdd)', ...
        sprintf('function tau = %s(Q, Qd, Qdd%s)', funcName, theta_arg));
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
        cfg.EnableOpenMP = opt.allow_open_mp;
        cfg.MATLABSourceComments = true;
        cfg.PreserveVariableNames = 'UserNames';

        % Define argument types for entry-point 'genTestPoses'.
        ARGS = cell(1,1);
        ARGS{1} = cell(3,1);
        ARGS{1}{1} = coder.typeof(0,[DOF Inf],[0 1]);
        ARGS{1}{2} = coder.typeof(0,[DOF Inf],[0 1]);
        ARGS{1}{3} = coder.typeof(0,[DOF Inf],[0 1]);
        if opt.explicit_regressor
            ARGS{1}{4} = coder.typeof(0, [size(P, 2), 1], [0 0]);
        end
        
        cdir = pwd;
        compileName = fullfile(funcDir, funcName);
        cd(funcDir)
        codegen(compileName,'-config', cfg, '-args', ARGS{1})
        fprintf(" Done!\n");
        cd(cdir);
        
    end
    
end
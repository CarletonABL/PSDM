function makeForwardDynamics(filename, E, P, Theta, varargin)

    p = inputParser;
    p.addOptional('do_mex', false);
    p.addOptional('parallel', false);
    p.parse(varargin{:});
    opt = p.Results;
    
    % Need to extract the acceleration terms from E, into a mask
    DOF = size(E, 1) / 5;
    accelMask = E((1:DOF)+4*DOF, :) > 0;
    accelMaskAll = any(accelMask, 1);
    M = size(E, 2);
    
    %% Get base code
    [Up, Up1, A, A1, setupCode, Up1names, Up1code, A1names, A1code] = makeBaseCode(E, P);
    
    %% Build up Yp_induced matrix
    Y_induced = E(:, ~accelMaskAll);
    Up_induced = Y_induced(1:(3*DOF), :);
    A_induced = Y_induced(3*DOF+(1:(2*DOF)), :);
    M_induced = size(Y_induced, 2);
    Yels = repelem({''}, M_induced);
    for i = 1:M_induced
        
        Up_ind = find( all( Up1 == Up_induced(:, i) ), 1);
        A_ind = find( all( A1 == A_induced(:, i) ), 1);
        
        Yels{i} = sprintf('%s.*%s', A1names{A_ind}, Up1names{Up_ind});
    end
    Y_induced_code = sprintf('Yp_induced_i = [%s];', strjoin(Yels, ','));
    
    %% PTHETA_INDUCED CODE
    
    P_induced = P(~accelMaskAll, :, :);
    PTheta_induced = zeros( size(P_induced, 1), DOF );
    for i = 1:DOF
        PTheta_induced(:, i) = P_induced(:, :, i) * Theta;
    end
    
    % Add code
    PTheta_induced_str = mat2str(PTheta_induced);
    PTheta_induced_code = sprintf('PTheta_induced = coder.const(%s);', PTheta_induced_str);
    
    %% MAKE Yp_accel_i code
    
    accelMask_trim = accelMask(:, accelMaskAll);
    Up_accel = Up(:, accelMaskAll);
    Up_accel_code = repelem({''}, DOF);
    Paccel = P(accelMaskAll, :, :);
        
    for i = 1:DOF
        Up_accel_i = Up_accel(:, accelMask_trim(i, :));
        Mi = size(Up_accel_i, 2);
        Up_accel_i_els = repelem({''}, Mi);
        
        for j = 1:Mi
            Up_ind = find( all( Up1 == Up_accel_i(:, j) ), 1);
            Up_accel_i_els{j} = Up1names{Up_ind};
        end
        Up_accel_code{i} = sprintf('Up_accel_%d = [%s];', i, strjoin(Up_accel_i_els, ','));
    end
    
    %% MAKE PTheta_accel_i code
    
    for i = 1:DOF
        
        P_accel_i = Paccel(accelMask_trim(i, :), :, :);
        PTheta_accel_i = zeros( size(P_accel_i, 1), DOF );
        
        for j = 1:DOF
            % Iterate through joint torque being considered (rows of D)
            PTheta_accel_i(:, j) = P_accel_i(:, :, j) * Theta;
        end
        
        % Add code
        PTheta_accel_i_str = mat2str(PTheta_accel_i);
        PTheta_accel_code{i} = sprintf('PTheta_accel_%d = coder.const(%s);', i, PTheta_accel_i_str);
        
    end
    
    %% Make D constructor
    for i = 1:DOF
        D_els{i} = sprintf('(Up_accel_%d * PTheta_accel_%d)''', i,i);
    end
    D_code = sprintf('Di = horzcat(%s);', strjoin(D_els, ','));
    
    %% Make function
    
    [funcDir, funcName, ~] = fileparts(filename);
    
    dir = fileparts(mfilename('fullpath'));
    funcText = fileread( fullfile(dir, 'templates', 'forwardDynamics.m') );
    
    funcText = strrep(funcText, '%SETUP_CODE%', setupCode);
    funcText = strrep(funcText, '%UP_CODE%', Up1code);
    funcText = strrep(funcText, '%A_CODE%', A1code);
    funcText = strrep(funcText, '%Y_INDUCED_CODE%', Y_induced_code);
    funcText = strrep(funcText, '%PTHETA_INDUCED_CODE%', PTheta_induced_code);
    funcText = strrep(funcText, '%UP_ACCEL_CODE%', strjoin(Up_accel_code, '\n'));
    funcText = strrep(funcText, '%PTHETA_ACCEL_CODE%', strjoin(PTheta_accel_code, '\n'));
    funcText = strrep(funcText, '%D_CODE%', D_code);
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
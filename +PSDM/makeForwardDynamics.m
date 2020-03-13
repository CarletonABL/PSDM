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
    
    % Get base code
    [Up1, A1, setupCode, Up1names, Up1code, A1names, A1code] = makeBaseCode(E, P);
    
    % Build up Y_induced matrix   
    Y_induced = E(:, ~accelMaskAll);
    Y_induced_code = makeYMatrixCode(Y_induced, 'Yp_induced_i', Up1, Up1names, A1, A1names);
    
    % Build up PTheta code
    P_induced = P(~accelMaskAll, :, :);
    PTheta_induced_code = makePThetaCode(P_induced, Theta, 'PTheta_induced');  
    
    % Make Up_accel_i code
    accelMask_trim = accelMask(:, accelMaskAll);
    Up_accel = E( 1:(3*DOF) , accelMaskAll);
    Up_accel_code = repelem({''}, DOF);    
    
    for i = 1:DOF
        Up_accel_i = Up_accel(:, accelMask_trim(i, :));
        Up_accel_code{i} = makeUpAccel( Up_accel_i, sprintf('Up_accel_%d', i), Up1, Up1names );
    end
    
    % Make PTheta_accel_i code
    Paccel = P(accelMaskAll, :, :);
    PTheta_accel_code = repelem({''}, DOF);
    
    for i = 1:DOF
        P_accel_i = Paccel(accelMask_trim(i, :), :, :);
        PTheta_accel_code{i} = makePThetaCode(P_accel_i, Theta, sprintf('PTheta_accel_%d', i));  
    end
    
    % Make D constructer
    D_els = repelem({''}, DOF);
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
        compileName = fullfile(funcDir, funcName);
        cd(funcDir)
        codegen(compileName,'-config', cfg, '-args', ARGS{1})
        fprintf(" Done!\n");
        cd(cdir);
        
    end
    
end


function UpCode = makeUpAccel( Up_accel_i, UpName, Up1, Up1names )

    Mi = size(Up_accel_i, 2);
    Up_accel_i_els = repelem({''}, Mi);

    for j = 1:Mi
        Up_ind = find( all( Up1 == Up_accel_i(:, j) ), 1);
        Up_accel_i_els{j} = Up1names{Up_ind};
    end
    
    UpCode = sprintf('%s = [%s];', UpName, strjoin(Up_accel_i_els, ','));

end
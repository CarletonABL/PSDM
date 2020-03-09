function makeInverseDynamics(filename, E, P, Theta, varargin)

    p = inputParser;
    p.addOptional('do_mex', false);
    p.addOptional('parallel', false);
    p.parse(varargin{:});
    opt = p.Results;

    DOF = size(P, 3);
    
    
    %% Make a list of upsilon vectors from input
    scode1 = sprintf('g%d_1 = gi(:, %d);\n', repelem(1:DOF, 2));
    ccode1 = sprintf('g%d_1 = gi(:, %d);\n', repelem(1:DOF, 2)+DOF);
    ccode2 = sprintf('g%d_2 = gi(:, %d).^2;\n', repelem(1:DOF, 2)+DOF);
    acode1 = sprintf('a%d_1 = ai(:, %d);\n', repelem(1:(2*DOF), 2));
    acode2 = sprintf('a%d_2 = ai(:, %d).^2;\n', repelem((1:DOF), 2));
    
    Zvarnames{1} = split(sprintf('g%d_1,', (1:(2*DOF))), ",");
    Zvarnames{2} = split(sprintf('g%d_2,', (1:(2*DOF))), ",");
    Avarnames{1} = split(sprintf('a%d_1,', (1:(2*DOF))), ",");
    Avarnames{2} = split(sprintf('a%d_2,', (1:(2*DOF))), ",");
    
    Up = E(1:(2*DOF), :);
    Up1 = unique(Up', 'rows')';
    A = E((1:(2*DOF))+2*DOF, :);
    A1 = unique(A', 'rows')';
    
    M = size(Up, 2);
    M1 = size(Up1, 2);
    Ma = size(A1, 2);
    
    setupCode = sprintf('%s\n\n%s\n\n%s\n\n%s\n\n%s\n\n', scode1, ccode1, ccode2, acode1, acode2);
    
    %% Build up code for upsilon elements
    Up1codes = repelem({''}, M1);
    Up1names = repelem({''}, M1);
    for i = 1:M1
        str = repelem({''}, 2*DOF);
        for j = 1:size(Up, 1)
            if Up1(j, i) > 0
                str{j} = Zvarnames{Up1(j, i)}{j};
            end
        end
        
        Up1names{i} = sprintf('Up%d', i);
        if any(Up1(:, i) > 0)
            Up1codes{i} = sprintf('Up%d = %s;', i, strjoin(str(Up1(:, i)>0), ".*"));
        else
            Up1codes{i} = sprintf('Up%d = 1;', i);
        end
    end
    Up1code = strjoin(Up1codes, '\n');
    
    %% Build up code for acceleration elements
    A1codes = repelem({''}, Ma);
    A1names = repelem({''}, Ma);
    for i = 1:Ma
        str = repelem({''}, 2*DOF);
        for j = 1:(2*DOF)
            if A1(j, i) > 0
                str{j} = Avarnames{A1(j, i)}{j};
            end
        end
        
        A1names{i} = sprintf('A%d', i);
        if any(A1(:, i) > 0)
            A1codes{i} = sprintf('A%d = %s;', i, strjoin(str(A1(:, i)>0), ".*"));
        else
            A1codes{i} = sprintf('A%d = 1;', i);
        end
    end
    A1code = strjoin(A1codes, '\n');
    
    %% Build up Y matrix    
    Yels = repelem({''}, M);
    for i = 1:M
        
        Up_ind = find( all( Up1 == Up(:, i) ), 1);
        A_ind = find( all( A1 == A(:, i) ), 1);
        
        Yels{i} = sprintf('%s.*%s', A1names{A_ind}, Up1names{Up_ind});
    end
    Ycode = sprintf('Ypi = [%s];', strjoin(Yels, ','));
    
    %% PTHETA CODE
    
    PTheta = zeros( size(P, 1), DOF );
    for i = 1:DOF
        PTheta(:, i) = P(:, :, i) * Theta;
    end
    
    %PTheta( (abs(PTheta(:)) < 1e-13) ) = 0;
    
    PThetastr = mat2str(PTheta);
    
    PThetaCode = sprintf('PTheta = coder.const(%s);', PThetastr);  
    
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
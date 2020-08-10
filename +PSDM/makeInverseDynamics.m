function makeInverseDynamics(filename, E, P, varargin)
    % MAKEINVERSEDYNAMICS Writes a procedural, faster real-time c function
    % to evaluate a model E, P. Function can be automatically mex'ed for
    % use in Matlab, if desired.
    %
    % PSDM.makeInverseDynamics(filename, E, P, varargin)
    %
    % INPUTS:
    %   - filename: The location where the file will be stored. This also
    %       determines the functions name. Name should have a .c extension.
    %       Example: fast_inverse_dynamics.c
    %   - E: A p x 5*DOF uint8 exponent matrix of the PSDM model.
    %   - P: A p x ell x DOF page matrix of the DOF reduction matrices Pi
    %       for the model.
    %
    % This function has no outputs. The generated code will define a single
    % function with the same name as the file. If mex generation is turned
    % on, then additional code will be added to handle the interface to
    % matlab, and compiled into a matlab executable that can be called as:
    %
    %   tau = fast_inverse_dynamics(Q, Qd, Qdd, Theta)
    %
    % where tau is a DOFxN matrix of joint torques, Q, Qd, Qdd are DOFxN
    % matrices of joint states, and Theta is the theta vector.
    %
    % Name/Value Pairs:
    %   Modify the functioning of the program with these name/value pairs.
    %   - mex: If true, after generating the matlab code, the file will
    %       undergo code-generation using matlab's codegen to generate a
    %       mex file. In this case, a header file will not be generated for
    %       the function. Default: false.
    %   - help: If true and mex is set to true, the program will also
    %       generate a helpfile such that it is possible to call:
    %           help fast_forward_dynamics
    %       to get the appropriate calling syntax.
    %       Default: true.
    %   - return_Y: If true, then the function calling syntax becomes
    %       either
    %
    %           [tau] = fast_inverse_dynamics(Q, Qd, Qdd, Theta)
    %           [tau, Y] = fast_inverse_dynamics(Q, Qd, Qdd, Theta)
    %
    %       as required.
    %       Default: false
    %
    % See also: PSDM.makeForwardDynamics.

    p = inputParser;
    p.addOptional('mex', false);
    p.addOptional('return_Y', false);
    p.addOptional('help', true);
    p.addOptional('tol', 1e-9);

    p.parse(varargin{:});
    opt = p.Results;
    opt.alg = 'ID';
    
    DOF = size(P, 3);
    
    %% Make base code
    
    [vars, names, code] = PSDM.fgen.makeSetupCode(E, P, opt);
    [vars, names, code] = PSDM.fgen.makePCode(vars, names, code, opt);
    
    tauName = arrayfun(@(i) sprintf('tau[startInd+%d]', i), 0:DOF-1, 'UniformOutput', false);
    [tauCode, names, code] = PSDM.fgen.makeTauCode(vars.E, vars.P, tauName, 'Y', opt.return_Y, names, code, opt);
    code.tau = tauCode;
        
    %% Make function
    
    [funcDir, names.func, ~] = fileparts(filename);
    dir = fileparts(mfilename('fullpath'));
    
    mexName = '';
    if opt.mex; mexName = '_mex'; end
    allName = '';
    if opt.return_Y; allName = '_all'; end
    funcText = fileread( fullfile(dir, 'templates', sprintf('inverseDynamics%s%s.c', mexName, allName)) );
    
    funcText = PSDM.fgen.makeReplacements( funcText, vars, names, code );
    
    % Write file
    fid = fopen(filename, 'wt');
    fprintf(fid, '%s', funcText);
    fclose(fid);
    
    %% Make header file, if desired
    if ~opt.mex
        headerTex = fileread( fullfile(dir, 'templates', sprintf('inverseDynamics%s%s.h', mexName, allName)) );
        headerTex = PSDM.fgen.makeReplacements( headerTex, vars, names, code );
        
        % Write file
        fid = fopen( fullfile( funcDir, strcat(names.func, '.h') ), 'wt');
        fprintf(fid, '%s', headerTex);
        fclose(fid);
    end
    
    %% Mex, if desired
    
    if opt.mex
        
        fprintf("Compiling into mex file...\n");
        mex(filename, '-R2018a', '-outdir', funcDir)
        fprintf("Done!\n");
        
        % Make help file
        if opt.help
            helpText = fileread( fullfile(dir, 'templates', sprintf('inverseDynamics_help%s.m', allName)) );
            helpText = PSDM.fgen.makeReplacements( helpText, vars, names, code );
            help_filename = fullfile( funcDir, strcat(names.func, '.m') );
            fid = fopen(help_filename, 'wt');
            fprintf(fid, '%s', helpText);
            fclose(fid);
        end
        
    end
    
end
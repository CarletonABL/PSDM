function makeForwardDynamics(filename, E, P, varargin)
    % MAKEFORWARDDYNAMICS Writes a procedural, faster real-time c function
    % to evaluate the forward dynamic model of a PSDM model E, P. Function
    % can be automatically mex'ed for use in Matlab, if desired.
    %
    % PSDM.makeForwardDynamics(filename, E, P, varargin)
    %
    % INPUTS:
    %   - filename: The location where the file will be stored. This also
    %       determines the functions name. Name should have a .c extension.
    %       Example: fast_forward_dynamics.c
    %   - E: A p x 5*DOF uint8 exponent matrix of the PSDM model.
    %   - P: A p x ell x DOF page matrix of the DOF reduction matrices Pi
    %       for the model.
    %
    % This function has no outputs. The generated code will define a single
    % function with the same name as the file. If mex generation is turned
    % on, then additional code will be added to handle the interface to
    % matlab, and compiled into a matlab executable that can be called as:
    %
    %   Qdd = fast_forward_dynamics(Q, Qd, tau, Theta)
    %
    % where Qdd is a DOFxN matrix of joint accelerations, Q, Qd, are DOFxN
    % matrices of joint states, tau is a DOFxN matrix of applied joint 
    % torques, and Theta is the theta vector.
    %
    % Name/Value Pairs:
    %   Modify the functioning of the program with these name/value pairs.
    %   - mex: If true, after generating the matlab code, the file will
    %       undergo code-generation using matlab's codegen to generate a
    %       mex file. Default: false.
    %   - help: If true and mex is set to true, the program will also
    %       generate a helpfile such that it is possible to call:
    %           help fast_forward_dynamics
    %       to get the appropriate calling syntax.
    %       Default: true.
    %   - return_all: If true, then the function calling syntax becomes
    %       either
    %
    %           [Qdd] = fast_forward_dynamics(Q, Qd, tau, Theta)
    %           [Qdd, tau_induced, D] = fast_forward_dynamics(Q, Qd, tau, Theta)
    %
    %       as required.
    %       Default: false
    %
    % See also: PSDM.makeForwardDynamics.

    p = inputParser;
    p.addOptional('mex', false);
    p.addOptional('help', true);
    p.addOptional('return_all', false);
    p.parse(varargin{:});
    opt = p.Results;
    
    opt.alg = 'FD';
    
    %% Make base code
    [vars, names, code] = PSDM.fgen.makeSetupCode(E, P, opt);
    [vars, names, code] = PSDM.fgen.makePCode(vars, names, code, opt);
    [vars, names, code] = PSDM.fgen.makeFDCode(vars, names, code, opt);
    
    %% Make function
    
    [funcDir, names.func, ~] = fileparts(filename);
    dir = fileparts(mfilename('fullpath'));
    
    mexName = '';
    if opt.mex; mexName = '_mex'; end
    allName = '';
    if opt.return_all; allName = '_all'; end
    funcText = fileread( fullfile(dir, 'templates', sprintf('forwardDynamics%s%s.c', mexName, allName)) );
    
    funcText = PSDM.fgen.makeReplacements( funcText, vars, names, code );
        
    
    %% Write file
    fid = fopen(filename, 'wt');
    fprintf(fid, '%s', funcText);
    fclose(fid);
    
    %% Mex, if desired
    
    if opt.mex
        
        fprintf("Compiling into mex file...\n");
        mex(filename, '-R2018a', '-outdir', funcDir)
        fprintf("Done!\n");
        
        % Make help file
        if opt.help
            helpText = fileread( fullfile(dir, 'templates', sprintf('forwardDynamics_help%s.m', allName)) );
            helpText = PSDM.fgen.makeReplacements( helpText, vars, names, code );
            help_filename = fullfile( funcDir, strcat(names.func, '.m') );
            fid = fopen(help_filename, 'wt');
            fprintf(fid, '%s', helpText);
            fclose(fid);
        end
        
    end
    
end
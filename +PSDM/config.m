function c = config()
    % CONFIG Allows for quick modification of parameters in all functions.

    % Setting this to true will use mex files for low-level functions which
    % run slowly in interpreted matlab code, such as:
    %   - PSDM.inverseDynamicsNewton
    %   - PSDM.generateYp
    %   - PSDM.inverseDynamics
    %   - PSDM.forwardDynamics
    c.use_mex = false;
    
    % If set to true, after each step of the algorithm, extra computations
    % will be done to double-check that the derived model is still exactly
    % accurate. Recommended to keep this as true!
    c.do_reprojection_tests = coder.target('matlab');
    
    % Whether or not to use parallel computing to speed up the algorithm
    % derivation. Note, setting this to true will result in an error, if
    % the Parallel Computing Toolbox isn't installed.
    c.use_par = true;
    
end

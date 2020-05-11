function c = config()
    % CONFIG Allows for quick modification of parameters in all functions.

    % Attempts to compile top-level functions into a mex package. This is
    % slightly faster than executing matlab code, particularly for smaller
    % DOF. At higher DOF, the pre-mexed matlab functions are actually
    % slightly faster and result in better runtimes.
    c.use_mex = false;
    
    % Setting this to true will use mex files for low-level functions which
    % run slowly in interpreted matlab code, such as:
    %   - PSDM.inverseDynamicsNewton
    %   - PSDM.generateYp
    %   - PSDM.inverseDynamics
    %   - PSDM.forwardDynamics
    c.use_mex_basic = false;
    
    % If set to true, after each step of the algorithm, extra computations
    % will be done to double-check that the derived model is still exactly
    % accurate. Recommended to keep this as true!
    c.do_reprojection_tests = true;
    
end
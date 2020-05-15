function x = linsolve(A, b, niter, useSymbolic, exitTolerance)
    % LINSOLVE Computes the least squares solution to A\b, however it
    % "recovers" the final digits of the computation using iteration with
    % newton's method and triple-precision residual calculations.
    %
    %   x = utils.linsolve(A, b) Computes the least squares solution to
    %       A\b with 2 iterative improvement steps using triple precision.
    %
    %   x = utils.linsolve(A, b, niter, useSymbolic, exit_tolerance)
    %       specifies the full algorithm options. 
    %       *   niter is the maximum number of iterations to do
    %           (default: 1). 
    %       *   useSymbolic specifies whether or not to use the symbolic
    %           toolbox for maximum precision in residual calculation. This
    %           produces very accurate results but is much slower and
    %           requires much more memory. Default: false.
    %       *   exit_tolerance: tolerance between improvement in steps
    %           before the algorithm auto-exits early. Default:
    %           eps(class(A)).
    %
    % See also: utils.dot3p, utils.residual3p
    
    %% Parse Defaults
    if nargin < 3 || isempty(niter)
        niter = 1;
    end
    if nargin < 4 || isempty(useSymbolic)
        useSymbolic = false;
    end
    if nargin < 5 || isempty(exitTolerance)
        exitTolerance = eps(class(A));
    end
    
    %% Start function
    coder.extrinsic('utilities.pv.linsolve_base')
    x = coder.nullcopy(zeros(size(A, 2), size(b, 2)));
    x = utilities.pv.linsolve_base(A, b, niter, useSymbolic, exitTolerance);

end
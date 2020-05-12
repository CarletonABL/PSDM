function [robot, opt] = parseArgs(varargin)
    
    varargin1 = varargin{1};

    if isa(varargin1{1}, 'function_handle')
        % Function calling syntax
        
        lt = logical(varargin1{2});
        assert(size(lt, 2) == 1, ...
            'Second argument must be a DOFx1 logical vector of joint types!');
        Nargs = 2;
        
        DOF = size(lt, 2);
        
    else
        % DH calling syntax
        DH = varargin1{1};
        DOF = size(DH, 1);
        
        assert(size(DH, 2) == 6, "Invalid DH table");
        assert(all(abs(DH(:, 6)) == 1), ...
            "Link sign column appears invalid. All numbers must be -1 or 1!");
        assert( all(any([DH(:, 5) == 1, DH(:, 5) == 0], 2), 1), ...
            "Link types column appears invalid. All values must be 0 or 1.");
        
        % Extract gravity with defaults
        if isnumeric(varargin1{2}) && ~isempty(varargin1{2})
            g = varargin1{2};
            assert(abs(sum(g.^2) - 1) < 1e-2, ...
                "Gravity vector is not a unit vector.")
            Nargs = 2;
        else
            g = [0; 0; 1];
            warning("Assuming g = [0;0;1]. Supply a proper g vector to suppress this warning.");
            Nargs = 1 + isempty(varargin1{2});
        end
        
        % Extract link types
        lt = logical(DH(:, 5));
        
        % Define inverse dynamics function
        IDfunc = @(Q, Qd, Qdd, Xi) PSDM.inverseDynamicsNewton(DH, Xi, Q, Qd, Qdd, int8(2), g)';
        
    end
    
    % Extract X with defaults
    if isnumeric(varargin1{3}) && ~isempty(varargin1{3})
        X = varargin1{3};
        assert(size(X, 1) == DOF && size(X, 2) == 10, ...
            'If the third argument given is numeric, it must be a DOFx10 matrix of base inertial parameters.');
        assert(all(X(:, [1, 5, 6, 7]) >= 0, 'all'), ...
            "Negative masses and principle inertias Ixx Iyy Izz are not possible!")
        Nargs = 3;
    else
        % Use a random, perturbed set of base params
        X = ones(DOF, 10) + rand(DOF, 10)*0.2 - 0.1;
        Nargs = Nargs + isempty(varargin1{3});
    end
    
    % Assign robot values
    robot.IDfunc = IDfunc;
    robot.X = X;
    robot.lt = lt;
    robot.DOF = DOF;
    
    % Parse name/value pairs
    p = inputParser;
    p.addOptional('tolerance', 1e-10);
    p.addOptional('verbose', true);
    p.addOptional('gravity_only', false);
    p.parse(varargin1{(Nargs+1):end});
    opt = p.Results;
    
    % Add in short form to have more concise code
    opt.v = opt.verbose;
    opt.tol = opt.tolerance;
    
end
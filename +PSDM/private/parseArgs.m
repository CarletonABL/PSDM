function [robot, opt] = parseArgs(p, varargin)
    
    if isa(varargin{1}, 'function_handle')
        % Function calling syntax
        DH = []; g = [];
        lt = logical(varargin{2});
        assert(size(lt, 2) == 1, ...
            'Second argument must be a DOFx1 logical vector of joint types!');
        Nargs = 2;
        
        DOF = size(lt, 2);
        
    else
        % DH calling syntax
        DH = varargin{1};
        DOF = size(DH, 1);
        
        assert(size(DH, 2) == 6, "Invalid DH table");
        assert(all(abs(DH(:, 6)) == 1), ...
            "Link sign column appears invalid. All numbers must be -1 or 1!");
        assert( all(any([DH(:, 5) == 1, DH(:, 5) == 0], 2), 1), ...
            "Link types column appears invalid. All values must be 0 or 1.");
        
        % Extract gravity with defaults
        if numel(varargin) >= 2 && isnumeric(varargin{2}) && ~isempty(varargin{2})
            g = varargin{2};
            
            if abs(sum(g.^2) - 1) > 1e-2 && coder.target('matlab')
                warning("Warning! Gravity vector is not a unit vector. Did you enter it correctly?");
            end
            
            Nargs = 2;
        else
            g = [0; 0; 1];
            warning("Assuming g = [0;0;1]. Supply a proper g vector to suppress this warning.");
            Nargs = 1 + double(numel(varargin) >= 2 && isempty(varargin{2}));
        end
        
        % Extract link types
        lt = logical(DH(:, 5));
        
        % Define inverse dynamics function
        IDfunc = @(Qf, Qdf, Qddf, Xf, g_scale) PSDM.inverseDynamicsNewton(DH, Xf, Qf, Qdf, Qddf, int8(2), g*g_scale)';
        
    end
    
    % Extract X with defaults
    if numel(varargin) >= 3 && isnumeric(varargin{3}) && ~isempty(varargin{3})
        X = varargin{3};
        assert(size(X, 1) == DOF && size(X, 2) == 10, ...
            'If the third argument given is numeric, it must be a DOFx10 matrix of base inertial parameters.');
        assert(all(X(:, [1, 5, 6, 7]) >= 0, 'all'), ...
            "Negative masses and principle inertias Ixx Iyy Izz are not possible!")
        Nargs = 3;
    else
        % Use a random, perturbed set of base params
        X = ones(DOF, 10) + rand(DOF, 10)*0.2 - 0.1;
        Nargs = Nargs + double(numel(varargin) >= 3 && isempty(varargin{3}));
    end
    
    % Assign robot values
    robot.IDfunc = IDfunc;
    robot.X = X;
    robot.lt = lt;
    robot.DOF = DOF;
    robot.DH = DH;
    robot.g = g;
    
    % Parse name/value pairs
    p.parse(varargin{(Nargs+1):end});
    opt = p.Results;
    
    
end
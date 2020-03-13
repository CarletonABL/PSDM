function A = getSetA(DOF, type)
    % GETSETA Produces the accelerations set for a manipulator of DOF
    % degrees of freedom.
    %
    % Inputs:
    %   -DOF: The number of degrees of freedom of the robot.
    %   -type: The type of accelerations desired. Options are: 'gravity',
    %   'velocity', 'coriolis', 'centrifugal', 'joint' or 'all'.
    %
    % Output:
    %   -A: The functions in the set A for the manipulator,
    %       represented as a (2*DOF)xM integer matrix of exponents on the
    %       functions: Qd, and Qdd, respectively.
    
    %% Parse inputs
    
    % Handle incomplete arguments
    if nargin < 2
        type = 'all';
    end
    
    %% Start function
    
    % Gravity accel
    Agrav = zeros(2*DOF, 1, 'uint8');
    
    % Centrifugal accel
    Acent = vertcat(uint8(2*eye(DOF)), zeros(DOF, 'uint8'));
    
    % Joint accel
    Ajoint = vertcat(zeros(DOF, 'uint8'), uint8(eye(DOF)));
    
    % Coriolis accels
    Acor_ind = nchoosek(1:DOF, 2);
    Ncor = size(Acor_ind, 1);
    Acor1 = zeros(DOF, Ncor, 'uint8');
    
    ind1 = sub2ind([DOF, Ncor], Acor_ind, repmat((1:Ncor)', [1 2]));
    Acor1(ind1(:)) = uint8(1);
    Acor = vertcat(Acor1, zeros(DOF, size(Acor1, 2), 'uint8'));
    
    % Output correct type
    switch type
        case 'gravity'
            A = Agrav;
        case 'coriolis'
            A = Acor;
        case 'centrifugal'
            A = Acent;
        case 'joint'
            A = Ajoint;
        case 'velocity'
            A = horcat(Acor, Acent);
        case 'all'
            A = horzcat(Agrav, Acent, Acor, Ajoint);
        otherwise
            error("Invalid type given!");
    end
    
end
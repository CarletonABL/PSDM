classdef Denso3 < SerialManipulator & ...
                   handle
    % DENSO Serial manipulator implementation of the Denso 6556 robot.
    
    %=================================================================%
    % Define main properties of denso manipulator
    %=================================================================%
    properties (Constant)
        
        DOF = 3;
        
        DH = [75e-3     -pi/2+.1       132e-3      0;
              -270e-3   0           0           +pi/2
              -90e-3    pi/2        0           -pi/2];
          
        linkTypes = [0; 0; 0];
        
        Qsign = [1; 1; 1];
        
        nominalPose = deg2rad([0 0 90]');
        
        Tbase = [1,0,0,0;
                 0,1,0,0;
                 0,0,1,0;
                 0,0,0,1];
                       
    end
    properties (Hidden, Constant)
                    
        useCodeGen = false;
        
        % I is a 6 x n vector. Each row is the inertia of that link in a
        % frame parallel to the DH frame.
        % Order of variables is: Ixx, Iyy, Izz, Ixy, Ixz, Iyz
        Ilist = [0.0430 0.0449 0.0490 0.0125 0.0022 -0.0024;
                   0.0060 0.0393 0.0415 -0.0004 -0.0012 0.01;
                   0.0278 0.0375 0.0309 0.0010 0.0005 -0.002];
                   %0.0138 0.0058 0.0123 -0.005 0.0004 0.05];
                   %0.0009 0.0010 0.0006 0.0001 0.0001 0.0001;
                   %0.0020 0.0300 0.0100 0.0200 0.0300 0.0200];
               
        % m
        % Note, weight of 6 combined into 5.
        m = [7.6; 2.8; 7]%; 2.9];% 0.94; 0.3];%; 0]
        
        % rc
        rc = [-0.0376,     0.1554      0.045;%       -0.001;%       0.0001      0.005;
              0.0435,      -0.0062      -0.007;%      0.1021;%      -0.000      0.005;
              -0.0076,     0.1148       0.004]%       -0.0028]%     0.0106      0.005];
        
        % Gravity direction vector
        g = [0; 0; 1];
              
    end
    
    %=================================================================%
    % Collision Detection overrides
    %=================================================================%
    properties (Constant, Hidden)
        % One row for each collision detection exception.
        % By default, adjacent frames are not checked.
        % Can add additional exceptions here if it is known that two frames
        % cannot collide.
        % 0 = world frame
        % 1-6 => link frames
        CollisionDetectionFrameExceptions = ...
           [];
        
        defaultsOverride = struct('speedDerate', 0.2, ...
                                  'Ts', 1e-3, ...
                                  'CDTolerance', 0.02, ...
                                  'CDIterMax', 20, ...
                                  'CDRelTolerance', 2e-4 / 1.5 / 1e2);
                              
    end
    
    methods
        
        %=================================================================%
        % Constructor
        %=================================================================%
        function Robot = Denso3(varargin)
            % SerialManipulatorWithCodeGen Construct an instance of this class
            
            Robot@SerialManipulator(varargin{:});
                        
        end
        
    end
    
    
end
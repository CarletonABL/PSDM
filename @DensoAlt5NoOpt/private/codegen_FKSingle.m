% AUTOGENERATED 29-Feb-2020 by /Users/steffanlloyd/Documents/School/00_Thesis/Robodeb/repositories/RoboTools/@SerialManipulator/generateFK
% Computes the forward kinematics transform for variables Q and frame i
% Q -> 6 x 1 vector of joint variables
% i -> the frame number. 
%       1 - n : frame
function T = codegen_FKSingle(Q, i)
    %#codegen
    i = int8(i);
          
    % Get frame
    switch i
        
		case 1
			T = codegen_FK_1(Q);

		case 2
			T = codegen_FK_2(Q);

		case 3
			T = codegen_FK_3(Q);

		case 4
			T = codegen_FK_4(Q);

		case 5
			T = codegen_FK_5(Q);

        
        otherwise
            error('No such frame defined');
    end
    
    coder.varsize('T', [4, 4], [0 0]);
        
end
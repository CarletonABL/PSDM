function PThetaCode = makePThetaCode(P, Theta, PThetaName)
    % MAKEPTHETACODE Generates the code for defining PTheta in a function

    DOF = size(P, 3);

    PTheta = zeros( size(P, 1), DOF );
    for i = 1:DOF
        PTheta(:, i) = P(:, :, i) * Theta;
    end
    
    PThetastr = mat2str(PTheta);
    
    PThetaCode = sprintf('%s = coder.const(%s);', PThetaName, PThetastr);
    
end
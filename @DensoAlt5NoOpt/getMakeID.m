function id = getMakeID(type)
    % Returns the ID of the robot when the symbolic codegen was run on the
    % functions of type "type", where type can be: 'FK', 'jacobian',
    % 'dynamics', or 'reg_dynamics'   

    try
        if strcmp(type, 'FK')
            id = codegen_FK_getMakeID();
        elseif strcmp(type, 'jacobian')
            id = codegen_jacobian_getMakeID();
        elseif strcmp(type, 'dynamics')
            id = codegen_dynamics_getMakeID();
        elseif strcmp(type, 'reg_dynamics')
            id = codegen_reg_dynamics_getMakeID();
        end
    catch
        id = NaN;
    end

end
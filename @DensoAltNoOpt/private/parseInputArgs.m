function [scene, args] = parseInputArgs(scene, varargin)
    % Need to do input parsing for Denso constructor in a weird way for
    % codegen.
    % So, use a private function to do it.
    
    if nargin==1 && strcmp(scene, 'empty')
        
        args = {'empty'};

    elseif nargin <= 1
        
        args = cell(1);
        args{1} = Denso.defaultsOverride;
        scene = [];
        
    else
        
        args = varargin;
        
    end
    
    if ~strcmp(scene, 'empty') && ...
       (nargin == 0 || isempty(scene) || ~isa(scene, 'function_handle'))
        scene = Denso.defaultScene;
    end
    
end
classdef Controller
    %CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        type
        show
        % global coordinates
        estimPosition
        estimVelocity
        estimOrientation
        particles
        map;
        mapEmpty;
        mapFull;
        map10;
        map100;
        MAP_RESOLUTION;        
    end
    
    methods
        function obj = Controller(inputArg1)
            obj.type=inputArg1;
        end
        [command,controller,vehicle]=calcCommand(obj,vehicle);
    end
end


classdef Vehicle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position
        orientation

        speed
        timeStep
        intertialSensorValue
        sensorData
        lowpassOrientation=0;

    end
    
    methods
        obj = timeUpdate(obj,command);
        obj = getSensorData(obj);
    end
end


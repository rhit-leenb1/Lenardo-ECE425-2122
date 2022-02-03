classdef MobileGUI < matlab.mixin.SetGet
    %PLATELOADER Controls the Beckman Coulter Plate Loader Robot
    %   Performs the basic actions to control the plate loader
    
    properties
        LEOserial
        MapPath
        Map
    end
    
    methods
        function obj = MobileGUI(portNumber,baudrate)
            fprintf('Connecting to robot...\n\n');
            portStr = sprintf('COM%d',portNumber);
            obj.LEOserial = serialport(portStr, baudrate);
        end
        
        function setSerial(portNumber,baudrate)
            fprintf('Connecting to robot...\n\n');
            portStr = sprintf('COM%d',portNumber);
            obj.LEOserial = serialport(portStr, baudrate);
        end
    end
end
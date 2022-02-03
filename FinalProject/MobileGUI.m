classdef MobileGUI < matlab.mixin.SetGet
    %PLATELOADER Controls the Beckman Coulter Plate Loader Robot
    %   Performs the basic actions to control the plate loader
    
    properties
        LEOserial
        MapPath
        maplength
        Map
    end
    
    methods
        function obj = MobileGUI(portNumber,baudrate)
            fprintf('Connecting to robot...');
            portStr = sprintf('COM%d',portNumber);
            obj.LEOserial = serialport(portStr, baudrate);
        end
        
        function pathPlanned = topPathPlan(obj, mapText, start, goal)
            obj.Map = mapText;
            obj.maplength = size(obj.Map);
            Amap = ones(obj.maplength(1), obj.maplength(2))*100;
            Amap(start(1),start(2)) = -99;
            Amap(goal(1),goal(2)) = 0;
            Amap = obj.distanceMapping(Amap,goal(1),goal(2),0);
            
            
        end
        
        function AmapReturn = distanceMapping(obj, Amap, x, y, value)
            if x < 1 || x > obj.maplength(1)
                AmapReturn = Amap;
                return 
            end
            if y < 1 || y > obj.maplength(2)
                AmapReturn = Amap;
                return 
            end
            
            if Amap(x,y) == -99 
                AmapReturn = Amap;
                return 
            end
            
            if Amap(x,y) == 100 || Amap(x,y) == 0
                Amap = obj.distanceMapping(Amap,x+1,y,value+1);
                Amap = obj.distanceMapping(Amap,x-1,y,value+1);
                Amap = obj.distanceMapping(Amap,x,y+1,value+1);
                Amap = obj.distanceMapping(Amap,x,y-1,value+1);
            end
            if Amap(x,y) > value 
                Amap(x,y) = value;
            end
            AmapReturn = Amap;
        end
        
    end
end
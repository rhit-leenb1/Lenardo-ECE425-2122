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
            
            % using wavefront planning / A* / dijkstra path planning for
            % distances of map
            Amap = ones(obj.maplength(1), obj.maplength(2))*-1;
            visitedNodes = zeros(obj.maplength(1), obj.maplength(2));
            Amap(start(1),start(2)) = -99;
            Amap(goal(1),goal(2)) = 0;
            Amap = obj.distanceMapping(Amap,goal(1),goal(2),0,visitedNodes);
            
            % taking Amap and using largest derivative to path plan for LEO
            
            
            
        end
        
        
        
        function AmapReturn = distanceMapping(obj, Amap, x, y, distance,visitedNodes)
            
            % end conditionals
            if Amap(x,y) == -99 
                AmapReturn = Amap;
                return 
            end
            
            if ~(visitedNodes(x,y) == 0)
                AmapReturn = Amap;
                return
            end
            
            % setting visited nodes per recursion
            visitedNodes(x,y) = 1;
            
            % conditionals for where directions can spread (walls and edge cases)
            if x+1 <= obj.maplength(1)
                Amap = obj.distanceMapping(Amap,x+1,y,distance+1, visitedNodes);
            end
            if x-1 > 0
                Amap = obj.distanceMapping(Amap,x-1,y,distance+1,visitedNodes);
            end
            if y+1 <= obj.maplength(2)
                Amap = obj.distanceMapping(Amap,x,y+1,distance+1,visitedNodes);
            end
            if y-1 > 0
                Amap = obj.distanceMapping(Amap,x,y-1,distance+1,visitedNodes);
            end
            
            
            % setting map block distance to minimum distance
            if Amap(x,y) > 0 && Amap(x,y) > distance
                Amap(x,y) = distance;
            end
            
            if Amap(x,y) == -1
                Amap(x,y) = distance;
            end
            
            AmapReturn = Amap;
        end
        
    end
end
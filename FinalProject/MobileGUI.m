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
           % obj.LEOserial = serialport(portStr, baudrate);
        end
        
        function pathPlanned = topPathPlan(obj, mapText, start, goal)
            obj.Map = mapText;
            obj.maplength = size(obj.Map);
            
            % using wavefront planning / A* / dijkstra path planning for
            % distances of map
            Amap = ones(obj.maplength(1), obj.maplength(2))*99;
            visitedNodes = zeros(obj.maplength(1), obj.maplength(2));
            Amap(start(2),start(1)) = norm(obj.maplength)^2; % need to transpose xy
            Amap(goal(2),goal(1)) = 0; % need to transpose xy
            Amap = obj.distanceMapping(Amap,goal(2),goal(1),0,visitedNodes); % transposed xy
            
            % taking Amap and using largest derivative to path plan for LEO
            pathPlanned = obj.topologicalPath(Amap,start(2),start(1));
            
        end
        
        function plannedPath = topologicalPath(obj, Amap,x,y)
            plannedPath = "S";
            currentValue = Amap(x,y);
            facingDirection = "N";
            
            while currentValue > 0
                if facingDirection == "N"
                    if y > 1
                        if Amap(x,y-1)<currentValue 
                            y = y-1;
                            facingDirection = "W";
                            plannedPath = plannedPath + "L";
                            currentValue = Amap(x,y);
                        end
                    end
                    if  x > 1
                        if Amap(x-1,y)<currentValue
                            x = x-1;
                            currentValue = Amap(x,y);
                        end
                    end
                    if  x < obj.maplength(1)
                        if Amap(x+1,y)<currentValue
                            Amap(x,y) = currentValue;
                            x = x+1;
                            facingDirection = "S";
                            plannedPath = plannedPath + "U";
                            currentValue = Amap(x,y);
                        end
                    end
                    if  y < obj.maplength(2)
                        if Amap(x,y+1)<currentValue
                             y = y+1;
                             facingDirection = "E";
                             plannedPath = plannedPath + "R";
                             currentValue = Amap(x,y);
                        end
                    end
                end
                
                if facingDirection == "S"
                    if y > 1
                        if Amap(x,y-1)<currentValue
                            y = y-1;
                            facingDirection = "W";
                            plannedPath = plannedPath + "R";
                            currentValue = Amap(x,y);
                        end
                    end
                    if  x < obj.maplength(1)
                        if Amap(x+1,y)<currentValue
                            x = x+1;
                            currentValue = Amap(x,y);
                        end
                    end
                    if  x > 1
                        if Amap(x-1,y)<currentValue
                            Amap(x,y) = currentValue;
                            x = x-1;
                            facingDirection = "N";
                            plannedPath = plannedPath + "U";
                            currentValue = Amap(x,y);
                        end
                    end
                    if  y < obj.maplength(2)
                        if Amap(x,y+1)<currentValue
                            y = y+1;
                            facingDirection = "E";
                            plannedPath = plannedPath + "L";
                            currentValue = Amap(x,y);
                        end    
                    end    
                end
                
                if facingDirection == "W"
                    if x > 1
                        if Amap(x-1,y)<currentValue
                            x = x-1;
                            facingDirection = "N";
                            plannedPath = plannedPath + "R";
                            currentValue = Amap(x,y);
                        end
                    end
                    if  y > 1
                        if Amap(x,y-1)<currentValue
                            y = y-1;
                            currentValue = Amap(x,y);
                        end
                    end
                    if  y < obj.maplength(2)
                        if Amap(x,y+1)<currentValue
                            Amap(x,y) = currentValue;
                            y = y+1;
                            facingDirection = "E";
                            plannedPath = plannedPath + "U";
                            currentValue = Amap(x,y);
                        end
                    end
                    if  x < obj.maplength(2)
                        if Amap(x+1,y)<currentValue
                            x = x+1;
                            facingDirection = "S";
                            plannedPath = plannedPath + "L";
                            currentValue = Amap(x,y);    
                        end
                    end
                end
                
                
                if facingDirection == "E"
                    if x > 1
                        if Amap(x-1,y)<currentValue
                            x = x-1;
                            facingDirection = "N";
                            plannedPath = plannedPath + "L";
                            currentValue = Amap(x,y);
                        end
                    end
                    if  y < obj.maplength(2)
                        if Amap(x,y+1)<currentValue
                            y = y+1;
                            currentValue = Amap(x,y);
                        end
                    end
                    if  y > 1
                        if Amap(x,y-1)<currentValue
                            Amap(x,y) = currentValue;
                            y = y-1;
                            facingDirection = "W";
                            plannedPath = plannedPath + "U";
                            currentValue = Amap(x,y);
                        end
                    end
                    if  x < obj.maplength(2)      
                        if Amap(x+1,y)<currentValue   
                            x = x+1; 
                            facingDirection = "S";
                            plannedPath = plannedPath + "R";     
                            currentValue = Amap(x,y);
                        end
                    end
                end
            end
            plannedPath = plannedPath + "T";
            
            
        end
        
        function AmapReturn = distanceMapping(obj, Amap, x, y, distance,visitedNodes)
            
            % end conditionals
            if Amap(x,y) == norm(obj.maplength)^2
                AmapReturn = Amap;
                return 
            end
            
            if obj.Map(x,y) == 99
                Amap(x,y) = 99;
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
            % South Direction
            if x+1 <= obj.maplength(1) && ~(bitand(obj.Map(x,y),0b0100))
                Amap = obj.distanceMapping(Amap,x+1,y,distance+1, visitedNodes);
            end
            % North Direction
            if x-1 > 0 && ~(bitand(obj.Map(x,y),0b0001))
                Amap = obj.distanceMapping(Amap,x-1,y,distance+1,visitedNodes);
            end
            % East Direction
            if y+1 <= obj.maplength(2) && ~(bitand(obj.Map(x,y),0b0010))
                Amap = obj.distanceMapping(Amap,x,y+1,distance+1,visitedNodes);
            end
            % West Direction
            if y-1 > 0 && ~(bitand(obj.Map(x,y),0b1000))
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
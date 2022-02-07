classdef MobileGUI < matlab.mixin.SetGet
    %PLATELOADER Controls the Beckman Coulter Plate Loader Robot
    %   Performs the basic actions to control the plate loader
    
    properties
        LEOserial
        Map
        maplength
    end
    
    methods
        function obj = MobileGUI(portNumber,baudrate)
            fprintf('Connecting to robot...');
            portStr = sprintf('COM%d',portNumber);
            obj.LEOserial = serialport(portStr, baudrate);
        end
        
        function response = topoRun(obj, command)
            if command == 'S'
                fprintf("\nStarting\n");
                writeline(obj.LEOserial,'Starting')
            elseif command == 'R'
                fprintf("Turning Right\n");
                 writeline(obj.LEOserial,'S -95')
            elseif command == 'L'
                fprintf("Turning Left\n");
                writeline(obj.LEOserial,'S 95')
            elseif command == 'U'
                fprintf("Turning Around\n");
                writeline(obj.LEOserial,'S 180')
            elseif command == 'T'
                fprintf("Terminate\n");
                writeline(obj.LEOserial,'Terminate')
            end
            
            response = readline(obj.LEOserial)
            response = [1 4];
        end
        
        function response = topoRunDUMMY(obj, command, robotPoint)
            if command == 'S'
                fprintf("\nStarting\n");
            elseif command == 'F'
                fprintf("Forward\n");
                if mod(robotPoint(3),4) == 0
                    robotPoint(2) = robotPoint(2) - 1;
                elseif mod(robotPoint(3),4) == 1
                    robotPoint(1) = robotPoint(1) + 1;
                elseif mod(robotPoint(3),4) == 2
                    robotPoint(2) = robotPoint(2) + 1;
                elseif mod(robotPoint(3),4) == 3
                    robotPoint(1) = robotPoint(1) - 1;
                end
                    
            elseif command == 'R'
                fprintf("Turning Right\n");
                robotPoint(3) = robotPoint(3) + 1;
            elseif command == 'L'
                fprintf("Turning Left\n");
                robotPoint(3) = robotPoint(3) - 1;
            elseif command == 'U'
                fprintf("Turning Around\n");
                robotPoint(3) = robotPoint(3) + 2;
            elseif command == 'T'
                fprintf("Terminate\n");
                %writeline(obj.LEOserial,'Terminate')
            end
            response = robotPoint;
            %response = [1 4];
        end
        
        function pathPlanned = pathPlan(obj, mapText, start, goal)
            obj.Map = mapText;
            obj.maplength = size(obj.Map);
            
            % using wavefront planning / A* / dijkstra path planning for
            % distances of map
            Amap = ones(obj.maplength(1), obj.maplength(2))*98;
            visitedNodes = zeros(obj.maplength(1), obj.maplength(2));
            Amap(start(2),start(1)) = norm(obj.maplength)^2; % need to transpose xy
            Amap(goal(2),goal(1)) = 0; % need to transpose xy
            Amap = obj.distanceMapping(Amap,goal(2),goal(1),0,visitedNodes); % transposed xy
            
            % taking Amap and using largest derivative to path plan for LEO
            pathPlanned = obj.topologicalPath(Amap,start(2),start(1),start(3));
            
        end
        
        % Plan a path from a wavefront planning map
        function plannedPath = topologicalPath(obj, Amap,x,y,direction)
            plannedPath = "S"; %Start pathing
            currentValue = Amap(x,y);
            facingDirection = "N";
            if mod(direction,4) == 1
                facingDirection = "E";
            elseif mod(direction,4) == 2
                facingDirection = "S";
            elseif mod(direction,4) == 3
                facingDirection = "W";
            end
            
            while currentValue > 0
                if facingDirection == "N" %check if robot is facing North
                    if y > 1 %checking West (turn left)
                        if Amap(x,y-1)<currentValue && ~(bitand(obj.Map(x,y),0b1000))
                            facingDirection = "W";
                            plannedPath = plannedPath + "L";
                        end
                    end
                    if  x > 1 %checking North (straight)
                        if Amap(x-1,y)<currentValue && ~(bitand(obj.Map(x,y),0b0001))
                            x = x-1;
                            currentValue = Amap(x,y);
                            plannedPath = plannedPath;% + "F";
                        end
                    end
                    if  x < obj.maplength(1) %checking South (turn around)
                        if Amap(x+1,y)<currentValue && ~(bitand(obj.Map(x,y),0b0100))
                            facingDirection = "S";
                            plannedPath = plannedPath + "U";
                        end
                    end
                    if  y < obj.maplength(2) %checking East (turn right)
                        if Amap(x,y+1)<currentValue && ~(bitand(obj.Map(x,y),0b0010))
                             facingDirection = "E";
                             plannedPath = plannedPath + "R";
                        end
                    end
                end
                
                if facingDirection == "S" %checking if robot is facing South
                    if y > 1 %checking West (turn right)
                        if Amap(x,y-1)<currentValue && ~(bitand(obj.Map(x,y),0b1000))
                            facingDirection = "W";
                            plannedPath = plannedPath + "R";
                        end
                    end
                    if  x < obj.maplength(1) %checking South (straight)
                        if Amap(x+1,y)<currentValue && ~(bitand(obj.Map(x,y),0b0100))
                            x = x+1;
                            currentValue = Amap(x,y);
                            plannedPath = plannedPath; %+ "F";
                        end
                    end
                    if  x > 1 %checking North (turn around)
                        if Amap(x-1,y)<currentValue && ~(bitand(obj.Map(x,y),0b0001))
                            facingDirection = "N";
                            plannedPath = plannedPath + "U";
                        end
                    end
                    if  y < obj.maplength(2) %checking East (turn left)
                        if Amap(x,y+1)<currentValue && ~(bitand(obj.Map(x,y),0b0010))
                            facingDirection = "E";
                            plannedPath = plannedPath + "L";
                        end    
                    end    
                end
                
                if facingDirection == "W" %checking if robot facing West
                    if x > 1 %checking North (turn right)
                        if Amap(x-1,y)<currentValue && ~(bitand(obj.Map(x,y),0b0001))
                            facingDirection = "N";
                            plannedPath = plannedPath + "R";
                        end
                    end
                    if  y > 1 %checking West (straight)
                        if Amap(x,y-1)<currentValue && ~(bitand(obj.Map(x,y),0b1000))
                            y = y-1;
                            currentValue = Amap(x,y);
                            plannedPath = plannedPath;% + "F";
                        end
                    end
                    if  y < obj.maplength(2) %checking East (turn around)
                        if Amap(x,y+1)<currentValue && ~(bitand(obj.Map(x,y),0b0010))
                            facingDirection = "E";
                            plannedPath = plannedPath + "U";
                        end
                    end
                    if  x < obj.maplength(2) %checking South (turn left)
                        if Amap(x+1,y)<currentValue && ~(bitand(obj.Map(x,y),0b0100))
                            facingDirection = "S";
                            plannedPath = plannedPath + "L";
                        end
                    end
                end
                
                
                if facingDirection == "E" %checking if robot facing East
                    if x > 1 %checking North (turn left)
                        if Amap(x-1,y)<currentValue && ~(bitand(obj.Map(x,y),0b0001))
                            facingDirection = "N";
                            plannedPath = plannedPath + "L";
                        end
                    end
                    if  y < obj.maplength(2) %checking East (straight)
                        if Amap(x,y+1)<currentValue && ~(bitand(obj.Map(x,y),0b0010))
                            y = y+1;
                            currentValue = Amap(x,y);
                            plannedPath = plannedPath;% + "F";
                        end
                    end
                    if  y > 1 %checking West (turn around)
                        if Amap(x,y-1)<currentValue && ~(bitand(obj.Map(x,y),0b1000))
                            facingDirection = "W";
                            plannedPath = plannedPath + "U";
                        end
                    end
                    if  x < obj.maplength(2) %checking South (turn right)     
                        if Amap(x+1,y)<currentValue && ~(bitand(obj.Map(x,y),0b0100)) 
                            facingDirection = "S";
                            plannedPath = plannedPath + "R";
                        end
                    end
                end
            end
            plannedPath = plannedPath + "T"; %Terminate pathing
            
            plannedPath = convertStringsToChars(plannedPath); %convert String to char array
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
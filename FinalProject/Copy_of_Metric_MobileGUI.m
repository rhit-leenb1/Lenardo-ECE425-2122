classdef Metric_MobileGUI < matlab.mixin.SetGet
    %PLATELOADER Controls the Beckman Coulter Plate Loader Robot
    %   Performs the basic actions to control the plate loader
    
    properties
        LEOserial
        Map
        maplength
    end
    
    methods
        function obj = Metric_MobileGUI(name)
            fprintf('%s initallized', name);
        end
        
        function setSerial(obj, portNumber,baudrate)
            portStr = sprintf('COM%d',portNumber);
            obj.LEOserial = serialport(portStr, baudrate);
        end
        
        function [startPoint, isFound, blockList, visitedMap] = localize(obj, map, oldStart, blockList, visitedMap)
            obj.Map = map;
            isFound = 0;
            writeline(obj.LEOserial,'C');
            blockNum = str2num(readline(obj.LEOserial));
            [Row, column] = find(obj.Map==blockNum);
            if ~isempty(blockList())
                iterNum = length(blockList(:,1));
            else
                iterNum = 0;
            end
            
            if iterNum >= norm(length(map))
                isFound = -1;
                return
            end
            
            % if only possible place
            if length(Row) == 1
                isFound = 1;
                startPoint = [Row column oldStart(3)];
                return
            end
            
            keepRow = zeros(length(Row),1);
            keepcolumn = zeros(length(column),1);
            % follow previous path to see what blocks work
            for block = 1:length(Row)
                x = column(block);
                y = Row(block);
                followPath = 1;
                
                if visitedMap(y,x) == 0
                    visitedMap(y,x) = 1;

                    % retracing old steps
                    for ind = 1:iterNum
                        if mod(blockList(ind,2),4) == 0
                            if map(y+1,x) == blockList(ind,1)
                                y = y+1;
                            else
                                followPath = 0;
                                break
                            end
                        elseif mod(blockList(ind,2),4) == 1
                            if map(y,x-1) == blockList(ind,1)
                                x = x - 1;
                            else
                                followPath = 0;
                                break
                            end
                        elseif mod(blockList(ind,2),4) == 2
                            if map(y-1,x) == blockList(ind,1)
                                y = y - 1;
                            else
                                followPath = 0;
                                break
                            end
                        elseif mod(blockList(ind,2),4) == 3
                            if map(y,x+1) == blockList(ind,1)
                                x = x + 1;
                            else
                                followPath = 0;
                                break
                            end
                        end
                    end
                    % filter through previous steps and keeping paths that
                    % followed correctly
                    if followPath == 1 && iterNum > 0
                        keepRow(block) = Row(block);
                        keepcolumn(block) = column(block);
                    end
                end
            end
            if norm(keepRow) > 0
                keepRow(keepRow==0) = [];
                keepcolumn(keepcolumn==0) = [];
                Row = keepRow;
                column = keepcolumn;
            end
            
            % if only one path that followed for local
            if length(Row) == 1
                isFound = 1;
                startPoint = [Row column oldStart(3)];
                return
            end
            
            startPoint = [Row column];
            startPoint(:,3) = obj.moveNext(blockNum,oldStart(3));
            
            if iterNum < 1
                blockList = [blockNum startPoint(1,3)];
            else
                blockList = [blockNum startPoint(1,3); blockList(:,1) blockList(:,2)];
            end
            
        end
        
        function newDirection = moveNext(obj, blockNum, facingDirection)
            newDirection = 2;
            % South Direction is open, move direction next for more map info
            if ~(bitand(blockNum,0b0100))
                if mod(facingDirection,4) == 0
                    writeline(obj.LEOserial,'U');
                    readline(obj.LEOserial);
                elseif mod(facingDirection,4) == 1
                    writeline(obj.LEOserial,'R');
                    readline(obj.LEOserial);                
                elseif mod(facingDirection,4) == 2  
                elseif mod(facingDirection,4) == 3
                    writeline(obj.LEOserial,'L');
                    readline(obj.LEOserial);  
                end
            end
            
            % North Direction is open, move direction next for more map info
            if ~(bitand(blockNum,0b0001))
                newDirection = 0;
                if mod(facingDirection,4) == 0
                elseif mod(facingDirection,4) == 1
                    writeline(obj.LEOserial,'L');
                    readline(obj.LEOserial);                
                elseif mod(facingDirection,4) == 2 
                    writeline(obj.LEOserial,'U');
                    readline(obj.LEOserial);
                elseif mod(facingDirection,4) == 3
                    writeline(obj.LEOserial,'R');
                    readline(obj.LEOserial);  
                end
            end
            
            % East Direction is open, move direction next for more map info
            if ~(bitand(blockNum,0b0010))
                newDirection = 1;
                if mod(facingDirection,4) == 0
                    writeline(obj.LEOserial,'R');
                    readline(obj.LEOserial);  
                elseif mod(facingDirection,4) == 1               
                elseif mod(facingDirection,4) == 2 
                    writeline(obj.LEOserial,'L');
                    readline(obj.LEOserial);
                elseif mod(facingDirection,4) == 3
                    writeline(obj.LEOserial,'U');
                    readline(obj.LEOserial);  
                end
            end

            % West Direction is open, move direction next for more map info
            if ~(bitand(blockNum,0b1000))
                newDirection = 3;
                if mod(facingDirection,4) == 0
                    writeline(obj.LEOserial,'L');
                    readline(obj.LEOserial);  
                elseif mod(facingDirection,4) == 1  
                    writeline(obj.LEOserial,'U');
                    readline(obj.LEOserial);  
                elseif mod(facingDirection,4) == 2 
                    writeline(obj.LEOserial,'R');
                    readline(obj.LEOserial);
                elseif mod(facingDirection,4) == 3
                end
            end
            
            writeline(obj.LEOserial,'F');
            readline(obj.LEOserial);
        end
        
        function response = stringRun(obj, command)
            disp(command)
            writeline(obj.LEOserial,command);
            response = readline(obj.LEOserial)
        end
        
        
        function response = charRunUpdate(obj, command, robotPoint)
            if command == 'S'
                fprintf("\nStarting\n");
                writeline(obj.LEOserial,'S')
            elseif command == 'F'
                %fprintf("Forward\n");
                if mod(robotPoint(3),4) == 0
                    robotPoint(2) = robotPoint(2) - 1;
                elseif mod(robotPoint(3),4) == 1
                    robotPoint(1) = robotPoint(1) + 1;
                elseif mod(robotPoint(3),4) == 2
                    robotPoint(2) = robotPoint(2) + 1;
                elseif mod(robotPoint(3),4) == 3
                    robotPoint(1) = robotPoint(1) - 1;
                end
                writeline(obj.LEOserial,'F')
                    
            elseif command == 'R'
                %fprintf("Turning Right\n");
                robotPoint(3) = robotPoint(3) + 1;
                writeline(obj.LEOserial,'R')
            elseif command == 'L'
                %fprintf("Turning Left\n");
                robotPoint(3) = robotPoint(3) - 1;
                writeline(obj.LEOserial,'L')
            elseif command == 'U'
                %fprintf("Turning Around\n");
                robotPoint(3) = robotPoint(3) + 2;
                writeline(obj.LEOserial,'U')
            elseif command == 'T'
                %fprintf("Terminate\n");
                writeline(obj.LEOserial,'T')
            end
            readline(obj.LEOserial)
            response = robotPoint;
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
                            plannedPath = plannedPath + "F";
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
                            plannedPath = plannedPath + "F";
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
                            plannedPath = plannedPath + "F";
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
                            plannedPath = plannedPath + "F";
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
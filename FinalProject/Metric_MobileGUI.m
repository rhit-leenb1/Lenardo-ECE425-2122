classdef Metric_MobileGUI < matlab.mixin.SetGet
    %PLATELOADER Controls the Beckman Coulter Plate Loader Robot
    %   Performs the basic actions to control the plate loader
    
    properties
        LEOserial
        Map
        maplength
    end
    
    methods
        %{
        Generic start up function to initialize object connection
        %}
        function obj = Metric_MobileGUI(name)
            fprintf('%s initallized', name);
        end
        
        %{
        Intialize com port connection via bluetooth
        %}
        function text = setSerial(obj, portNumber,baudrate)
            portStr = sprintf('COM%d',portNumber); % setting com port in string
            obj.LEOserial = serialport(portStr, baudrate); % connecting and storing serial port
            text = readline(obj.LEOserial)
        end
        
        %{
        Localization method:
            1. ping C to get block value
            2. locate block on map
            3. iterate through previous pathing from current block to
            verify which options can be localized
        %}
        function [startPoint, isFound, blockList, visitedMap] = localize(obj, map, oldStart, blockList, visitedMap)
            obj.Map = map;
            isFound = 0;
            writeline(obj.LEOserial,'C');
            returnedValue = readline(obj.LEOserial);
            blockNum = str2num(returnedValue);
            %blockNum = obj.tranformOri(blockNum, oldStart);
            [Row, column] = find(obj.Map==blockNum);
            if ~isempty(blockList())
                iterNum = length(blockList(:,1));
            else
                iterNum = 0;
            end
            
            % limited number iteration
            if iterNum >= 16
                startPoint = oldStart;
                isFound = -1;
                return
            end
            
            % if only on single possible block exception
            if length(Row) == 1
                isFound = 1
                startPoint = [column Row oldStart(1,3)];
                return
            end
            
            keepRow = zeros(length(Row),1);
            keepcolumn = zeros(length(column),1);
            % follow previous path to see what blocks work
            for block = 1:length(Row)
                x = column(block);
                y = Row(block);
                followPath = 1;
                
                % track visited places to prevent looping back
                if visitedMap(y,x) == 0
                    visitedMap(y,x) = 1;

                    % retracing old steps and path to remove options
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
            % removes all non possible blocks based upon previous pathing
            if norm(keepRow) > 0
                keepRow(keepRow==0) = [];
                keepcolumn(keepcolumn==0) = [];
                Row = keepRow;
                column = keepcolumn;
            end
            
            % if only one path that followed for local
            if length(Row) == 1
                isFound = 1;
                startPoint = [column Row oldStart(3)];
                return
            end
            
            % all posssible block locations on the map
            startPoint = [column Row];
            startPoint(:,3) = oldStart(3);
            
            
            % keep memory of previously visited blocks
            if iterNum < 1
                blockList = [blockNum startPoint(1,3)];
            else
                blockList = [blockNum startPoint(1,3); blockList(:,1) blockList(:,2)];
            end
        end
        
        %{
            Move direction for automated localization. Moves to a non
            blocked opening surrounding current robot position.
        %}
        function newDirection = moveNext(obj, blockNum, facingDirection)
            newDirection = 2;
            hasTurn = 0;
            % South Direction is open, move direction next for more map info
            if ~(bitand(blockNum,0b0100)) && hasTurn == 0 && mod(facingDirection,4)~=0
                fprintf("South\n");
                if mod(facingDirection,4) == 0
                    obj.turn('U');
                elseif mod(facingDirection,4) == 1
                    obj.turn('R')          
                elseif mod(facingDirection,4) == 2
                    writeline(obj.LEOserial,'F');
                    response = readline(obj.LEOserial);
                elseif mod(facingDirection,4) == 3
                    obj.turn('L')
                end
                hasTurn = 1;
            end
            
            % North Direction is open, move direction next for more map info
            if ~(bitand(blockNum,0b0001)) && hasTurn == 0 && mod(facingDirection,4)~=2
                fprintf("North\n");
                newDirection = 0;
                if mod(facingDirection,4) == 0
                    writeline(obj.LEOserial,'F');
                    response = readline(obj.LEOserial);
                elseif mod(facingDirection,4) == 1
                    obj.turn('L');       
                elseif mod(facingDirection,4) == 2 
                    obj.turn('U');
                elseif mod(facingDirection,4) == 3
                    obj.turn('R');
                end
                hasTurn = 1;
            end
            
            % East Direction is open, move direction next for more map info
            if ~(bitand(blockNum,0b0010)) && hasTurn == 0 && mod(facingDirection,4)~=3
                fprintf("East\n");
                newDirection = 1;
                if mod(facingDirection,4) == 0
                    obj.turn('R');
                elseif mod(facingDirection,4) == 1 
                    writeline(obj.LEOserial,'F');
                    response = readline(obj.LEOserial);
                elseif mod(facingDirection,4) == 2 
                    obj.turn('L');
                elseif mod(facingDirection,4) == 3
                    obj.turn('U');
                end
                hasTurn = 1;
            end

            % West Direction is open, move direction next for more map info
            if ~(bitand(blockNum,0b1000)) && hasTurn == 0 && mod(facingDirection,4)~=1
                fprintf("West\n");
                newDirection = 3;
                if mod(facingDirection,4) == 0
                    obj.turn('L');
                elseif mod(facingDirection,4) == 1  
                    obj.turn('U');
                elseif mod(facingDirection,4) == 2 
                    obj.turn('R');
                elseif mod(facingDirection,4) == 3
                    writeline(obj.LEOserial,'F');
                    response = readline(obj.LEOserial);
                end
                hasTurn = 1;
            end
        end
        
        function turn(obj, direction)
            writeline(obj.LEOserial,direction);
            readline(obj.LEOserial); 
            writeline(obj.LEOserial,'F');
            readline(obj.LEOserial);
        end
        
        
        %{
            sends string to LEO to run (topological path execution)
        %}
        function response = stringRun(obj, command)
            disp(command)
            writeline(obj.LEOserial,command);
            response = readline(obj.LEOserial)
        end
        
        %{
            For localization to reorientate block reading based upon robot
            orientation to absolute map orientation
            Utilize bit shifting
        %}
        function transformedBlockNum = tranformOri(obj, blocknum, robotPosition)
            transformedBlockNum = fi(blocknum,0,4);
            if mod(robotPosition(3),4) == 0
            elseif mod(robotPosition(3),4) == 1
                transformedBlockNum = bitrol(transformedBlockNum,1);
            elseif mod(robotPosition(3),4) == 2
                transformedBlockNum = bitrol(transformedBlockNum,2);
            elseif mod(robotPosition(3),4) == 3
                transformedBlockNum = bitrol(transformedBlockNum,3);
            end
            transformedBlockNum = str2num(num2str(transformedBlockNum));
        end
        
        %{
            sends character to LEO to run (metric path + localization execution)
        %}
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
            elseif command == 'C'
                %fprintf("Checking Walls\n");
                writeline(obj.LEOserial,'C')
                response = readline(obj.LEOserial);
                return
            end
            response = robotPoint
        end
        
        %{
            Path Plan master function to execute A* mapping and gradient
            descent to manuver through map
        %}
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
        
        %{
            using an A* map, finds a gradient decent and generates path
            planning for metric path planning (previously set for
            topological by removing 'F' characters)
        %}
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
        
        %{
            create a 2d map using A* and wavefront propagation
        %}
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
        % readmatrix fucntion can convert the readline string information
        % to the map martix.
        % type can control the map output.
        % 1 is OGrid
        % 2 is Topo
        function [matrixReturn] = readmatrix(obj, type)
            returnedValue = readline(obj.LEOserial);% read string information
            if strlength(returnedValue)<3           % check the String length (Normally the martix infor is more than 16 char
                matrixReturn = single(0);           % provide a none double output
            else
                matrixNum = str2num(returnedValue); % convert string to number
                if type == 1                        % OGird
                    matrix = reshape(matrixNum,4,4);    %convert to 4x4 matrix
                    matrix(matrix==-1)=15;              %convert all -1 to 15 (same as 99)
                    matrix(matrix<15)=0;            %convert all none 15 to 0 (empty box)
                elseif type == 2                    % Topo
                    matrix = reshape(matrixNum,4,4);    %convert to 4x4 matrix
                    matrix(matrix==-1)=15;              %convert all -1 to 15
                end
            matrixReturn = matrix;                  %send the map matrix
            end
            
        end
        % StartMap will send mapping command 'M'
        function StartMap(obj)
            writeline(obj.LEOserial,'M')
        end
        
        
    end
end
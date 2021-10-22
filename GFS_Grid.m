% Greedy best first search function that is applied to input grid. 
% Takes following Arguments: MAP
%                            Start point
%                            Goal Point
%                            DrawMapEverytime

% Gives following outputs:   Path to goal cell

% Color Code:               GREEN   : Start point
%                           YELLOW  : Goal Point
%                           RED     : Explored
%                           BLUE    : To be Explored (in the frontier)
%                           BLACK   : Obstacle
%                           WHITE   : Empty space
                             

%    Written by Saifeldin Hassan and Tony Hanna

function [route, numExpanded] = GFS_Grid(input_map, start_coords, goal_coords, drawMapEveryTime)


%%%%%%%%%%%%%%%%%%%%%%%
%                     %
%   Initial setup     %
%                     %
%%%%%%%%%%%%%%%%%%%%%%%

% Color map key for display (R G B):
 
cmap = [ 1 1 1; ... % White - 1
         0 0 0; ... % Black - 2
         1 0 0; ... % Red - 3
         0 0 1; ... % Blue - 4 
         0 1 0; ... % Green - 5
         1 1 0; ... % Yellow - 6
         0.5 0.5 0.5]; %Grey - 7
     
colormap(cmap);

% Assign nrows and ncols to the size of input map
[nrows,ncols] = size(input_map); 

% Create map
map = zeros(nrows,ncols);

% Set index to int for start node and goal node
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
goal_node  = sub2ind(size(map), goal_coords(1), goal_coords(2));

% Color Obstacles/clear path in map
map(~input_map) = 1;
map(input_map)  = 2;

% Color start and goal nodes
map(start_node) = 5; %Green
map(goal_node)  = 6; %Yellow

numExpanded = 0; %Init number of nodes expanded to 0
goal_x = goal_coords(1);
goal_y = goal_coords(2);   
maxMapIndex = nrows*ncols;
heuristic_grid = zeros(nrows,ncols);
numExpanded = 0;
Visited_Frontier = start_node;
frontierList = CList(start_node);
frontierNode = frontierList.front();


% Heuristic function grid building to be used for along
% regular path planning.   
% Heuristic can be called with index as well.

for (n = 1:nrows)
     for(n2 = 1:ncols)
         h = abs(n - goal_x) + abs(n2 - goal_y);
         heuristic(n,n2) = h;
     end        
end
    




heuristicNode = heuristic(start_coords(1),start_coords(2));
heuristicList = CList(heuristicNode);
   
step_count = 0;

% 
% 
% Main loop
% 
%

while true
    

    
    %Draw Frontier map
    map(start_node) = 5;
    map(goal_node) = 6;
    
    if (drawMapEveryTime)
        pause(0.5)
        image(1.5,1.5,map)
        grid on;
        axis image;
        drawnow;
    end
    
    frontierNode = frontierList.front();
    heuristicNode = heuristicList.front();
    if (frontierNode==goal_node)

            break;

    end
        
    map(frontierNode) = 3;

    
    [i j] = ind2sub(size(map), frontierNode);
    
    for n = 1 : 4 % each cell usually has 4 neighbors
            % Visit every neighbor of the Frontier_node node
            if n == 1 % Up
                row = i-1; col = j;
            elseif n == 2 % Left (Start with left before right)
                row  = i; col = j-1;
            elseif n == 3 % Right
                row = i; col  = j+1;
            else % Down
                row = i+1; col = j;
       
            end
         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Check if neighbor is inside the map
            if (row < 1 || row > nrows) % if row outside the map
                if n < 4 % if there are other neighbors to check
                    continue
                else    % n = 4 and all options are not valid so pop and break
                    frontierList.popfront()
                    heuristicList.popfront()
                    break
                end
    
            elseif (col < 1 || col > ncols) % if column outside the map
                if n < 4 % if there are other neighbors to check
                    continue
                else    % n = 4 and all options are not valid so pop and break
                    frontierList.popfront()
                    heuristicList.popfront()
                    break
                end
            end
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % skip if it’s an obstacle
            % for easy handling
            neighbor = sub2ind(size(map),row,col);
            
            if input_map(neighbor) % equivalent to input_map = true, so there is a wall
                if n < 4           % if there are other neighbors to check
                    continue
                else               % n = 4 an d all options are not valid so pop and break
                    frontierList.popfront()
                    heuristicList.popfront()
                    break
                end
            end
            
            
            if(any(Visited_Frontier == neighbor))
                if n < 4  % if there are other neighbors to check
                    continue
                else % n = 4 and all options are not valid so pop and break
                    frontierList.popfront()
                    break
                end
            end
            
            numExpanded = numExpanded + 1;
            
            heuristicNode_old = heuristicNode;
            heuristicNode = heuristic(neighbor);
           
            frontierList.pushtofront(neighbor);
            heuristicList.pushtofront(heuristic(neighbor)); 
            Visited_Frontier = [Visited_Frontier neighbor];
            
            if (heuristicNode > heuristicNode_old)
                frontierList.popfront()
                heuristicList.popfront()
            end
            
            
            

   % Update the map color
            if (map(neighbor) ~= 6) % if it's not the goal
                map(neighbor) = 4;   % mark neighbor with blue (to be explored)
            end
            
            % Nodes are expanded on the grid
            if (drawMapEveryTime)
                pause(0.5)
                image(1.5,1.5,map)
                grid on; % show grid
                axis image;
                drawnow;
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % We need to break once we push a new neighbor into the stack
            % Do not explore other neighbors
            break;           
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % We need to check if we can explore further or if we are done
        if (frontierList.empty()) % if queue is empty
            break;
        end
       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Construct route by following path from start to goal by following parents links
    if (frontierList.empty()) % if queue is empty
        route = [];
    else

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Construct final route
        route  = Visited_Frontier;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualize map and path
        for k = 2 : length(route) - 1
            map(route(k)) = 7;  %draw route in grey
            pause(0.5);
            image(1.5,1.5,map)
            grid on; % show grid
            axis image;
            drawnow;
        end







    end
end
end



   
       
       
       
    
    
    
    
    
    
    
    

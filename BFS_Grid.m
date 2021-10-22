% BFS function that is applied to input grid. 
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


function [route,numExpanded] = BFS_Grid(input_map, start_coords, goal_coords, drawMapEveryTime)

% Color map key for display (R G B):
 
cmap = [ 1 1 1; ... % White - 1
         0 0 0; ... % Black - 2
         1 0 0; ... % Red - 3
         0 0 1; ... % Blue - 4 
         1 1 0; ... % Yellow - 5
         0.5 0.5 0.5]; %Grey - 6

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

Visited_Frontier = start_node; %Init Frontier to start node


%Define frontier node using CQueue 

frontierQ = CQueue(start_node); 
   
%%%%%%%%%%%%%%%%%%%%%
%     Main Loop     %
%%%%%%%%%%%%%%%%%%%%%



 while true
        % Draw Frontier_node map
        map(start_node) = 5;    %start node
        map(goal_node)  = 6;   % goal node
        % make drawMapEveryTime = true if you want to see how the nodes are
        % expanded on the grid
        if (drawMapEveryTime)
            pause(0.5)  % pause to show the mapping steps
            image(1.5,1.5,map)
            grid on; % show grid
            axis image;
            drawnow;
        end

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        Frontier_node = frontierQ.front(); % get frontier node by using Queue front        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % If frontier node is goal, we're done
        if (Frontier_node==goal_node)
            break;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Update map by marking explored node in Frontier by red
        map(Frontier_node) = 3;
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Expand node by exploring children nodes (neighbors)
        
        % Compute row, column coordinates of Frontier_node node
        [i,j] = ind2sub(size(map),Frontier_node);
        
        % Visit each neighbor of the Frontier_node node and update the map,
        % stack. A neighbor is not valid if already explored or if it's a
        % wall. If current node cannot be expanded into new nodes, then pop
        % out from stack
        
        for n = 1 : 4 % each cell has usually 4 neighbors
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
                    frontierQ.pop()
                    break
                end
                       
                
            elseif (col < 1 || col > ncols) % if column outside the map
                if n < 4 % if there are other neighbors to check
                    continue
                else    % n = 4 and all options are not valid so pop and break
                    frontierQ.pop()
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
                    frontierQ.pop
                    break
                end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Check if not explored before

            % Check if neighbor is in Visited_Frontier
            % if yes, skip the neighbor
            if(any(Visited_Frontier == neighbor))
                if n < 4  % if there are other neighbors to check
                    continue
                else % n = 4 and all options are not valid so pop and break
                    frontierQ.pop
                    break
                end
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Well, the neighbor has passed all the tests and is a valid
            % candidate to be a frontier node
            
            % Update exploration number of steps
            numExpanded = numExpanded + 1;
            
            % Push the new node to frontier queue
            frontierQ.push(neighbor);
           
           
            Visited_Frontier = [Visited_Frontier neighbor];  % update visited frontier
            
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
        if (frontierQ.isempty()) % if queue is empty
            break;
        end
    end     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Construct route by following path from start to goal by following parents links
if (frontierQ.isempty()) % if queue is empty
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


    






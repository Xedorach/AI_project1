function [route,numExpanded] = DFS_Grid(input_map,start_coords,goal_coords,drawMapEveryTime)

% Run DFS algorithm on a grid

    % Inputs:

    %   input_map: a logical array where the freespace cells are false or 0

    %   and the obstacles are true or 1

    %   start_coords and goal_coords: coordinates of the start and goal cells

    %   respectively, the first entry is the row and the second is the column

    % Output:

    %   route: an array containing the linear indices of the cells along

    %   the shortest route found by DFS from start to goal or an empty

    %   array if there is no route. This is a single dimensional vector

    %   numExpanded: returns the total number of nodes expanded during

    %   search. Goal is not counted as an expanded node



    % Set up color map for display

    % 1 - white - clear cell

    % 2 - black - obstacle

    % 3 - red - explored

    % 4 - blue - in the frontier

    % 5 - green - start

    % 6 - yellow - goal

    %       R G B

    cmap = [1 1 1; ...   % white

            0 0 0; ...   % black

            1 0 0; ...   % red

            0 0 1; ...   % blue

            0 1 0; ...   % green

            1 1 0; ...   % yellow

            0.5 0.5 0.5];% gray



    colormap(cmap);



   [nrows,ncols] = size(input_map);



    % map - a table that keeps track of the status of each grid cell

    map = zeros(nrows,ncols);



    map(~input_map) = 1;     % For any element in input_map = 0, mark
                                %in white as an empty cell

    map(input_map)  = 2;     % Mark obstacle cells in black

        % Generate linear indices of start and goal nodes

    start_node  = sub2ind(size(map), start_coords(1), start_coords(2));

    goal_node   = sub2ind(size(map), goal_coords(1), goal_coords(2));



    map(start_node) = 5;    % mark start node with green

    map(goal_node) = 6;    % mark goal node with yellow



    % Keep track of number of nodes expanded

    numExpanded = 0;

    % Define visited frontier

    Visited_Frontier = start_node; % it includes initially the start node



    % Define the frontier stack. Use the CStack function by
    % downloading from:
    % https://www.mathworks.com/matlabcentral/fileexchange/28922-list-queue-stack

    stack = CStack(start_node); % includes initially the start node



    % Main Loop

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

        % Get the frontier node (top of the stack) without removing it

        Frontier_node = stack.top(); % get frontier node  - stack.top
        %only reads top of stack

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

                    stack.pop()

                    break

                end





            elseif (col < 1 || col > ncols) % if column outside the map

                if n < 4 % if there are other neighbors to check

                    continue

                else    % n = 4 and all options are not valid so pop and break

                    stack.pop()

                    break

                end

            end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % skip if itâ€™s an obstacle

            % for easy handling

            neighbor = sub2ind(size(map),row,col);

            if input_map(neighbor) % equivalent to input_map = true, so there is a wall

                if n < 4  % if there are other neighbors to check

                    continue

                else % n = 4 and all options are not valid so pop and break

                    stack.pop

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

                    stack.pop

                    break

                end

            end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Well, the neighbor has passed all the tests and is a valid

            % candidate to be a frontier node



            % Update exploration number of steps

            numExpanded = numExpanded + 1;



            % Push the new node to stack

            stack.push(neighbor);



            Visited_Frontier = [Visited_Frontier neighbor];  % update
                                                                    %visited frontier



            % Update the map color

            if (map(neighbor) ~= 6); % if it's not the goal

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

        if (stack.isempty()) % if stack is empty

            break;

        end

    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Construct route by following path from start to goal by following
%% parents links

if (stack.isempty()) % if stack is empty

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

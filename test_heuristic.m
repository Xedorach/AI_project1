%Get map dimensions

clc;close all

%Create a 10x5 map filled with zeros (empty cells)
map = false(10,5);

% Mark obstacles (fill the cells by setting them to 1)

start_coords = [7,4];
goal_coords = [3,2];

map(:,1) = true;
map(10,1:3) = true;
map(2:3,3) = true;
map(1,end) = true;

map_X = size(map,1);
map_Y = size(map,2);
goal_x = goal_coords(1);
goal_y = goal_coords(2);
   
maxMapIndex = map_X*map_Y;
   
for (n = 1:map_X)
    for(n2 = 1:map_Y)
       
        h = abs(n - goal_x) + abs(n2 - goal_y);
        heuristic(n,n2) = h;
    end
end

heuristic

% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(ranges,2);
for j = 1:N % for each time,
   angle =  scanAngles + pose(3,j);
      
    % Find grids hit by the rays (in the gird map coordinate)
     x_obstacle = ranges(: , j) .* cos( angle  ) + pose(1,j);
     y_obstacle = ranges(: , j) .* -sin( angle ) + pose(2,j);
     
     grid_robot = ceil(myResol * [pose(1,j) ; pose(2,j)]);
    % Find occupied-measurement cells and free-measurement cells
    M = size(x_obstacle,1);
    for k = 1:M
        grid_occ = ceil(myResol * [x_obstacle(k) ; y_obstacle(k)]);
        
        [freex, freey] = bresenham(grid_robot(1) , grid_robot(2), grid_occ(1), grid_occ(2));
    % Update the log-odds
        free = sub2ind(size(myMap),freey+myorigin(2),freex+myorigin(1));
        
        occ = sub2ind(size(myMap), grid_occ(2)+myorigin(2), grid_occ(1)+myorigin(1));
    % Saturate the log-odd values
    	myMap(free) = myMap(free) - lo_free;
        myMap(occ) = myMap(occ) + lo_occ;
    % Visualize the map as needed
       if(myMap(free) < lo_min)myMap(free) = lo_min;
         end
       if(myMap(occ) > lo_max)myMap(occ) = lo_max;
         end
    end
    
end


    
end


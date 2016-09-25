% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 

% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
% myPose(:,1) = param.init_pose;
myPose(:,1) =[ 0 ; 0 ; -4.7806];
         
   
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 200;                          % Please decide a reasonable number of M, 
Max_Particle = 1000;                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);% 3 *  M particles M
noise_sigma = diag([0.2, 0.2 ,1]);
noise_u = [0 0 0];
weight = 0.1 * ones(1,size(P,2));
for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
     if j < 20
         M = 100;
         noise_sigma = diag([0.1 0.1 0.035]);
     else
         M = 200 ;
         noise_sigma = diag([0.025 0.025 0.03]);
     end
    % 1) Propagate the particles 
%         P = P +  ( randn(size(P,2), 3) * noise_sigma )' ;
       
    % 2) Measurement Update 
    for i = 1:size(P,2)  
         P(:,i) = myPose(:,j-1) +  mvnrnd(noise_u,noise_sigma)';
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
         angle =  scanAngles + P(3,i);
         x_obstacle = ranges(: , j) .* cos( angle  ) + P(1,i);
         y_obstacle = ranges(: , j) .* -sin( angle ) + P(2,i);
         
         grid_particle = ceil(myResolution * [P(1,i) ; P(2,i)]);

         N_obstacle = size(x_obstacle,1);
         for k = 1:N_obstacle
             grid_occ = ceil(myResolution * [x_obstacle(k) ; y_obstacle(k)]);
             grid_occ_lim(2) = grid_occ(2)+myOrigin(2);
             grid_occ_lim(1) = grid_occ(1)+myOrigin(1);
             if grid_occ_lim(1)>0 && grid_occ_lim(1)<size(map,2)+1  &&   grid_occ_lim(2)>0 && grid_occ_lim(2)<size(map,1)+1  % 2 vs map_x ---1 va map_y
%                occ = sub2ind(size(map), grid_occ_lim(2), grid_occ_lim(1))
               weight(i) =  weight(i) + (map(grid_occ_lim(2),grid_occ_lim(1))> 0.5 )* 10; 
               weight(i) =  weight(i)  - (map(grid_occ_lim(2),grid_occ_lim(1))> 0.5 ) * 5;
             end

             [freex, freey] = bresenham(grid_particle(1) , grid_particle(2), grid_occ(1), grid_occ(2));
             freey_lim = freey+myOrigin(2);
             freex_lim = freex+myOrigin(1);
             del_index = freex_lim< 1 | freex_lim> size(map,2) | freey_lim<1 | freey_lim>size(map,1);
             
             freex_lim(del_index) = [];
             freey_lim(del_index) = [];
             free = sub2ind(size(map),freey_lim,freex_lim);
             
             weight(i) = weight(i) - sum((map(free)>0.5) * 5);
             weight(i) = weight(i) + sum((map(free)<0.5) * 1);  
         end
      end    
    %   2-3) Update the particle weights         
      %  weight = weight / sum(weight);
        [Min,Index_2] = min(weight);
        [Max,Index] = max(weight);
         Max
%         if Max >  5.00e+04
       
    %   2-4) Choose the best particle to update the pose
        myPose(:,j) = P(:,Index);
    % 3) Resample if the effective number of particles is smaller than a threshold
        threshold  = (Max + Min)/2;
        squence = find ( weight > threshold );
        Q = P( :,squence );
        num = ceil(M/size(Q,2));
        if num > Max_Particle
            num = Max_Particle;
        end
        P = repmat(Q , [1, num] ); 
%          size(P)
        weight = 0.1 * ones(1,size(P,2));
        
%         else
%             j = j - 1;
%         end
        
    % 4) Visualize the pose on the map as needed
   

end

end


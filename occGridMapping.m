% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;
pose(1:2,:) = pose(1:2,:);

Hpose(1:2,:) = ceil(myResol*pose(1:2,:))+myorigin;
N = size(pose,2);
for j = 1:N % for each time,
    
    
    % Important: coordinate systems such that y down and x to the right
    % so access by A(y,x) for origin at top left
    
    % Find real locations hit by the rays
    % Will be 1081 coordinates
    hit = [ranges(:,j).*cos(scanAngles+pose(3,j))...
          -ranges(:,j).*sin(scanAngles+pose(3,j))];
    hit = hit + pose(1:2,j)';
    
    empties = [500,500];
    size(hit,1);
    
    occup_pixels = ceil(myResol*hit)+myorigin';
    
    for i = 1:size(hit,1)
        
        if mod(i,3)==0
             
            [ex,ey] = bresenham(Hpose(1,j),Hpose(2,j),occup_pixels(i,1),occup_pixels(i,2));
            new_empties = [ex,ey];
            empties = [empties; new_empties];
        end
        
    end
    % Will speed things up by taking only unique empties
    
    empties = unique(empties,'rows');
    
    % Convert the real locations to pixel
    % Find occupied-measurement cells and free-measurement cells
    
    
    %empties = empties + pose(1:2,j)';
    %empties = max(min(empties,900),1);
    %empty_indices = ceil(myResol*empties)+myorigin';
    
    %occup_indices(1:5,:);
    %empty_indices(1:5,:);
    
    % Update the log-odds
    
    occ = sub2ind(size(myMap),occup_pixels(:,2),occup_pixels(:,1));
    free= sub2ind(size(myMap),empties(:,2),empties(:,1));
    
    myMap(occ) = myMap(occ) + lo_occ;
    myMap(free) = myMap(free) - lo_free;
    
    % Saturate the log-odd values
    
    myMap(occ) = max(min(myMap(occ),lo_max),lo_min);
    myMap(free) = max(min(myMap(free),lo_max),lo_min);
    
    % Visualize the map as needed

end


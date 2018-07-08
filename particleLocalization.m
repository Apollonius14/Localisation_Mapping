% Robotics: Estimation and Learning 

function myPose = particleLocalization(ranges, scanAngles, map, param)
%load practice.mat

% Number of poses to calculate
N = size(ranges, 2);

% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

% Map Parameters 
res = param.resol;
% Origin in pixels
orig = param.origin; 
% The initial pose is given
myPose(:,1) = param.init_pose;
%myPose(:,1) = [0;0;-4.78];
%Slice of LIDAR
slice = [1:4:1081];

% Decide the number of particles, Mpart
Mpart = 50;                        
% Initialised to be all at first pose with no uncertainity
Pstep = repmat(myPose(:,1), [1, Mpart]);
Psamp = zeros(3,Mpart);


% Show map
figure;
colormap('gray');
imshow(map);
hold on

%% Rudimentary mapping to occupied or not
wall = 1;
out = -1;
free = -1.01;
map(map>0.7*max(map))=wall;
map(map==0.49)=out;
map(map<0.49)=free;





%% Live Plot
x = [1,2,3];
y = [1,2,3];
lidar = plot(gca,x,y,'g-');

% Initialise LIDAR parameters
%Hits = zeros(Mpart,size(scanAngles,1),2);
Hits = zeros(Mpart,size(slice,2),2);
for j = 2:N
    neff = 0;
    j
    loop = 1;
    %pause(0.1);
    while neff < 0.4*Mpart
        
        loop = loop+1;
        scores = 1200*ones(1,Mpart);
        
        %Add odometry noise to last known particle
        x_n = 0.01;
        y_n = 0.01;
        t_n = 0.01;        
        for p=1:Mpart
            Psamp(:,p) = Pstep(:,p)+[randn*x_n;randn*y_n;randn*t_n];
        end
        
        %Guess a propagation
        dist = 0.025+randn*0.125;
        dirc = randn*min(0.5,randn*(loop/50));
        for p=1:Mpart
            Psamp(:,p) = Psamp(:,p) + ...
            [dist*cos(Pstep(3,p)+dirc);...
            -dist*sin(Pstep(3,p)+dirc);...
             dirc];
        end

        % Measure scores for each particle
        % coords is size Mpart x 2 x Lidar Angles
        
        for p=1:Mpart
 
            coords = [[ranges(slice,j).*cos(scanAngles(slice)+Psamp(3,p))]';...
                     -[ranges(slice,j).*sin(scanAngles(slice)+Psamp(3,p))]'];
                 
            coords = (coords + Psamp(1:2,p)).*res+orig;
            size(coords);
            Hits(p,:,:) = floor(coords');
        end
        
        for p=1:Mpart
            %for g=1:size(scanAngles,1)
            for g=1:size(slice,2)
            
            xco = min(Hits(p,g,1),size(map,2));
            yco = min(Hits(p,g,2),size(map,1));
        
            xco = max(xco,1);
            yco = max(yco,1);
                 
            scores(p) = scores(p)+(map(yco,xco));
            end
        end
        
        % Prune off map particles
        
       % for k=1:Mpart
       %    
       %    xcop = ceil(Psamp(2,k)*res+orig(1));
       %    ycop = ceil(Psamp(1,k)*res+orig(2));
       %   
       %   if map(ycop,xcop)==out
       %       scores(k) = 0;
       %   end
       % end
        
        % Measure neff
        tshow = sort(scores,'descend');
        tshow(1:10);
        neff = sum(scores(scores>1300));
        
    end
    
    % Take Newpose as weighted average of top 30% of particles
    
        top_ind = false(1,Mpart);
        top_ind(scores>0.99*max(scores)) = true;  
        faves = Psamp(:,top_ind);
        newweights=scores(:,top_ind)/(sum(scores(:,top_ind)));
        myPose(:,j)=newweights*faves';
        Pstep = repmat(myPose(:,j), [1, Mpart]);
 
       % Plot Lidar Scan of best pose
     %   coords = [[ranges(slice,j).*cos(scanAngles(slice)+myPose(3,j))]';...
     %             -[ranges(slice,j).*sin(scanAngles(slice)+myPose(3,j))]'];
     %       
     %       coords = floor((coords + myPose(1:2,j)).*res+orig);
     %       set(lidar,'xdata',coords(1,:),'ydata',coords(2,:));
     %      drawnow
     %      hold on
           plot(myPose(1,1:j-1)*res+param.origin(1),myPose(2,1:j-1)*res+param.origin(2), 'r.-');
     sprintf('%s',j)   
end

        
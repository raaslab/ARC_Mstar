%
%=========VisiLibity Demonstration Script=========
%
%This script uses the the MEX-files generated from
%visibility_polygon.cpp and in_environment.cpp.  Follow
%the instructions in the respective .cpp files to create
%these MEX-files before running this script. A graphical
%representation of the supplied environment file
%example1.environment is dislplayed and the user can
%then select points (must be in the environment) with
%the mouse and the visibility polygon of that point will
%be computed and plotted over the environment.
%


%Clear the desk


function []= plot_environment(Robot_path_x, Robot_path_y, Target_path_x,Target_path_y,Teammate,environment)
format long;

%Robustness constant
epsilon = 0.000000001;


%Snap distance (distance within which an observer location will be snapped to the
%boundary before the visibility polygon is computed)
snap_distance = 0.05;


%Read environment geometry from file
% sensor_x = [2,3,4,5,6,6,6,6,7,8,9,10,11,11,11,11,11,11];
% sensor_y =	[3,3,3,3,3,4,5,6,6,6,6,6,6,7,8,9,10,11];


% environment_sensor = environment_sensor(sensor_x,sensor_y,environment);
sensor_detect_indicator = [0 0 0 0];
Negative_reward = 50;

%Calculate a good plot window (bounding box) based on outer polygon of environment
environment_min_x = min(environment{1}(:,1));
environment_max_x = max(environment{1}(:,1));
environment_min_y = min(environment{1}(:,2));
environment_max_y = max(environment{1}(:,2));
X_MIN = environment_min_x-0.1*(environment_max_x-environment_min_x);
X_MAX = environment_max_x+0.1*(environment_max_x-environment_min_x);
Y_MIN = environment_min_y-0.1*(environment_max_y-environment_min_y);
Y_MAX = environment_max_y+0.1*(environment_max_y-environment_min_y);



%Clear plot and form window with desired properties
clf; hold on;
axis equal; axis off; axis([X_MIN X_MAX Y_MIN Y_MAX]);


%Plot Environment
patch( environment{1}(:,1) , environment{1}(:,2) , 0.1*ones(1,length(environment{1}(:,1)) ) , ...
    'w' , 'linewidth' , 1.5 );
for i = 2 : size(environment,2)
    patch( environment{i}(:,1) , environment{i}(:,2) , 0.1*ones(1,length(environment{i}(:,1)) ) , ...
        'k' , 'EdgeColor' , [0 0 0] , 'FaceColor' , [0.8 0.8 0.8] , 'linewidth' , 1.5 );
end




%Select test points and plot resulting visibility polygon

% current_x =[2,3,4,3,3,2,1,1,2,2,3,4,5,5,6,7,7,7]; %
% current_y =[14,14,14,14,13,13,13,12,12,13,13,13,13,14,14,14,15,16];

% current_x = [7 7 7 7 8 9 10 ]; %
% current_y = [8 7 6 5 5 5 5 ];
%
% sensor_x = [3  4  5  6  7  7  7];
% sensor_y = [12 12 12 12 12 11 10];
%
% Teamate_x = [10	10 10 10 10	10 10];
% Teamate_y = [10	10 10 10 10	10 10];

current_x = Robot_path_x; %
current_y = Robot_path_y;

sensor_x = Target_path_x;
sensor_y = Target_path_y;

Teamate_x = Teammate(1);
Teamate_y = Teammate(2);

Total_scan = false(1000,1000);
reward_step = 0;



%Acquire test point.
%[observer_x observer_y] = ginput(1);
observer_x = current_x(1);
observer_y = current_y(1);
%Make sure the current point is in the environment
if  in_environment( [observer_x observer_y] , environment , epsilon )
    
    %             Clear plot and form window with desired properties
    clf;  hold on;
    axis equal; axis off; axis([X_MIN X_MAX Y_MIN Y_MAX]);
    
    %Plot environment
    patch( environment{1}(:,1) , environment{1}(:,2) , 0.1*ones(1,length(environment{1}(:,1)) ) , ...
        'w' , 'linewidth' , 1.5 );
    for i = 2 : size(environment,2)
        patch( environment{i}(:,1) , environment{i}(:,2) , 0.1*ones(1,length(environment{i}(:,1)) ) , ...
            'k' , 'EdgeColor' , [0 0 0] , 'FaceColor' , [0.8 0.8 0.8] , 'linewidth' , 0.1 );
    end
    
    
    
    %             Plot observer
    plot3( observer_x , observer_y , 0.3 , ...
        'o' , 'Markersize' , 15 , 'MarkerEdgeColor' , 'k' , 'MarkerFaceColor' , 'r' );
    hold on
    
    
    W{1} = visibility_polygon( [sensor_x(1) sensor_y(1)] , environment , epsilon , snap_distance );
    V{1} = visibility_polygon( [observer_x observer_y] , environment , epsilon , snap_distance );
    
    
    %sensor polygon
    
    Area_sensor = polyarea(W{1}(:,1),W{1}(:,2));
    patch( W{1}(:,1) , W{1}(:,2) , 0.1*ones( size(W{1},1) , 1 ) , ...
        [0.7,0.7,0.9] , 'LineStyle' , 'none' );
    %         plot3( W{1}(:,1) , W{1}(:,2) , 0.1*ones( size(W{1},1) , 1 ) , ...
    %             'y*' , 'Markersize' , 5 );
    plot3( sensor_x(1) , sensor_y(1) , 0.3 , ...
        's' , 'Markersize' , 15, 'MarkerFaceColor' , [0.9,0.8,0.7],'MarkerFaceColor','b','MarkerEdgeColor','b' );
    
    plot3(Teamate_x,Teamate_y, 0.3 , ...
        'p' , 'Markersize' , 20, 'MarkerFaceColor' , [0.9,0.8,0.7],'MarkerFaceColor','r','MarkerEdgeColor','r' );
    %total polygon
    
    Total_visiable{1} =  V;
    
    for k = 1:1-1
        tpatch = patch( Total_visiable{k}{1}(:,1) , Total_visiable{k}{1}(:,2) , 0.1*ones( size(Total_visiable{k}{1},1) , 1 ) , ...
            [0.9,0.8,0.8] , 'LineStyle' , 'none' );
        alpha(tpatch,0.6)
    end
    
    %Compute and plot visibility polygon
    
    Area = polyarea(V{1}(:,1),V{1}(:,2));
    
    vpatch= patch( V{1}(:,1) , V{1}(:,2) , 0.1*ones( size(V{1},1) , 1 ) , ...
        [0.9,0.5,0.5],'LineStyle' , 'none' );
    %         plot3( V{1}(:,1) , V{1}(:,2) , 0.1*ones( size(V{1},1) , 1 ) , ...
    %             'b*' , 'Markersize' , 5 );
    alpha(vpatch, 0.6)
    
    
    hold on
    
    
    
    %%overlap area
    x1= V{1}(:,1);
    y1= V{1}(:,2);
    b1 = poly2mask(x1,y1,1000, 1000);
    areaImage = bwarea(b1);
    Total_scan = b1 | Total_scan;
    reward_step(1) = bwarea(Total_scan);
    
    
    
    
    plot3(sensor_x(1:1),sensor_y(1:1),0.1*ones( nnz(sensor_x(1:1)) , 1 ),'b','LineWidth',5)
    plot3(current_x(1:1),current_y(1:1),0.1*ones( nnz(current_x(1:1)) , 1 ),'r','LineWidth',5)
    pause(0.1)
    hold off
    
    %      mov(1) = getframe(gca);
    %      jj = 1;
    %      imwrite(mov(1),sprintf('High%d.jpg',jj))
    
    %sensor the next point
    
    
    
end
%
% movie2avi(mov, 'equilibrium.avi', 'compression','None', 'fps',10);
% winopen('equilibrium.avi')
% hold on
%  plot3(sensor_x,sensor_y,0.1*ones( nnz(sensor_x) , 1 ),'b','LineWidth',5)
%  plot3(current_x,current_y,0.1*ones( nnz(current_x) , 1 ),'r','LineWidth',5)


end

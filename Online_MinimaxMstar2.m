%Robustness constant
clear
epsilon = 0.000000001;


%Snap distance (distance within which an observer location will be snapped to the
%boundary before the visibility polygon is computed)
snap_distance = 0.05;


%Read environment geometry from file
environment = read_vertices_from_file('./Mstar.environment');


%Calculate a good plot window (bounding box) based on outer polygon of environment
environment_min_x = min(environment{1}(:,1));
environment_max_x = max(environment{1}(:,1));
environment_min_y = min(environment{1}(:,2));
environment_max_y = max(environment{1}(:,2));
X_MIN = environment_min_x-0.1*(environment_max_x-environment_min_x);
X_MAX = environment_max_x+0.1*(environment_max_x-environment_min_x);
Y_MIN = environment_min_y-0.1*(environment_max_y-environment_min_y);
Y_MAX = environment_max_y+0.1*(environment_max_y-environment_min_y);

Initial_Robot = [3;6];
% Initial_Target = [7;9];
% sensor_x = [6	6	6	6	6	6	6	6	6	6	6	6	6	7	8	8	8	8	8	7];
% sensor_y =	[1	2	3	4	5	6	7	8	9	10	11	12	13	13	13	14	15	16	17	17];
sensor_x =  [4];
sensor_y =	[8];
Teammate = [7,6];

Robot_detect = 0;
Teammate_detect = 0;
Record_Robot_path_x = Initial_Robot(1);
Record_Robot_path_y = Initial_Robot(2);

Initial_Target = [sensor_x(1) sensor_y(1)];
Record_Target_path_x = sensor_x(1);
Record_Target_path_y = sensor_y(1);


% %Clear plot and form window with desired properties

%Compute and plot visibility polygon
W{1} = visibility_polygon( [Initial_Target(1) Initial_Target(2)] , environment , epsilon , snap_distance );
Scaned_record_Target = {poly2mask(W{1}(:,1),W{1}(:,2),50, 50)};
Total_scan_Target = false(50,50);


%Compute and plot visibility polygon for the target
V{1} = visibility_polygon( [Initial_Robot(1) Initial_Robot(2)] , environment , epsilon , snap_distance );
Area = polyarea(V{1}(:,1),V{1}(:,2));
Reward(Initial_Robot(1),Initial_Robot(2)) = Area;

% patch( V{1}(:,1) , V{1}(:,2) , 0.1*ones( size(V{1},1) , 1 ) , ...
%     [0.9,0.6,0.6] , 'linewidth' , 1.5 );
% plot3( V{1}(:,1) , V{1}(:,2) , 0.1*ones( size(V{1},1) , 1 ) , ...
%     'b*' , 'Markersize' , 5 );
% hold off
Scaned_record = {poly2mask(V{1}(:,1),V{1}(:,2),50, 50)};
Total_scan_Robot = false(50,50);

plot_environment(Record_Robot_path_x, Record_Robot_path_y, Record_Target_path_x,Record_Target_path_y,Teammate,environment)

for oo = 1:6
    format long;
    
    Negative_reward = 1.0;
    Negtive_Teammate = 50.0;
    Vis = digraph([1],[]);
    Vis.Nodes.Robot_x= Record_Robot_path_x(oo);
    Vis.Nodes.Robot_y= Record_Robot_path_y(oo);
    Vis.Nodes.Target_x= Record_Target_path_x(oo);
    Vis.Nodes.Target_y= Record_Target_path_y(oo);
    %     Vis.Nodes.Target_x=Record_Target_path_x(oo);
    %     Vis.Nodes.Target_y=Record_Target_path_y(oo);
    
    Vis.Nodes.Generation = 1;
    Vis.Nodes.Parent = 0;
    bwarea(Total_scan_Robot)
    Vis.Nodes.Robot_Region{1} = Total_scan_Robot | Scaned_record{1,1} | poly2mask(V{1}(:,1),V{1}(:,2),50, 50);
    Vis.Nodes.Robot_Reward = bwarea(Vis.Nodes.Robot_Region{1});
    
    Total_scan_Target = Total_scan_Target |  poly2mask(W{1}(:,1),W{1}(:,2),50, 50);
    
    Vis.Nodes.Detection_time = 0;
    Vis.Nodes.Decision_Value = 0;
    Vis.Nodes.Decision_Node = 0;
    Vis.Nodes.Teammate_detected = 0;
    Vis.Nodes.path = 0;
    Vis.Nodes.QMAX = -1110;
    Vis.Nodes.QMIN = 1000;

    
    level = 6;
    Count = 1;

    for l = 1:level
        if l == 1
            start = 1;
            en = 1;
        else
            start = en +1;
            en = Count;
        end
        for j = start:en
            if mod(l,2)
                %% go right
                if in_environment( [Vis.Nodes.Robot_x(j)+1, Vis.Nodes.Robot_y(j)] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Robot_x(j)+0.3, Vis.Nodes.Robot_y(j)] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Robot_x(j)+0.6, Vis.Nodes.Robot_y(j)] , environment , epsilon )
                    [Vis] = RobotMove_Mstar(Vis,j,Count,1,0,environment);
                    Count = Count+1;
                end       
                
                %% %go left
                if in_environment( [Vis.Nodes.Robot_x(j)-1, Vis.Nodes.Robot_y(j)] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Robot_x(j)-0.3, Vis.Nodes.Robot_y(j)] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Robot_x(j)-0.6, Vis.Nodes.Robot_y(j)] , environment , epsilon )
                    
                    [Vis] = RobotMove_Mstar(Vis,j,Count,-1,0,environment);
                    Count = Count+1;
                end
                
                %% %go up   
                if in_environment( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)+1] , environment , epsilon )...
                        && in_environment( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)+0.3] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)+0.6] , environment , epsilon )
                    
                    [Vis] = RobotMove_Mstar(Vis,j,Count,0,1,environment);
                    Count = Count+1;                    
                end

                %% %go down
                if in_environment( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)-1] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)-0.3] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)-0.6] , environment , epsilon )
                    
                    [Vis] = RobotMove_Mstar(Vis,j,Count,0,-1,environment);
                    Count = Count+1;                   
                end

                %%
            else  %Opponent Moves
                %% Oppoent go right
                if in_environment( [Vis.Nodes.Target_x(j)+1, Vis.Nodes.Target_y(j)] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Target_x(j)+0.3, Vis.Nodes.Target_y(j)] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Target_x(j)+0.6, Vis.Nodes.Target_y(j)] , environment , epsilon )
                    
                    [Vis] = TargetMove_Mstar(Vis,j,Count,1,0,environment,Teammate);
                    Count = Count+1;     
                    
                    if Vis.Nodes.Generation(Count) == level + 1
                        Vis.Nodes.QMAX(Count) = Vis.Nodes.Robot_Reward(Count) - Vis.Nodes.Detection_time(Count)*Negative_reward - Vis.Nodes.Teammate_detected(Count)*Negtive_Teammate;
                        Vis.Nodes.QMIN(Count) = Vis.Nodes.Robot_Reward(Count) - Vis.Nodes.Detection_time(Count)*Negative_reward;
                    end
                end
             
                %% Oppoent go left
                
                if in_environment( [Vis.Nodes.Target_x(j)-1, Vis.Nodes.Target_y(j)] , environment , epsilon )...
                        && in_environment( [Vis.Nodes.Target_x(j)-0.3, Vis.Nodes.Target_y(j)] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Target_x(j)-0.6, Vis.Nodes.Target_y(j)] , environment , epsilon )
                    
                    [Vis] = TargetMove_Mstar(Vis,j,Count,-1,0,environment,Teammate);
                    Count = Count+1; 
                    
                    if Vis.Nodes.Generation(Count) == level + 1
                        Vis.Nodes.QMAX(Count) = Vis.Nodes.Robot_Reward(Count) - Vis.Nodes.Detection_time(Count)*Negative_reward - Vis.Nodes.Teammate_detected(Count)*Negtive_Teammate;
                        Vis.Nodes.QMIN(Count) = Vis.Nodes.Robot_Reward(Count) - Vis.Nodes.Detection_time(Count)*Negative_reward;
                    end
                end      

                %% Oppoent go up
                if in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)+1] , environment , epsilon )...
                        && in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)+0.3] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)+0.6] , environment , epsilon )
                    
                    [Vis] = TargetMove_Mstar(Vis,j,Count,0,1,environment,Teammate);
                    Count = Count+1;
                    
                    if Vis.Nodes.Generation(Count) == level + 1
                        Vis.Nodes.QMAX(Count) = Vis.Nodes.Robot_Reward(Count) - Vis.Nodes.Detection_time(Count)*Negative_reward -  Vis.Nodes.Teammate_detected(Count)*Negtive_Teammate;
                        Vis.Nodes.QMIN(Count) = Vis.Nodes.Robot_Reward(Count) - Vis.Nodes.Detection_time(Count)*Negative_reward ;
                    end
                end
                
                
                %% Oppoent go down

                if in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)-1] , environment , epsilon )...
                        && in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)-0.3] , environment , epsilon )...
                        &&in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)-0.6] , environment , epsilon )
                    
                    [Vis] = TargetMove_Mstar(Vis,j,Count,0,-1,environment,Teammate);
                    Count = Count+1;
                    
                    if Vis.Nodes.Generation(Count) == level + 1
                        Vis.Nodes.QMAX(Count) = Vis.Nodes.Robot_Reward(Count) - Vis.Nodes.Detection_time(Count)*Negative_reward -  Vis.Nodes.Teammate_detected(Count)*Negtive_Teammate;
                        Vis.Nodes.QMIN(Count) = Vis.Nodes.Robot_Reward(Count) - Vis.Nodes.Detection_time(Count)*Negative_reward;
                    end
                end
            end
        end
    end
%     PlotGragh(Vis);
    %find the optimal path
    
    T = nnz(Vis.Nodes.Generation);
    Node_path = 1;
    Best_node = 1;
    for i = T:-1:1
        if Vis.Nodes.Generation(i) >= level + 1
            continue
        else
            sucIDs = successors(Vis,i);
            BestMax = Vis.Nodes.QMAX(sucIDs(1));
            BestMin = Vis.Nodes.QMIN(sucIDs(1));
            if mod(Vis.Nodes.Generation(i),2)
                Vis.Nodes.path(i) = sucIDs(1);
                for k = 1:nnz(sucIDs)
                    if Vis.Nodes.QMAX(sucIDs(k)) > BestMax
                        BestMax = Vis.Nodes.QMAX(sucIDs(k));
                        Vis.Nodes.path(i) = sucIDs(k);
                    end
                    BestMin = min(BestMin,Vis.Nodes.QMIN(sucIDs(k))); 
                end
                Vis.Nodes.QMAX(i) = BestMax;
                Vis.Nodes.QMIN(i) = BestMin;
            else
                for k = 1:nnz(sucIDs)
                    if BestMin < Vis.Nodes.QMIN(sucIDs(k))
                        BestMin = Vis.Nodes.QMIN(sucIDs(k));
                        BestMax = Vis.Nodes.QMAX(sucIDs(k));
                    elseif BestMin == Vis.Nodes.QMIN(sucIDs(k))
                        BestMax = min(BestMax,Vis.Nodes.QMAX(sucIDs(k)));
                    end
                    Vis.Nodes.QMAX(i) = BestMax;
                    Vis.Nodes.QMIN(i) = BestMin;
                end
            end

            
        end
    end
%     PlotGragh(Vis);
    
    Robot_next = Vis.Nodes.path(1);
%     Target_next = Vis.Nodes.path(Robot_next);
    
    Record_Robot_path_x(oo+1) = Vis.Nodes.Robot_x(Robot_next);
    Record_Robot_path_y(oo+1) = Vis.Nodes.Robot_y(Robot_next);
    
    if Robot_detect == 0
        [Record_Target_path_x(oo+1), Record_Target_path_y(oo+1)] =  GreedyTargetMove(Record_Target_path_x(oo), Record_Target_path_y(oo),environment,Total_scan_Target);
    else
        [Record_Target_path_x(oo+1), Record_Target_path_y(oo+1)] =  TrackingMoveAfterDetected(Record_Target_path_x(oo),Record_Target_path_y(oo),Record_Robot_path_x(oo),Record_Robot_path_y(oo),environment);
    end

    %alha
%     if sset-beta > 0 || set2 < 5.1
%        [Record_Target_path_x(oo+1), Record_Target_path_y(oo+1)] = ...
%            Target_hurestic(Record_Target_path_x(oo), Record_Target_path_y(oo), Teammate(1), Teammate(2),environment);
%     else
%        [Record_Target_path_x(oo+1), Record_Target_path_y(oo+1)] = ...
%            Target_hurestic(Record_Target_path_x(oo), Record_Target_path_y(oo), Record_Robot_path_x(oo+1), Record_Robot_path_y(oo+1),environment);
%  
%     end

    %     plot_environment(Record_Robot_path_x(oo),Record_Robot_path_y(oo),Record_Target_path_x(oo),Record_Target_path_y(oo))
    W{1} = visibility_polygon( [Record_Target_path_x(oo+1) Record_Target_path_y(oo+1)] , environment , epsilon , snap_distance );
    V{1} = visibility_polygon( [Record_Robot_path_x(oo+1) Record_Robot_path_y(oo+1)] , environment , epsilon , snap_distance );
    if in_environment( [Record_Robot_path_x(oo+1) Record_Robot_path_y(oo+1)] , W , epsilon )
        Robot_detect = Robot_detect + 1;
    end
    
    if in_environment( [Teammate(1) Teammate(2)] , W , epsilon )
        Teammate_detect = Teammate_detect + 1;
    end
    
    Total_Robot_visiable{oo} =  V;
    Total_Target_visiable{oo} =  W;
    %Compute and plot visibility polygon
    
    Area = polyarea(V{1}(:,1),V{1}(:,2));
    x1= V{1}(:,1);
    y1= V{1}(:,2);
    b1 = poly2mask(x1,y1,50, 50);
    areaImage = bwarea(b1);
    Total_scan_Robot = b1 | Total_scan_Robot;
    reward_step(oo) = bwarea(Total_scan_Robot)
    True_reward(oo) = reward_step(oo)- Negative_reward*Robot_detect;
%     hold on
%     for k = 1:oo-1
%         Vpatch = patch( Total_Robot_visiable{k}{1}(:,1) , Total_Robot_visiable{k}{1}(:,2) , 0.1*ones( size(Total_Robot_visiable{k}{1},1) , 1 ) , ...
%             [0.9,0.8,0.8] , 'LineStyle' , 'none' );
%         alpha(Vpatch,0.6)
%     end
%     
%     for k = 1:oo-1
%         Wpatch = patch( Total_Target_visiable{k}{1}(:,1) , Total_Target_visiable{k}{1}(:,2) , 0.1*ones( size(Total_Target_visiable{k}{1},1) , 1 ) , ...
%             [0.7,0.7,0.9] , 'LineStyle' , 'none' );
%         alpha(Wpatch,0.6)
%     end
    plot_environment(Record_Robot_path_x(oo+1), Record_Robot_path_y(oo+1), Record_Target_path_x(oo+1),Record_Target_path_y(oo+1),Teammate,environment)
    


    
end








%Robustness constant
epsilon = 0.000000001;


%Snap distance (distance within which an observer location will be snapped to the
%boundary before the visibility polygon is computed)
snap_distance = 0.05;


%Read environment geometry from file
environment = read_vertices_from_file('./MCTS2.environment');


%Calculate a good plot window (bounding box) based on outer polygon of environment
environment_min_x = min(environment{1}(:,1));
environment_max_x = max(environment{1}(:,1));
environment_min_y = min(environment{1}(:,2));
environment_max_y = max(environment{1}(:,2));
X_MIN = environment_min_x-0.1*(environment_max_x-environment_min_x);
X_MAX = environment_max_x+0.1*(environment_max_x-environment_min_x);
Y_MIN = environment_min_y-0.1*(environment_max_y-environment_min_y);
Y_MAX = environment_max_y+0.1*(environment_max_y-environment_min_y);

Initial_Robot = [8;8];
% Initial_Target = [7;9];
% sensor_x = [6	6	6	6	6	6	6	6	6	6	6	6	6	7	8	8	8	8	8	7];
% sensor_y =	[1	2	3	4	5	6	7	8	9	10	11	12	13	13	13	14	15	16	17	17];
sensor_x =  [7];
sensor_y =	[9];
Teammate = [10,10];
Record_Robot_path_x = Initial_Robot(1);
Record_Robot_path_y = Initial_Robot(2);

Initial_Target = [sensor_x(1) sensor_y(1)];
Record_Target_path_x = sensor_x(1);
Record_Target_path_y = sensor_y(1);


% %Clear plot and form window with desired properties

%Compute and plot visibility polygon
W{1} = visibility_polygon( [Initial_Target(1) Initial_Target(2)] , environment , epsilon , snap_distance );



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
for oo = 1:50
    format long;
    
    Negtive_Reward = 50.0;
    Negtive_Teammate = 1000.0;
    Total_scan = false(1000,1000);
    Vis = digraph([1],[]);
    Vis.Nodes.Robot_x= Record_Robot_path_x(oo);
    Vis.Nodes.Robot_y= Record_Robot_path_y(oo);
    Vis.Nodes.Target_x= Record_Target_path_x(oo);
    Vis.Nodes.Target_y= Record_Target_path_y(oo);
    %     Vis.Nodes.Target_x=Record_Target_path_x(oo);
    %     Vis.Nodes.Target_y=Record_Target_path_y(oo);
    
    Vis.Nodes.Generation = 1;
    Vis.Nodes.Parent = 0;
    Vis.Nodes.Robot_Region{1} = Scaned_record{1,1} | poly2mask(V{1}(:,1),V{1}(:,2),50, 50);
    Vis.Nodes.Robot_Reward = bwarea(Vis.Nodes.Robot_Region{1});
    Vis.Nodes.Detection_time = 0;
    Vis.Nodes.Decision_Value = 0;
    Vis.Nodes.Decision_Node = 0;
    Vis.Nodes.Teammate_detected = 0;
    Vis.Nodes.path = 0;
    Vis.Nodes.Prune = 0;
    Vis.Nodes.QMAX = -1110;
    Vis.Nodes.QMIN = -1000;
    
    level = 4;
    Count = 1;
    for l = 1:level
        if l == 1
            start = 1;
            en = 1;
        else
            start = (4^(l-1)-1)/3 +1;
            en = (4^(l)-1)/3;
        end
        for j = start:en
            if mod(l,2)
                %%
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;
                Vis.Nodes.Parent(Count+1) = j;
                if Vis.Nodes.Prune(j) ~= -1
                    if in_environment( [Vis.Nodes.Robot_x(j)+1, Vis.Nodes.Target_y(j)] , environment , epsilon )...
                            &&in_environment( [Vis.Nodes.Robot_x(j)+0.5, Vis.Nodes.Target_y(j)] , environment , epsilon )
                        Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j)+1;
                        Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
                        Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                        Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                        Vis.Nodes.Decision_Value(Count+1) = 0;
                        
                        V{1} = visibility_polygon( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , environment , epsilon , snap_distance );
                        Vis.Nodes.Robot_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Robot_Region{j};
                        Vis.Nodes.Robot_Reward(Count+1) = bwarea(Vis.Nodes.Robot_Region{Count+1});
                        Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
                        Vis.Nodes.QMAX(Count+1) = -1110;
                        Vis.Nodes.QMIN(Count+1) = -1000;
                    else
                        Vis.Nodes.Prune(Count+1) = -1;
                    end
                else
                    Vis.Nodes.Prune(Count+1) = -1;
                end
                Count = Count+1;
                %%
                
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Parent(Count+1) =j;
                Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;
                if Vis.Nodes.Prune(j) ~= -1
                    if in_environment( [Vis.Nodes.Robot_x(j)-1, Vis.Nodes.Target_y(j)] , environment , epsilon )...
                            &&in_environment( [Vis.Nodes.Robot_x(j)-0.5, Vis.Nodes.Target_y(j)] , environment , epsilon )
                        Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j)-1;
                        Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
                        Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                        Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                        Vis.Nodes.Decision_Value(Count+1) = 0;
                        
                        V{1} = visibility_polygon( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , environment , epsilon , snap_distance );
                        Vis.Nodes.Robot_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Robot_Region{j};
                        Vis.Nodes.Robot_Reward(Count+1) = bwarea(Vis.Nodes.Robot_Region{Count+1});
                        Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
                        Vis.Nodes.QMAX(Count+1) = -110;
                        Vis.Nodes.QMIN(Count+1) = -1000;
                    else
                        Vis.Nodes.Prune(Count+1) = -1;
                    end
                    
                else
                    Vis.Nodes.Prune(Count+1) = -1;
                end
                Count = Count+1;
                %%
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Parent(Count+1) =j;
                Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;
                if Vis.Nodes.Prune(j) ~= -1
                    if in_environment( [Vis.Nodes.Robot_x(j), Vis.Nodes.Target_y(j)+1] , environment , epsilon )...
                            && in_environment( [Vis.Nodes.Robot_x(j), Vis.Nodes.Target_y(j)+0.5] , environment , epsilon )
                        Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
                        Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j)+1;
                        Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                        Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                        Vis.Nodes.Decision_Value(Count+1) = 0;
                        
                        V{1} = visibility_polygon( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , environment , epsilon , snap_distance );
                        Vis.Nodes.Robot_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Robot_Region{j};
                        Vis.Nodes.Robot_Reward(Count+1) = bwarea(Vis.Nodes.Robot_Region{Count+1});
                        Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
                        Vis.Nodes.QMAX(Count+1) = -110;
                        Vis.Nodes.QMIN(Count+1) = -1000;
                    else
                        Vis.Nodes.Prune(Count+1) = -1;
                    end
                    
                else
                    Vis.Nodes.Prune(Count+1) = -1;
                end
                Count = Count+1;
                %%
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Parent(Count+1) =j;
                Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;
                if Vis.Nodes.Prune(j) ~= -1
                    if in_environment( [Vis.Nodes.Robot_x(j), Vis.Nodes.Robot_y(j)-1] , environment , epsilon )
                        Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
                        Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j)-1;
                        Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                        Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                        Vis.Nodes.Decision_Value(Count+1) = 0;
                        
                        V{1} = visibility_polygon( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , environment , epsilon , snap_distance );
                        Vis.Nodes.Robot_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Robot_Region{j};
                        Vis.Nodes.Robot_Reward(Count+1) = bwarea(Vis.Nodes.Robot_Region{Count+1});
                        Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
                        Vis.Nodes.QMAX(Count+1) = -110;
                        Vis.Nodes.QMIN(Count+1) = -1000;
                    else
                        Vis.Nodes.Prune(Count+1) = -1;
                    end
                    
                else
                    Vis.Nodes.Prune(Count+1) = -1;
                end
                Count = Count+1;
                %%
            else
                %%
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Parent(Count+1) =j;
                Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;
                if Vis.Nodes.Prune(j) ~= -1
                    if in_environment( [Vis.Nodes.Target_x(j)+1, Vis.Nodes.Target_y(j)] , environment , epsilon )...
                            &&in_environment( [Vis.Nodes.Target_x(j)+0.5, Vis.Nodes.Target_y(j)] , environment , epsilon )
                        Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
                        Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
                        Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j)+1;
                        Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                        
                        V{1} = visibility_polygon( [Vis.Nodes.Target_x(Count+1) Vis.Nodes.Target_y(Count+1)] , environment , epsilon , snap_distance );
                        Vis.Nodes.Robot_Region{Count+1} = Vis.Nodes.Robot_Region{j} ;
                        Vis.Nodes.Robot_Reward(Count+1) = Vis.Nodes.Robot_Reward(j);
                        Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
                        Vis.Nodes.QMAX(Count+1) = -110;
                        Vis.Nodes.QMIN(Count+1) = -1000;
                        
                        
                        if in_environment( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , V , epsilon )
                            Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j)+1;
                        end
                        
                        if in_environment( [Teammate(1) Teammate(2)] , V , epsilon )
                            Vis.Nodes.Teammate_detected(Count+1) = Vis.Nodes.Teammate_detected(j)+1;
                        end
                        
                    else
                        Vis.Nodes.Prune(Count+1) = -1;
                    end
                    
                else
                    Vis.Nodes.Prune(Count+1) = -1;
                end
                if Vis.Nodes.Generation(Count+1) == level + 1
                    Vis.Nodes.QMAX(Count+1) = Vis.Nodes.Robot_Reward(Count+1) - Vis.Nodes.Detection_time(Count+1)*Negative_reward;
                    Vis.Nodes.QMIN(Count+1) = Vis.Nodes.Robot_Reward(Count+1) - Vis.Nodes.Detection_time(Count+1)*Negative_reward - Vis.Nodes.Teammate_detected(Count+1)*Negtive_Teammate;
                end
                Count = Count+1;
                %%
                
                
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Parent(Count+1) =j;
                Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;
                if Vis.Nodes.Prune(j) ~= -1
                    if in_environment( [Vis.Nodes.Target_x(j)-1, Vis.Nodes.Target_y(j)] , environment , epsilon )...
                            && in_environment( [Vis.Nodes.Target_x(j)-0.5, Vis.Nodes.Target_y(j)] , environment , epsilon )
                        Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
                        Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
                        Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j)-1;
                        Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
                        
                        V{1} = visibility_polygon( [Vis.Nodes.Target_x(Count+1) Vis.Nodes.Target_y(Count+1)] , environment , epsilon , snap_distance );
                        Vis.Nodes.Robot_Region{Count+1} = Vis.Nodes.Robot_Region{j} ;
                        Vis.Nodes.Robot_Reward(Count+1) = Vis.Nodes.Robot_Reward(j);
                        Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
                        Vis.Nodes.QMAX(Count+1) = -110;
                        Vis.Nodes.QMIN(Count+1) = -1000;
                        if in_environment( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , V , epsilon )
                            Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j)+1;
                        end
                        
                        if in_environment( [Teammate(1) Teammate(2)] , V , epsilon )
                            Vis.Nodes.Teammate_detected(Count+1) = Vis.Nodes.Teammate_detected(j)+1;
                        end
                        
                    else
                        Vis.Nodes.Prune(Count+1) = -1;
                    end
                    
                else
                    Vis.Nodes.Prune(Count+1) = -1;
                end
                
                if Vis.Nodes.Generation(Count+1) == level + 1
                    Vis.Nodes.QMAX(Count+1) = Vis.Nodes.Robot_Reward(Count+1) - Vis.Nodes.Detection_time(Count+1)*Negative_reward;
                    Vis.Nodes.QMIN(Count+1) = Vis.Nodes.Robot_Reward(Count+1) - Vis.Nodes.Detection_time(Count+1)*Negative_reward - Vis.Nodes.Teammate_detected(Count+1)*Negtive_Teammate;
                end
                
                Count = Count+1;
                %%
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Parent(Count+1) =j;
                Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;
                if Vis.Nodes.Prune(j) ~= -1
                   if in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)+1] , environment , epsilon )...
                            && in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)+0.5] , environment , epsilon )
                        Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
                        Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
                        Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                        Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j)+1;
                        
                        V{1} = visibility_polygon( [Vis.Nodes.Target_x(Count+1) Vis.Nodes.Target_y(Count+1)] , environment , epsilon , snap_distance );
                        Vis.Nodes.Robot_Region{Count+1} = Vis.Nodes.Robot_Region{j} ;
                        Vis.Nodes.Robot_Reward(Count+1) = Vis.Nodes.Robot_Reward(j);
                        Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
                        Vis.Nodes.QMAX(Count+1) = -110;
                        Vis.Nodes.QMIN(Count+1) = -1000;
                        if in_environment( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , V , epsilon )
                            Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j)+1;
                        end
                        
                        if in_environment( [Teammate(1) Teammate(2)] , V , epsilon )
                            Vis.Nodes.Teammate_detected(Count+1) = Vis.Nodes.Teammate_detected(j)+1;
                        end
                        
                    else
                        Vis.Nodes.Prune(Count+1) = -1;
                    end
                    
                else
                    Vis.Nodes.Prune(Count+1) = -1;
                end
                if Vis.Nodes.Generation(Count+1) == level + 1
                    Vis.Nodes.QMAX(Count+1) = Vis.Nodes.Robot_Reward(Count+1) - Vis.Nodes.Detection_time(Count+1)*Negative_reward - Vis.Nodes.Teammate_detected(Count+1)*Negtive_Teammate;
                    Vis.Nodes.QMIN(Count+1) = Vis.Nodes.Robot_Reward(Count+1) - Vis.Nodes.Detection_time(Count+1)*Negative_reward ;
                end
                Count = Count+1;
                %%
                Vis=addedge(Vis,j,Count+1);
                Vis.Nodes.Parent(Count+1) =j;
                Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;
                if Vis.Nodes.Prune(j) ~= -1
                    if in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)-1] , environment , epsilon )...
                            && in_environment( [Vis.Nodes.Target_x(j), Vis.Nodes.Target_y(j)-0.5] , environment , epsilon )
                        Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
                        Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
                        Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
                        Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j)-1;
                        
                        V{1} = visibility_polygon( [Vis.Nodes.Target_x(Count+1) Vis.Nodes.Target_y(Count+1)] , environment , epsilon , snap_distance );
                        Vis.Nodes.Robot_Region{Count+1} = Vis.Nodes.Robot_Region{j} ;
                        Vis.Nodes.Robot_Reward(Count+1) = Vis.Nodes.Robot_Reward(j);
                        Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
                        Vis.Nodes.QMAX(Count+1) = -110;
                        Vis.Nodes.QMIN(Count+1) = -1000;
                        if in_environment( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , V , epsilon )
                            Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j)+1;
                        end
                        
                        if in_environment( [Teammate(1) Teammate(2)] , V , epsilon )
                            Vis.Nodes.Teammate_detected(Count+1) = Vis.Nodes.Teammate_detected(j)+1;
                        end
                        
                    else
                        Vis.Nodes.Prune(Count+1) = -1;
                    end
                    
                else
                    Vis.Nodes.Prune(Count+1) = -1;
                end
                if Vis.Nodes.Generation(Count+1) == level + 1
                    Vis.Nodes.QMAX(Count+1) = Vis.Nodes.Robot_Reward(Count+1) - Vis.Nodes.Detection_time(Count+1)*Negative_reward - Vis.Nodes.Teammate_detected(Count+1)*Negtive_Teammate;
                    Vis.Nodes.QMIN(Count+1) = Vis.Nodes.Robot_Reward(Count+1) - Vis.Nodes.Detection_time(Count+1)*Negative_reward;
                end
                Count = Count+1;
                %%
                
            end
            
            %%
            
        end
    end
    
    %find the optimal path
    
    T = nnz(Vis.Nodes.Generation);
    Node_path = 1;
    Best_node = 1;
    for i = T:-1:1
        if Vis.Nodes.Generation(i) >= level + 1
            continue
        else
            sucIDs = successors(Vis,i);
            for k = 1:nnz(sucIDs)
                if mod(Vis.Nodes.Generation(i),2)&& Vis.Nodes.Prune(sucIDs(k)) ~= -1
                    if Vis.Nodes.QMAX(i) < Vis.Nodes.QMAX(sucIDs(k))
                        Vis.Nodes.QMAX(i) = min(Vis.Nodes.QMAX(sucIDs(k)),Vis.Nodes.QMAX(i));
                        Vis.Nodes.QMIN(i) = max(Vis.Nodes.QMIN(sucIDs(k)),Vis.Nodes.QMIN(i));
                        Vis.Nodes.path(i) = sucIDs(k);
                    end
                else
                    if Vis.Nodes.QMIN(i) >= Vis.Nodes.QMIN(sucIDs(k)) && Vis.Nodes.Prune(sucIDs(k)) ~= -1
                        Vis.Nodes.QMIN(i) = Vis.Nodes.QMIN(sucIDs(k));
                        Vis.Nodes.QMAX(i) = Vis.Nodes.QMAX(sucIDs(k));
                        Vis.Nodes.path(i) = sucIDs(k);
                    end
                end
            end
            
        end
    end
    Robot_next = Vis.Nodes.path(1);
    Target_next = Vis.Nodes.path(Robot_next);
    
    Record_Robot_path_x(oo+1) = Vis.Nodes.Robot_x(Robot_next);
    Record_Robot_path_y(oo+1) = Vis.Nodes.Robot_y(Robot_next);
    Record_Target_path_x(oo+1) = Vis.Nodes.Target_x(Target_next);
    Record_Target_path_y(oo+1) = Vis.Nodes.Target_y(Target_next);
    plot_environment(Record_Robot_path_x(oo),Record_Robot_path_y(oo),Record_Target_path_x(oo),Record_Target_path_y(oo))
    
    
    a = 1;
end








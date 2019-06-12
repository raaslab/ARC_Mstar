function [Vis] = TargetMove_Mstar(Vis,j,Count,x,y,environment,Teammate)
Vis=addedge(Vis,j,Count+1);
Vis.Nodes.Parent(Count+1) =j;
Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;

Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j);
Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j);
Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j) + x;
Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j) + y;

V{1} = visibility_polygon( [Vis.Nodes.Target_x(Count+1) Vis.Nodes.Target_y(Count+1)] , environment , 0.000000001 , 0.05 );
Vis.Nodes.Robot_Region{Count+1} = Vis.Nodes.Robot_Region{j} ;
Vis.Nodes.Robot_Reward(Count+1) = Vis.Nodes.Robot_Reward(j);
Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
Vis.Nodes.QMAX(Count+1) = -1000;
Vis.Nodes.QMIN(Count+1) = 1000;


if in_environment( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , V ,  0.000000001 )
    Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j)+1;
end

if in_environment( [Teammate(1) Teammate(2)] , V , 0.000000001 )
    Vis.Nodes.Teammate_detected(Count+1) = Vis.Nodes.Teammate_detected(j)+1;
end
end
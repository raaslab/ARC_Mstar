function [Vis] = RobotMove_Mstar(Vis,j,Count,x,y,environment)

Vis=addedge(Vis,j,Count+1);
Vis.Nodes.Generation(Count+1) = Vis.Nodes.Generation(j)+1;
Vis.Nodes.Parent(Count+1) = j;

Vis.Nodes.Robot_x(Count+1) = Vis.Nodes.Robot_x(j) + x;
Vis.Nodes.Robot_y(Count+1) = Vis.Nodes.Robot_y(j) + y;
Vis.Nodes.Target_x(Count+1) = Vis.Nodes.Target_x(j);
Vis.Nodes.Target_y(Count+1) = Vis.Nodes.Target_y(j);
Vis.Nodes.Decision_Value(Count+1) = 0;

V{1} = visibility_polygon( [Vis.Nodes.Robot_x(Count+1) Vis.Nodes.Robot_y(Count+1)] , environment , 0.000000001 , 0.05 );
Vis.Nodes.Robot_Region{Count+1} = poly2mask(V{1}(:,1),V{1}(:,2),50, 50) | Vis.Nodes.Robot_Region{j};
Vis.Nodes.Robot_Reward(Count+1) = bwarea(Vis.Nodes.Robot_Region{Count+1});
Vis.Nodes.Detection_time(Count+1) = Vis.Nodes.Detection_time(j);
Vis.Nodes.QMAX(Count+1) = -1000;
Vis.Nodes.QMIN(Count+1) = 1000;

end
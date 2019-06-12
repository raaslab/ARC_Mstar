function [] = PlotGragh(G)
Label = {};
l2 =25; l3 = 7; l4=1; l5=1;
for i = 1: nnz(G.Nodes.Generation)
   Label{i} = ['(',num2str(G.Nodes.QMAX(i)),',',num2str(G.Nodes.QMIN(i)),')'];

   if G.Nodes.Generation(i) == 5
                  X(i) = l5;
                  Y(i) = 2;
                  l5=l5+1.5;
    elseif G.Nodes.Generation(i) == 4
                  X(i) = l4;
                  Y(i) = 3;
                  l4=l4+3.5;
            
    elseif G.Nodes.Generation(i) == 3
                  X(i) = l3;
                  Y(i) = 4;
                  l3=l3+14;
                  
    elseif G.Nodes.Generation(i) == 2
                  X(i) = l2;
                  Y(i) = 5;
                  l2=l2+23;
                  
    elseif  G.Nodes.Generation(i) == 1
             X(i) = 50;
             Y(i) = 6;
    
   end
end
H=plot(G,'XData',X,'YData',Y);
H.NodeLabel = Label;


end
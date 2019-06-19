function [Next_x, Next_y] =  GreedyTargetMove(x,y,environment,Total_scan_Target)
Record_scan = Total_scan_Target;

%% go right
if in_environment( [x+1, y] , environment , 0.000000001 )...
        &&in_environment( [x+0.3, y] , environment , 0.000000001 )...
        &&in_environment( [x+0.6, y] , environment , 0.000000001 )

    W{1} = visibility_polygon( [x+1 y] , environment , 0.000000001 ,  0.05 );
    if bwarea(Total_scan_Target |  poly2mask(W{1}(:,1),W{1}(:,2),50, 50)) >= bwarea(Record_scan)
        Next_x = x + 1;
        Next_y = y;
        Record_scan = Total_scan_Target | poly2mask(W{1}(:,1),W{1}(:,2),50, 50);
    end
end

%% go right
if in_environment( [x-1, y] , environment , 0.000000001 )...
        &&in_environment( [x-0.3, y] , environment , 0.000000001 )...
        &&in_environment( [x-0.6, y] , environment , 0.000000001 )

    W{1} = visibility_polygon( [x-1 y] , environment , 0.000000001 ,  0.05 );
    if bwarea(Total_scan_Target |  poly2mask(W{1}(:,1),W{1}(:,2),50, 50)) >= bwarea(Record_scan)
        Next_x = x - 1;
        Next_y = y;
        Record_scan = Total_scan_Target | poly2mask(W{1}(:,1),W{1}(:,2),50, 50);
    end
end

%% go up
if in_environment( [x, y+1] , environment , 0.000000001 )...
        &&in_environment( [x, y+0.3] , environment , 0.000000001 )...
        &&in_environment( [x, y+0.6] , environment , 0.000000001 )

    W{1} = visibility_polygon( [x y+1] , environment , 0.000000001 ,  0.05 );
    if bwarea(Total_scan_Target |  poly2mask(W{1}(:,1),W{1}(:,2),50, 50)) >= bwarea(Record_scan)
        Next_x = x;
        Next_y = y+1;
        Record_scan = Total_scan_Target | poly2mask(W{1}(:,1),W{1}(:,2),50, 50);
    end
end

%% go right
if in_environment( [x, y-1] , environment , 0.000000001 )...
        &&in_environment( [x, y-0.3] , environment , 0.000000001 )...
        &&in_environment( [x, y-0.6] , environment , 0.000000001 )

    W{1} = visibility_polygon( [x y-1] , environment , 0.000000001 ,  0.05 );
    if bwarea(Total_scan_Target |  poly2mask(W{1}(:,1),W{1}(:,2),50, 50)) >= bwarea(Record_scan)
        Next_x = x;
        Next_y = y-1;
        Record_scan = Total_scan_Target | poly2mask(W{1}(:,1),W{1}(:,2),50, 50);
    end
end

end
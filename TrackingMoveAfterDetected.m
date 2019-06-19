function [Next_x, Next_y] =  TrackingMoveAfterDetected(x_target,y_target,x_robot,y_robot,environment)
x = x_target;
y = y_target;
distance = norm([x_target,y_target]-[x_robot,y_robot],2);

%% go right
if in_environment( [x+1, y] , environment , 0.000000001 )...
        &&in_environment( [x+0.3, y] , environment , 0.000000001 )...
        &&in_environment( [x+0.6, y] , environment , 0.000000001 )
    
    if norm([x_target+1,y_target]-[x_robot,y_robot],2) <= distance
        distance = norm([x_target+1,y_target]-[x_robot,y_robot],2);
        Next_x = x + 1;
        Next_y = y;
    end
end

%% go right
if in_environment( [x-1, y] , environment , 0.000000001 )...
        &&in_environment( [x-0.3, y] , environment , 0.000000001 )...
        &&in_environment( [x-0.6, y] , environment , 0.000000001 )

    if norm([x_target-1,y_target]-[x_robot,y_robot],2) <= distance
        distance = norm([x_target-1,y_target]-[x_robot,y_robot],2);
        Next_x = x - 1;
        Next_y = y;
    end
end

%% go up
if in_environment( [x, y+1] , environment , 0.000000001 )...
        &&in_environment( [x, y+0.3] , environment , 0.000000001 )...
        &&in_environment( [x, y+0.6] , environment , 0.000000001 )

    if norm([x_target,y_target+1]-[x_robot,y_robot],2) <= distance
        distance = norm([x_target,y_target+1]-[x_robot,y_robot],2);
        Next_x = x;
        Next_y = y+1;
    end
end

%% go right
if in_environment( [x, y-1] , environment , 0.000000001 )...
        &&in_environment( [x, y-0.3] , environment , 0.000000001 )...
        &&in_environment( [x, y-0.6] , environment , 0.000000001 )

    if norm([x_target,y_target-1]-[x_robot,y_robot],2) <= distance
        distance = norm([x_target,y_target-1]-[x_robot,y_robot],2);
        Next_x = x;
        Next_y = y-1;
    end
end


end
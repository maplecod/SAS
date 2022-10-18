
function [path,iteration_times,current_direction] = AS(map,origin,desitination)
iteration_times = 0;
%% 状态变量初始化
%包含： 起点，终点
%       已完成，未完成
%       障碍
Obstacle = 2;
Origin = 3;
Destination = 4;
Finished = 5;
Unfinished = 6;
Path = 7;

%% 
%初始化数据
map_size = size(map);                           %得到地图得大小
%节点总代价值    原点为0，其余为无穷大
node_g_cost = Inf(map_size(1),map_size(2));
node_g_cost(origin(1),origin(2)) = 0;
%启发函数
node_f_cost = Inf(map_size(1),map_size(2));
node_f_cost(origin(1),origin(2)) = fdis(origin,desitination,1,2);
%节点父节点     所有父节点为0
node_parent = zeros(map_size(1),map_size(2));
%运行方向
current_direction = zeros(map_size(1),map_size(2));
current_direction(origin(1),origin(2)) = 2;
%终点索引
destination_index = sub2ind(map_size,desitination(1),desitination(2));
%是否成功
plan_succeeded = false;

%% 
while 1
    iteration_times = iteration_times+1;
    [min_node_cost,current_node_index] = min(node_g_cost(:));
    if(min_node_cost == inf || current_node_index == destination_index)
        plan_succeeded = true;
        break;
    end
    %当前点设置为无穷大，并在标记中标记为已完成。
    node_g_cost(current_node_index) = inf;
    map(current_node_index) = Finished;
    %获得当前节点的索引。
    [x,y] = ind2sub(map_size,current_node_index);
    for k = 1:8   
        if(k==1)
            adjacent_node = [x-1,y];
        elseif(k==2)
            adjacent_node = [x+1,y];
        elseif(k==3)
            adjacent_node = [x,y-1];
        elseif(k==4)
            adjacent_node = [x,y+1];
        elseif(k==5)
            adjacent_node = [x+1,y+1];
        elseif(k==6)
            adjacent_node = [x+1,y-1];
        elseif(k==7)
            adjacent_node = [x-1,y+1];
        elseif(k==8)
            adjacent_node = [x-1,y-1];
        end
        %判断点是否在地图内部
        if((adjacent_node(1) > 0 && adjacent_node(1) <= map_size(1)) && (adjacent_node(2) > 0 && adjacent_node(2) <= map_size(2))) % make sure the adjacent_node don't exceeds the map
            current_direction(adjacent_node(1),adjacent_node(2)) = k;
            %判断地图周围是否是障碍和已经结束，end结束
            if(map(adjacent_node(1),adjacent_node(2)) ~= Obstacle && map(adjacent_node(1),adjacent_node(2)) ~= Finished)                
                if(node_g_cost(adjacent_node(1),adjacent_node(2)) > min_node_cost + 1)
                    if(k <= 4)
                        node_g_cost(adjacent_node(1),adjacent_node(2)) = min_node_cost + 1;
                    else
                        node_g_cost(adjacent_node(1),adjacent_node(2)) = min_node_cost + sqrt(2);
                    end
                    node_f_cost(adjacent_node(1),adjacent_node(2)) = node_g_cost(adjacent_node(1),adjacent_node(2)) + fdis(origin, desitination, 1,2);
                   %做路径平滑
                    if(current_direction(x,y) ~= current_direction(adjacent_node(1),adjacent_node(2)))
                        node_f_cost(adjacent_node(1),adjacent_node(2)) = node_f_cost(adjacent_node(1),adjacent_node(2)) + 2;
                    end
%                     current_direction(adjacent_node(1),adjacent_node(2)) = current_direction(x,y);
                    
                    %判断地图周围点是不是起点，是：该点父节点为0，不是：父节点为当前点
                    if(map(adjacent_node(1),adjacent_node(2)) == Origin)
                        node_parent(adjacent_node(1),adjacent_node(2)) = 0;
                    else
                        node_parent(adjacent_node(1),adjacent_node(2)) = current_node_index;
                    end
                    %一轮判断结束
                    %地图上周围点标记为未完成
                    map(adjacent_node(1),adjacent_node(2)) = Unfinished;
                end
            end
        end
    end
end
%%    
    if(plan_succeeded == true)
        path = [];
        node = destination_index;
        while(node_parent(node) ~= 0)
            path = [node_parent(node),path];
            node = node_parent(node);
        end
    else
        path = [];
    end


end
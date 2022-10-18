function [path,iteration_times] = SAS(map,origin,desitination,flystep)
iteration_times = 0;
%% 状态变量初始化
%包含： 起点，终点
%       已完成，未完成
%       障碍
Obstacle = 2;
Origin = 3;
Destination = 4;
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
parent_direction = zeros(map_size(1),map_size(2));
%终点索引
destination_index = sub2ind(map_size,desitination(1),desitination(2));
%是否成功
plan_succeeded = false;

%% 
while 1
    iteration_times = iteration_times+1;
    [min_node_cost,current_node_index] = min(node_f_cost(:));
    if(min_node_cost == inf || current_node_index == destination_index)
        plan_succeeded = true;
        break;
    end
    %当前点设置为无穷大，并在标记中标记为已完成。
    node_f_cost(current_node_index) = inf;
    %获得当前节点的索引。
    [x,y] = ind2sub(map_size,current_node_index);
    if(iteration_times == 1)
        for k = 0:11   
            if(k == 0)
                parent_direction(x,y) = 0;
            elseif(k == 1)
                parent_direction(x,y) = pi/6;
            elseif(k == 2)
                parent_direction(x,y) = -pi/6;
            elseif(k == 3)
                parent_direction(x,y) = pi/3;
            elseif(k == 4)
                parent_direction(x,y) = -pi/3;
            elseif(k == 5)
                parent_direction(x,y) = pi/2;
            elseif(k == 6)
                parent_direction(x,y) = -pi/2;
            elseif(k == 7)
                parent_direction(x,y) = pi*2/3;
            elseif(k == 8)
                parent_direction(x,y) = -pi*2/3;
            elseif(k == 9)
                parent_direction(x,y) = pi*5/6;
            elseif(k == 10)
                parent_direction(x,y) = -pi*5/6;
            elseif(k == 11)
                parent_direction(x,y) = pi;
            end
            adjacent_node = [round(x + cos(parent_direction(x,y)) * flystep),round(y + sin(parent_direction(x,y)) * flystep)];
            %判断点是否在地图内部
            if((adjacent_node(1) > 0 && adjacent_node(1) <= map_size(1)) && (adjacent_node(2) > 0 && adjacent_node(2) <= map_size(2))) % make sure the adjacent_node don't exceeds the map
                %判断地图周围是否是障碍和已经结束，end结束
                if(map(adjacent_node(1),adjacent_node(2)) ~= Obstacle)                
                    node_g_cost(adjacent_node(1),adjacent_node(2)) = min_node_cost + 1;
                    node_f_cost(adjacent_node(1),adjacent_node(2)) = node_g_cost(adjacent_node(1),adjacent_node(2)) + fdis(origin, desitination, 1,2);
                    %存储当前扩展点得方向 方向值为父节点得方向
                    current_direction(adjacent_node(1),adjacent_node(2)) = parent_direction(x,y);
                    %判断地图周围点是不是起点，是：该点父节点为0，不是：父节点为当前点
                    if(map(adjacent_node(1),adjacent_node(2)) == Origin)
                        node_parent(adjacent_node(1),adjacent_node(2)) = 0;
                    else
                        node_parent(adjacent_node(1),adjacent_node(2)) = current_node_index;
                    end
                    %一轮判断结束
                end
            end
        end
    else
        %% 问题每次角度叠加都是在上一个角度上修改，而不是在初始角度上修改。
        for n = 1:5   
            if(n == 1)
                current_direction(x,y) = current_direction(x,y) + 0;
            elseif(n == 2)
                current_direction(x,y) = current_direction(x,y) + pi/6;
            elseif(n == 3)
                current_direction(x,y) = current_direction(x,y) - 2*pi/6;
            elseif(n == 4)
                current_direction(x,y) = current_direction(x,y) + pi/6 + pi/3;
            elseif(n == 5)
                current_direction(x,y) = current_direction(x,y) - 2*pi/3;
            end
            adjacent_node = [round(x + cos(current_direction(x,y)) * flystep),round(y + sin(current_direction(x,y)) * flystep)];
            %判断点是否在地图内部
            if((adjacent_node(1) > 0 && adjacent_node(1) <= map_size(1)) && (adjacent_node(2) > 0 && adjacent_node(2) <= map_size(2))) % make sure the adjacent_node don't exceeds the map
                %判断地图周围是否是障碍和已经结束，end结束
                if(map(adjacent_node(1),adjacent_node(2)) ~= Obstacle && map(adjacent_node(1),adjacent_node(2)) ~= Finished)                
                    node_g_cost(adjacent_node(1),adjacent_node(2)) = min_node_cost + 1;
                    node_f_cost(adjacent_node(1),adjacent_node(2)) = node_g_cost(adjacent_node(1),adjacent_node(2)) + fdis(origin, desitination, 1,2);
                    current_direction(adjacent_node(1),adjacent_node(2)) = current_direction(x,y);
                   %做路径平滑
                    %判断地图周围点是不是起点;是：该点父节点为0，不是：父节点为当前点
                    if(map(adjacent_node(1),adjacent_node(2)) == Origin)
                        node_parent(adjacent_node(1),adjacent_node(2)) = 0;
                    else
                        node_parent(adjacent_node(1),adjacent_node(2)) = current_node_index;
                    end
                    %一轮判断结束
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
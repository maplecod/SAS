function [path,iteration_times] = SAS(map,origin,desitination,flystep)
iteration_times = 0;
%% ״̬������ʼ��
%������ ��㣬�յ�
%       ����ɣ�δ���
%       �ϰ�
Obstacle = 2;
Origin = 3;
Destination = 4;
Path = 7;
%% 
%��ʼ������
map_size = size(map);                           %�õ���ͼ�ô�С
%�ڵ��ܴ���ֵ    ԭ��Ϊ0������Ϊ�����
node_g_cost = Inf(map_size(1),map_size(2));
node_g_cost(origin(1),origin(2)) = 0;
%��������
node_f_cost = Inf(map_size(1),map_size(2));
node_f_cost(origin(1),origin(2)) = fdis(origin,desitination,1,2);
%�ڵ㸸�ڵ�     ���и��ڵ�Ϊ0
node_parent = zeros(map_size(1),map_size(2));
%���з���
current_direction = zeros(map_size(1),map_size(2));
parent_direction = zeros(map_size(1),map_size(2));
%�յ�����
destination_index = sub2ind(map_size,desitination(1),desitination(2));
%�Ƿ�ɹ�
plan_succeeded = false;

%% 
while 1
    iteration_times = iteration_times+1;
    [min_node_cost,current_node_index] = min(node_f_cost(:));
    if(min_node_cost == inf || current_node_index == destination_index)
        plan_succeeded = true;
        break;
    end
    %��ǰ������Ϊ����󣬲��ڱ���б��Ϊ����ɡ�
    node_f_cost(current_node_index) = inf;
    %��õ�ǰ�ڵ��������
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
            %�жϵ��Ƿ��ڵ�ͼ�ڲ�
            if((adjacent_node(1) > 0 && adjacent_node(1) <= map_size(1)) && (adjacent_node(2) > 0 && adjacent_node(2) <= map_size(2))) % make sure the adjacent_node don't exceeds the map
                %�жϵ�ͼ��Χ�Ƿ����ϰ����Ѿ�������end����
                if(map(adjacent_node(1),adjacent_node(2)) ~= Obstacle)                
                    node_g_cost(adjacent_node(1),adjacent_node(2)) = min_node_cost + 1;
                    node_f_cost(adjacent_node(1),adjacent_node(2)) = node_g_cost(adjacent_node(1),adjacent_node(2)) + fdis(origin, desitination, 1,2);
                    %�洢��ǰ��չ��÷��� ����ֵΪ���ڵ�÷���
                    current_direction(adjacent_node(1),adjacent_node(2)) = parent_direction(x,y);
                    %�жϵ�ͼ��Χ���ǲ�����㣬�ǣ��õ㸸�ڵ�Ϊ0�����ǣ����ڵ�Ϊ��ǰ��
                    if(map(adjacent_node(1),adjacent_node(2)) == Origin)
                        node_parent(adjacent_node(1),adjacent_node(2)) = 0;
                    else
                        node_parent(adjacent_node(1),adjacent_node(2)) = current_node_index;
                    end
                    %һ���жϽ���
                end
            end
        end
    else
        %% ����ÿ�νǶȵ��Ӷ�������һ���Ƕ����޸ģ��������ڳ�ʼ�Ƕ����޸ġ�
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
            %�жϵ��Ƿ��ڵ�ͼ�ڲ�
            if((adjacent_node(1) > 0 && adjacent_node(1) <= map_size(1)) && (adjacent_node(2) > 0 && adjacent_node(2) <= map_size(2))) % make sure the adjacent_node don't exceeds the map
                %�жϵ�ͼ��Χ�Ƿ����ϰ����Ѿ�������end����
                if(map(adjacent_node(1),adjacent_node(2)) ~= Obstacle && map(adjacent_node(1),adjacent_node(2)) ~= Finished)                
                    node_g_cost(adjacent_node(1),adjacent_node(2)) = min_node_cost + 1;
                    node_f_cost(adjacent_node(1),adjacent_node(2)) = node_g_cost(adjacent_node(1),adjacent_node(2)) + fdis(origin, desitination, 1,2);
                    current_direction(adjacent_node(1),adjacent_node(2)) = current_direction(x,y);
                   %��·��ƽ��
                    %�жϵ�ͼ��Χ���ǲ������;�ǣ��õ㸸�ڵ�Ϊ0�����ǣ����ڵ�Ϊ��ǰ��
                    if(map(adjacent_node(1),adjacent_node(2)) == Origin)
                        node_parent(adjacent_node(1),adjacent_node(2)) = 0;
                    else
                        node_parent(adjacent_node(1),adjacent_node(2)) = current_node_index;
                    end
                    %һ���жϽ���
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
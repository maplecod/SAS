
function [path,iteration_times,current_direction] = AS(map,origin,desitination)
iteration_times = 0;
%% ״̬������ʼ��
%������ ��㣬�յ�
%       ����ɣ�δ���
%       �ϰ�
Obstacle = 2;
Origin = 3;
Destination = 4;
Finished = 5;
Unfinished = 6;
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
current_direction(origin(1),origin(2)) = 2;
%�յ�����
destination_index = sub2ind(map_size,desitination(1),desitination(2));
%�Ƿ�ɹ�
plan_succeeded = false;

%% 
while 1
    iteration_times = iteration_times+1;
    [min_node_cost,current_node_index] = min(node_g_cost(:));
    if(min_node_cost == inf || current_node_index == destination_index)
        plan_succeeded = true;
        break;
    end
    %��ǰ������Ϊ����󣬲��ڱ���б��Ϊ����ɡ�
    node_g_cost(current_node_index) = inf;
    map(current_node_index) = Finished;
    %��õ�ǰ�ڵ��������
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
        %�жϵ��Ƿ��ڵ�ͼ�ڲ�
        if((adjacent_node(1) > 0 && adjacent_node(1) <= map_size(1)) && (adjacent_node(2) > 0 && adjacent_node(2) <= map_size(2))) % make sure the adjacent_node don't exceeds the map
            current_direction(adjacent_node(1),adjacent_node(2)) = k;
            %�жϵ�ͼ��Χ�Ƿ����ϰ����Ѿ�������end����
            if(map(adjacent_node(1),adjacent_node(2)) ~= Obstacle && map(adjacent_node(1),adjacent_node(2)) ~= Finished)                
                if(node_g_cost(adjacent_node(1),adjacent_node(2)) > min_node_cost + 1)
                    if(k <= 4)
                        node_g_cost(adjacent_node(1),adjacent_node(2)) = min_node_cost + 1;
                    else
                        node_g_cost(adjacent_node(1),adjacent_node(2)) = min_node_cost + sqrt(2);
                    end
                    node_f_cost(adjacent_node(1),adjacent_node(2)) = node_g_cost(adjacent_node(1),adjacent_node(2)) + fdis(origin, desitination, 1,2);
                   %��·��ƽ��
                    if(current_direction(x,y) ~= current_direction(adjacent_node(1),adjacent_node(2)))
                        node_f_cost(adjacent_node(1),adjacent_node(2)) = node_f_cost(adjacent_node(1),adjacent_node(2)) + 2;
                    end
%                     current_direction(adjacent_node(1),adjacent_node(2)) = current_direction(x,y);
                    
                    %�жϵ�ͼ��Χ���ǲ�����㣬�ǣ��õ㸸�ڵ�Ϊ0�����ǣ����ڵ�Ϊ��ǰ��
                    if(map(adjacent_node(1),adjacent_node(2)) == Origin)
                        node_parent(adjacent_node(1),adjacent_node(2)) = 0;
                    else
                        node_parent(adjacent_node(1),adjacent_node(2)) = current_node_index;
                    end
                    %һ���жϽ���
                    %��ͼ����Χ����Ϊδ���
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
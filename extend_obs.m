%% 
%输入参数：map，  obsNum,       obsIndex       num,          extend_multiple
%         地图，  障碍数量，    障碍的索引      第几次扩展    扩展倍数
%输出的参数：膨胀路径点
%数组：用来存放膨胀路径
%思路：输入地图和障碍的索引，首先得到数组的尺寸，
function extend_map= extend_obs(map,obsNum,obsIndex,entend_mul)

map_size = size(map);
extend_map = zeros(map_size(1),map_size(2));
[obsx,obsy] = ind2sub(map_size,obsIndex);

for i=1:obsNum
    for j=obsx(i) - entend_mul:obsx(i) + entend_mul
        for k=obsy(i) - entend_mul:obsy(i) + entend_mul
            adjacent = [j,k];
            while(adjacent(1) <= 0)
                adjacent(1)  = 1;
            end
            while(adjacent(1) > map_size(1))
                adjacent(1) = map_size(1);
            end
            while(adjacent(2) <= 0 )
                adjacent(2) = 1;
            end
            while(adjacent(2) > map_size(2))
                adjacent(2) = map_size(2);
            end
             %如果地图上的点不是障碍点的话，对其进行膨胀，加判断语句是防止对已经是障碍点的点再进行置换，之后间障碍点存放一个数组中
            if(map(adjacent(1),adjacent(2)) ~= 2) 
                    extend_map(adjacent(1),adjacent(2)) = 2;
                    extend_map(obsx(i),obsy(i)) = 2;
            end
        end
    end
end

end



%% 
%���������map��  obsNum,       obsIndex       num,          extend_multiple
%         ��ͼ��  �ϰ�������    �ϰ�������      �ڼ�����չ    ��չ����
%����Ĳ���������·����
%���飺�����������·��
%˼·�������ͼ���ϰ������������ȵõ�����ĳߴ磬
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
             %�����ͼ�ϵĵ㲻���ϰ���Ļ�������������ͣ����ж�����Ƿ�ֹ���Ѿ����ϰ���ĵ��ٽ����û���֮����ϰ�����һ��������
            if(map(adjacent(1),adjacent(2)) ~= 2) 
                    extend_map(adjacent(1),adjacent(2)) = 2;
                    extend_map(obsx(i),obsy(i)) = 2;
            end
        end
    end
end

end



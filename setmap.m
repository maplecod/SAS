%% 
%���ܣ������ͼ�ĳ������ϰ�����Ŀ��
%���أ���ͼ���ϰ�����������ͼ�ĳߴ�
function [map,obsIndex] = setmap(row, col, obsNum)

map = zeros(row,col);
map_size = size(map);

obsIndex = randi([1,map_size(1)*map_size(2)],obsNum,1);     %�ϰ������������     floor����
map(obsIndex) = 2;                                        %����ͼ�������ϰ����ֵ����Ϊ2   randi�����������

end
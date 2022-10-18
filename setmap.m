%% 
%功能：输入地图的长、宽、障碍的数目。
%返回：地图、障碍的索引、地图的尺寸
function [map,obsIndex] = setmap(row, col, obsNum)

map = zeros(row,col);
map_size = size(map);

obsIndex = randi([1,map_size(1)*map_size(2)],obsNum,1);     %障碍物的线性索引     floor（）
map(obsIndex) = 2;                                        %定地图区域中障碍物的值设置为2   randi（）随机生成

end
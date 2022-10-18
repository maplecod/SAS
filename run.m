clear;
clc;
%% ��ͼ�е�״̬
Obstacle = 2;
Origin = 3;
Destination = 4;
Finished = 5;
Unfinished = 6;
Path = 7;
%% ������ʼ��
row = 50;
col = 50;
obsNum = 20;
entend_mul = 3;
flastep = 4;
%����յ�
origin = [16,4];
desitination = [46,49];
%% ���������ͼ
% while true
%     [origin_map,obs_index] = setmap(row,col,obsNum);
%     map = extend_obs(origin_map,obsNum,obs_index,entend_mul);
%     if((map(origin(1),origin(2)) ~= Obstacle) && (map(desitination(1),desitination(2)) ~= Obstacle))
%         break;
%     end
% end
%% �����ͼ
load('map');
%% ��ʾ��ǰ��ͼ
mapsize = size(map);
% map(origin(1),origin(2)) = Origin;
% map(desitination(1),desitination(2)) = Destination;
% image(0.5,0.5,map);
% grid on;
% set(gca,'YDir','normal');
% set(gca,'xtick',1:1:row);
% set(gca,'ytick',1:1:col);
% axis image;
%% A*�㷨
[path,iteration_times] = AS1(map,origin,desitination);
path_size = length(path);
pathxy = zeros(path_size,2);
for d=1:path_size
    [tmpx,tmpy] = ind2sub(mapsize,path(d));
    pathxy(d,:) = [tmpx,tmpy];
end
pathxy = [origin;pathxy;desitination];
image(0.5,0.5,map);
colormap([1 1 1;0 0 0]);
grid on;
set(gca,'YDir','normal');
set(gca,'xtick',1:1:mapsize(1));
set(gca,'ytick',1:1:mapsize(2));
axis image;
hold on;
plot(origin(2)-0.5,origin(1)-0.5,'bo','LineWidth',2);
plot(desitination(2)-0.5,desitination(1)-0.5,'g^','LineWidth',2);



if(size(path,2) ~= 0)
    disp('plan succeeded!');
    plot(pathxy(:,2)-0.5,pathxy(:,1)-0.5,'r-','LineWidth',2);
else
    disp('plan failed!');
end

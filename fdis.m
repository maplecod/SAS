function dis = fdis(node1,node2,weight,num)
if(num == 1)%Âü¹þ¶Ù¾àÀë
    dis = weight*(abs(node1(1) - node2(1)) + abs(node1(2) - node2(2)));
elseif(num == 2)
    dis = weight * sqrt((node1(1) - node2(1))^2 + (node1(2) - node2(2))^2);
elseif(num == 3)
    dis = weight * max((node1(1) - node2(1)),(node1(2) - node2(2)));
else
    disp('num false');
end
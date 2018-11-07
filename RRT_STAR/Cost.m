function d = Cost(x1,x2)
%Returns distance between two points give as inputs
d = sqrt((x1.coord(1)-x2.coord(1))^2 + (x1.coord(2)-x2.coord(2))^2);
end


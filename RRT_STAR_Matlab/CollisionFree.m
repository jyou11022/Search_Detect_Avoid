function z = CollisionFree(x,y,Obst)
% Checks if the path between x and y is not in collision with ostacles(Ost)
z = true;
    for a = 1:length(Obst)
        th = 0:pi/50:2*pi;
        xunit = Obst(a).rad.*cos(th) + Obst(a).mid(1);
        yunit = Obst(a).rad.*sin(th) + Obst(a).mid(2);
        [xi, yi] = polyxpoly([x.coord(1),y.coord(1)],[x.coord(2),y.coord(2)], xunit, yunit);
        if isempty([xi,yi]) == false
            z = false; 
            break;
        end
    end
       
end


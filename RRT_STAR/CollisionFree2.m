function z = CollisionFree2(w,x,y,Obst)
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
    if isempty(w)
        z = false;
    else
        x1 = (x.coord-w.coord);
        x2 = (y.coord-x.coord);
        if abs(acosd(dot(x1,x2)/(norm(x1)*norm(x2)))) > 30
            z = false;
        end
    end
    if y.coord(1) < 0 || y.coord(2) < 0 || y.coord(1) > 1 || y.coord(2) > 1
        z = false;
    end
end


function z = Steer(x,y,EPS)
%Returns a point created by the intersection of line between x and y, and circle with x as center and EPS as radius.
%Used to create a point near by x with direction in y and maximum distance of EPS
   if Cost(x,y) < EPS
       z = y;
   else
       z.coord(1) = x.coord(1) + ((y.coord(1)-x.coord(1))*EPS)/Cost(x,y);
       z.coord(2) = x.coord(2) + ((y.coord(2)-x.coord(2))*EPS)/Cost(x,y);
   end
end



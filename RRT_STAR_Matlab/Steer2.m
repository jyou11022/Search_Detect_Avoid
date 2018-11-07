function z = Steer2(ang,w,x,y,EPS)
%Returns a point created by the intersection of line between x and y, and circle with x as center and EPS as radius.
%Used to create a point near by x with direction in y and maximum distance of EPS
   x1 = (x.coord-w.coord);
   x2 = (y.coord-x.coord);
   if abs(acosd(dot(x1,x2)/(norm(x1)*norm(x2)))) <= 30
       z.coord(1) = x.coord(1) + ((y.coord(1)-x.coord(1))*EPS)/Cost(y,x);
       z.coord(2) = x.coord(2) + ((y.coord(2)-x.coord(2))*EPS)/Cost(y,x);
   else
       v1=[((x.coord(1)-w.coord(1))*EPS)/Cost(x,w),((x.coord(2)-w.coord(2))*EPS)/Cost(x,w)];
       v2 = [cosd(ang) -sind(ang); sind(ang) cosd(ang)]*v1';
       z.coord(1) = x.coord(1) + (v2(1));
       z.coord(2) = x.coord(2) + (v2(2));
   end
end



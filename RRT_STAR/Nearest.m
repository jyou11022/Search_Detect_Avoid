function x = Nearest(V,T,x)
%Returns the nearest vertex (from V) to the location of x.  
%   Detailed explanation goes here
c = [];
for y = 1:length(V)
    c = [c Cost(V(y),x)];
end
[a b] = min(c);
x = V(b);
end

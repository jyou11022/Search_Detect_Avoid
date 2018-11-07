function z = Near(V,T,x,Near_r)
%Returns an array of vertices (from V) that are within Near_r distance from
%the location of x.
z = [];

for j = [1:length(V)]
    if Cost(V(j),x) < Near_r
        z = [z V(j)];
    end
end
end


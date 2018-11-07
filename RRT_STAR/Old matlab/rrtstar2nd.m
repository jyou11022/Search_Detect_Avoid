clearvars -except yy
close all

x_goal.coord = [0.9,0,9]; %Final Point (Not fully defined yet)
num_iter = 500; %Set number of iteration
EPS = .05; %Maximum edge length
Near_r = .05;
Obst = [];
yy=[];
x_init.coord = [0.5,0.5];
x_init.cost = 0;
x_init.ind = 1;
Tree = graph;
Tree = addnode(Tree, 1);
Vertex(1) = x_init;    %Initializing Vertex; Edge variable not needed since it is defined within the field

for x = 1:num_iter
    x_rand.coord = [rand(),rand()];
    yy=[yy ; x_rand.coord];
%     x_rand.coord = yy(x,:);
    x_nearest = Nearest(Vertex,Tree,x_rand);
    x_new = Steer(x_nearest,x_rand,EPS);
    if CollisionFree(x_nearest,x_new,Obst)
        X_near = Near(Vertex,Tree,x_new,Near_r);
        x_new.ind = length(Vertex)+1;
        
        x_min = x_nearest; 
        x_min.cost = x_nearest.cost+Cost(x_nearest,x_new);
        for y = 1:length(X_near)
            if CollisionFree(X_near(y),x_new,Obst) && (X_near(y).cost+Cost(X_near(y),x_new))<x_min.cost
                x_min = X_near(y); 
                x_min.cost = X_near(y).cost+Cost(X_near(y),x_new);
            end
        end
        Tree = addedge(Tree,[x_min.ind],[x_new.ind]);
        x_new.cost = x_min.cost + Cost(x_min,x_new);
        Vertex = [Vertex x_new];
        for z = 1:length(X_near)
            nom_cost=[];
            if CollisionFree(x_new,X_near(z),Obst) && (x_new.cost+Cost(x_new,X_near(z)))<X_near(z).cost
                N = neighbors(Tree,X_near(z).ind);
                for j = 1:length(N)
                    nom_cost(j) = Vertex(N(j)).cost;
                end
                [bb cc] = min(nom_cost);
                x_parent = Vertex(N(cc));
                Tree = rmedge(Tree,[x_parent.ind],[X_near(z).ind]);
                Tree = addedge(Tree,[x_new.ind],[X_near(z).ind]);
                Vertex(X_near(z).ind).cost = x_new.cost + Cost(x_new,X_near(z));
            end
        end
    end
end
Vertex;
Tree;
plot(Tree);
figure; hold on;
for aaa = 1:length(Vertex)
    plot(Vertex(aaa).coord(1), Vertex(aaa).coord(2),'x','color','b');
    drawnow;
    hold on;
end
edges = Tree.Edges.EndNodes;
for a = 1:length(Tree.Edges.EndNodes)
    line([Vertex(edges(a,1)).coord(1),Vertex(edges(a,2)).coord(1)],[Vertex(edges(a,1)).coord(2),Vertex(edges(a,2)).coord(2)],'Color', 'm', 'LineWidth', 1)
    drawnow;
    hold on;
end
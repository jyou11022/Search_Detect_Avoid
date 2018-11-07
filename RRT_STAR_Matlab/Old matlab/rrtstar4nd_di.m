clearvars -except yy
close all

figure; hold on;
Obst_1.mid = [0.3,0.5]; Obst_1.rad = .18;
Obst_2.mid = [0.7,0.5]; Obst_2.rad = .18;
Obst = [Obst_1,Obst_2];

for a = 1:length(Obst)
    th = 0:pi/50:2*pi;
    xunit = Obst(a).rad.*cos(th) + Obst(a).mid(1);
    yunit = Obst(a).rad.*sin(th) + Obst(a).mid(2);
    plot(xunit,yunit, 'r');
end
x_goal.coord = [0.9,0,9]; %Final Point (Not fully defined yet)
num_iter = 2000; %Set number of iteration
EPS = .05; %Maximum edge length
Near_r = .08;
% yy=[];
x_init.coord = [0.5,0.5];
plot(x_init.coord(1), x_init.coord(2),'o','color','black');
x_init.cost = 0;
x_init.ind = 1;
Tree = digraph;
Tree = addnode(Tree, 1);
Vertex(1) = x_init;    %Initializing Vertex; Edge variable not needed since it is defined within the field
for x = 1:num_iter
%     x_rand.coord = [rand(),rand()];
%     yy=[yy ; x_rand.coord];
    x_rand.coord = yy(x,:);
    x_nearest = Nearest(Vertex,Tree,x_rand);
    x_new = Steer(x_nearest,x_rand,EPS);
    if CollisionFree(x_nearest,x_new,Obst)
        X_near = Near(Vertex,Tree,x_new,Near_r);
        x_new.ind = length(Vertex)+1;
        
        x_min = x_nearest; 
        c_cost = x_nearest.cost+Cost(x_nearest,x_new);
        for y = 1:length(X_near)
            if CollisionFree(X_near(y),x_new,Obst) && (X_near(y).cost+Cost(X_near(y),x_new))<c_cost
                x_min = X_near(y); 
                c_cost = X_near(y).cost+Cost(X_near(y),x_new);
            end
        end
        Tree = addedge(Tree,[x_min.ind],[x_new.ind]);
%         line([x_min.coord(1),x_new.coord(1)],[x_min.coord(2),x_new.coord(2)],'Color', 'm', 'LineWidth', 1);
%         drawnow;
%         hold on;
        x_new.cost = x_min.cost + Cost(x_min,x_new);
        Vertex = [Vertex x_new];
%         plot(x_new.coord(1), x_new.coord(2),'x','color','b');
%         drawnow;
%         hold on;
        
        for z = 1:length(X_near)
            nom_cost=[];
            if CollisionFree(x_new,X_near(z),Obst) && (x_new.cost+Cost(x_new,X_near(z)))<X_near(z).cost
                
%                 parent_ind = predecessors(Tree,X_near(z).ind);
%                 x_parent = Vertex(parent_ind);
%                 Tree = rmedge(Tree,[x_parent.ind],[X_near(z).ind]);
%                 Tree = addedge(Tree,[x_new.ind],[X_near(z).ind]);
%                 Vertex(X_near(z).ind).cost = x_new.cost + Cost(x_new,X_near(z));

%                 line([x_parent.coord(1),X_near(z).coord(1)],[x_parent.coord(2),X_near(z).coord(2)],'Color', 'w', 'LineWidth', 1.1)
%                 line([x_new.coord(1),X_near(z).coord(1)],[x_new.coord(2),X_near(z).coord(2)],'Color', 'm', 'LineWidth', 1)
%                 drawnow;
%                 hold on;
            end
        end
    end
end
% figure;
% plot(Tree)
for aaa = 1:length(Vertex)
    plot(Vertex(aaa).coord(1), Vertex(aaa).coord(2),'x','color','b');
%     drawnow;
    hold on;
end
edges = Tree.Edges.EndNodes;
for a = 1:length(Tree.Edges.EndNodes)
    line([Vertex(edges(a,1)).coord(1),Vertex(edges(a,2)).coord(1)],[Vertex(edges(a,1)).coord(2),Vertex(edges(a,2)).coord(2)],'Color', 'm', 'LineWidth', 1)
%     drawnow;
    hold on;
end
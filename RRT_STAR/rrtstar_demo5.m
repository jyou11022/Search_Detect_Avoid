%% Jun You 9/11/17
% This script will demonstrate the improved version of rapidly-exploring random tree called RRT*
% The variables defined to describe vertices which are inputs for the functions used in this script are structured with fields:
%     coord : Array of coordinates [x value, y value]
%     ind : used to number the vertices; will be used to keep track of indices used to create network graphs
% 
% Once ran, you will see tree shaped graph with pink line segments representing graphs. 
% The yellow segment will be removed branch and blue segments will be optimized branch. 
% This is to demontrate the different between RRT and RRT*.
% (RRT will not have the blue branches formed but yellow branches as pink
% braches)
% 
% For information on the algorithm, go to section 3 and 4 of the paper on 
% https://people.eecs.berkeley.edu/~pabbeel/cs287-fa15/optreadings/rrtstar.pdf
%% Initialization
clearvars -except yy
close all
figure; 
axis([0 1 0 1]);
hold on;
% Obstacles (circles)
Obst_1.mid = [0.7,0.7]; Obst_1.rad = .18; % Center and radius of circle
Obst_2.mid = [0.3,0.3]; Obst_2.rad = .18;
Obst_3.mid = [0.5,0.5]; Obst_3.rad = .08;
Obst = [Obst_1,Obst_2,Obst_3]; 
% Plotting Obstacles
for a = 1:length(Obst)
    th = 0:pi/100:2*pi;
    xunit = Obst(a).rad.*cos(th) + Obst(a).mid(1);
    yunit = Obst(a).rad.*sin(th) + Obst(a).mid(2);
    plot(xunit,yunit, 'r');
end
waypoints = cat(1,[0.1,0.1],[0.8,0.1],[0.1,0.8],[0.9,0.9]);
num_iter = 200; %Set number of iteration to create random point
EPS = .05; % Maximum edge length
Near_r = .08; % Maximum searching radius for rebranching

for wp_ind = 1:length(waypoints)-1

%Initial Coordinate
x_init.coord = waypoints(wp_ind,:);
x_init.ind = 1; 
x_goal.coord = waypoints(wp_ind+1,:); % Coordinate of the goal
clear Vertex Tree;
V_end = [];
Vertex(1) = x_init;
plot(x_init.coord(1), x_init.coord(2),'o','color','black');
plot(x_goal.coord(1), x_goal.coord(2),'x','color','black');
% Using digraph function to keep track of edges using indices of vertices. 
Tree = digraph;
Tree = addnode(Tree, 1);
% Toggle comment (ctrl+R) or uncomment (ctrl+T) the 4 lines below with *** to use previous random points for testing purposes 
yy=[]; %***
for iter = 1:num_iter
    x_rand.coord = [rand(),rand()]; %*** 
    yy=[yy ; x_rand.coord]; %***
%     x_rand.coord = yy(x,:); %***
    x_nearest = Nearest(Vertex,Tree,x_rand);
    x_new = Steer(x_nearest,x_rand,EPS);
    if CollisionFree(x_nearest,x_new,Obst)
        X_near = Near(Vertex,Tree,x_new,Near_r);
        x_new.ind = length(Vertex)+1;
        x_min = x_nearest; 
        c_cost = distances(Tree,1,x_nearest.ind)+Cost(x_nearest,x_new);
        
        % Connect along a minimum-cost path
        for y = 1:length(X_near)
            if CollisionFree(X_near(y),x_new,Obst) && (distances(Tree,1,X_near(y).ind)+Cost(X_near(y),x_new))<c_cost
                x_min = X_near(y); 
                c_cost = distances(Tree,1,X_near(y).ind)+Cost(X_near(y),x_new);
            end
        end
        Vertex = [Vertex x_new];
        Tree = addedge(Tree,[x_min.ind],[x_new.ind],Cost(x_min,x_new));
        
        % Draw edges and vertices to form paths
        line([x_min.coord(1),x_new.coord(1)],[x_min.coord(2),x_new.coord(2)],'Color', 'y', 'LineWidth', 1);
        plot(x_new.coord(1), x_new.coord(2),'x','color','y');
        hold on;
        
        % Rewire the tree to create optimal paths (You may
        % comment/remove this whole for-loop to change to RRT)
        for z = 1:length(X_near)
            nom_cost=[];
            if CollisionFree(x_new,X_near(z),Obst) && (distances(Tree,1,x_new.ind)+Cost(x_new,X_near(z)))<distances(Tree,1,X_near(z).ind)
                parent_ind = predecessors(Tree,X_near(z).ind);
                x_parent = Vertex(parent_ind);
                Tree = rmedge(Tree,[x_parent.ind],[X_near(z).ind]);
                Tree = addedge(Tree,[x_new.ind],[X_near(z).ind],Cost(x_new,X_near(z)));
                                
                % Draw remove old branch (Redrew in white) and replace with better branch (in blue)
                line([x_parent.coord(1),X_near(z).coord(1)],[x_parent.coord(2),X_near(z).coord(2)],'Color', 'w', 'LineWidth', 1)
                line([x_new.coord(1),X_near(z).coord(1)],[x_new.coord(2),X_near(z).coord(2)],'Color', 'g', 'LineWidth', 1)
                drawnow;
                hold on;
            end
        end
    end
    % Check if the end the last vertex connects to the goal
    if CollisionFree(x_new,x_goal,Obst) && Cost(x_new,x_goal)< 3*EPS
        if length(V_end) < 1 || distances(Tree,1,x_new.ind) < distances(Tree,1,V_end.ind)
            V_end = x_new;
        end
    end
end
    x_goal.ind = length(Vertex)+1;
    Vertex = [Vertex x_goal];
    Tree = addedge(Tree,[V_end.ind],[x_goal.ind],Cost(V_end,x_goal));
    plot(x_goal.coord(1), x_goal.coord(2),'x','color','black');
    path = shortestpath(Tree,1,x_goal.ind);
    for j = 1:length(path)-1
        line([Vertex(path(j)).coord(1),Vertex(path(j+1)).coord(1)],[Vertex(path(j)).coord(2),Vertex(path(j+1)).coord(2)],'Color', 'black', 'LineWidth', 1)
    end
end
% The script below is another way of plotting paths. Use this if you want to graph directly from the network graph. 
% figure;
% plot(Tree)
% figure;
% for aaa = 1:length(Vertex)
%     plot(Vertex(aaa).coord(1), Vertex(aaa).coord(2),'x','color','b');
%     drawnow;
%     hold on;
% end
% edges = Tree.Edges.EndNodes;
% for a = 1:length(Tree.Edges.EndNodes)
%     line([Vertex(edges(a,1)).coord(1),Vertex(edges(a,2)).coord(1)],[Vertex(edges(a,1)).coord(2),Vertex(edges(a,2)).coord(2)],'Color', 'm', 'LineWidth', 1)
%     drawnow;
%     hold on;
% end
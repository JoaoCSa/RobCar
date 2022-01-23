load('graph_data.mat');
load('oc_mat.mat');

%% Computes graph and shortest path


%% Select starting point and end point
valid = 0;

while valid ~= 1
    
figure(1)
imshow(map)
hold on
plot(G,'XData',x,'YData',y,'NodeLabelColor','w','NodeColor','w','EdgeColor','w')    
    
[xi,yi] = ginput(1);
hold on
p1 = plot(xi,yi,'r*');
p1t = text(xi,yi,'IP');

[xf,yf] = ginput(1);
hold on
p2 = plot(xf,yf,'g*');
p2t = text(xf,yf,'FP');

if oc_mat(round(yf),round(xf)) == 0 && oc_mat(round(yi),round(xi)) == 0
    valid = 1;
else
    set(p1,'Visible','off')
    set(p1t,'Visible','off')
    set(p2,'Visible','off')
    set(p2t,'Visible','off')
end

end

%% Add starting point to the graph

d = 10^10; % Initialize distance in infinity
for i = 1:numel(s) 
    v1 = [x(s(i)), y(s(i))];
    v2 = [x(t(i)), y(t(i))];
    dist = dist_to_edge([xi,yi],v1,v2);
if dist < d
    d = dist;
    ind = i; 
end
end

x = [x; xi];
y = [y; yi];
i_pi = numel(x);

s = [s s(ind)];
t = [t i_pi];

s = [s i_pi];
t = [t t(ind)];

if  findedge(G,t(ind),s(ind)) ~= 0 % Checks for undirected edge
    s = [s i_pi];
    t = [t s(ind)];
    s = [s t(ind)];
    t = [t i_pi];
end
   
s(ind) = [];
t(ind) = [];

%% Add final point to the graph

d = 10^10; % Initialize distance in infinity
for i = 1:numel(s) 
    v1 = [x(s(i)), y(s(i))];
    v2 = [x(t(i)), y(t(i))];
    dist = dist_to_edge([xf,yf],v1,v2);
if dist < d
    d = dist;
    ind = i; 
end
end

x = [x; xf];
y = [y; yf];
i_pf = numel(x);

s = [s s(ind)];
t = [t i_pf];
s = [s i_pf];
t = [t t(ind)];

if findedge(G,t(ind),s(ind)) ~= 0 % Checks for undirected edge
    s = [s i_pf];
    t = [t s(ind)];
    s = [s t(ind)];
    t = [t i_pf];
end

s(ind) = [];
t(ind) = [];

%% Update the graph
weights = sqrt((x(s) - x(t)).^2 + (y(s) - y(t)).^2); % Distance between nodes
G = digraph(s,t,weights);

figure(2)
imshow(map)
hold on
plot(G,'XData',x,'YData',y,'NodeLabelColor','w','NodeColor','w','EdgeColor','w');

%% Finds shortest path between designated points

init = i_pi;
final = i_pf;

[short_path,d] = shortestpath(G,init,final);

figure(3)
imshow(map)
hold on
p = plot(G,'XData',x,'YData',y,'EdgeLabel',G.Edges.Weight,'NodeLabelColor','w','EdgeLabelColor','w','NodeColor','w','EdgeColor','w');
highlight(p,short_path,'EdgeLabelColor','y','NodeColor','y','EdgeColor','r','LineWidth',2);

%% Smooth trajectory

x_s = x(short_path); 
y_s = y(short_path);

z = linspace(0, 1, numel(x_s));

m = numel(x_s); % Number of points in the trajectory
zz = linspace(0, 1, m);

xx = pchip(z, x_s, zz); % pchip seems to work best for spline interpolation
yy = pchip(z, y_s, zz);

figure(4)
imshow(map)

hold on
plot(xi,yi,'r*')
text(xi,yi,'IP')
hold on
plot(xf,yf,'g*')
text(xf,yf,'FP')
hold on
plot(xx, yy,'b','LineWidth',2);

%% Convert pixel coordinates to real dimensions (m)

bar = 270; %270 pixel (measured with imagetool)
bar_length = 50; %50 m

K = bar_length/bar; % 0.1852 m/pixel

xx = xx*K;
yy = yy*K;
d = d*K;

%{
   Arrays xx and yy  have trajectory coordinates in meters
%}

x(end-1:end) = []; y(end-1:end) = []; % Removes added points from x and y


%% Runs robot simulation
rob



%% Functions
function dist = dist_to_edge(pt,v1,v2)
    d_v1v2 = norm(v1-v2);
    d_v1pt = norm(v1-pt);
    d_v2pt = norm(v2-pt); 
    
    if dot(v1-v2,pt-v2)*dot(v2-v1,pt-v1)>=0
        A = [v1,1;v2,1;pt,1];
        dist = abs(det(A))/d_v1v2;        
    else
        dist = min(d_v1pt, d_v2pt);
    end
end

function G = add_node_to_graph()
end

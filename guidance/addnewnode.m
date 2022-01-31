% Adds new point to the graph
function [G,s,t,x,y,ind_p] = addnewnode(G,s,t,x,y,xp,yp)

d = 10^10; % Initialize distance in infinity

for i = 1:numel(s)
    v1 = [x(s(i)), y(s(i))];
    v2 = [x(t(i)), y(t(i))];
    dist = distToEdge([xp,yp],v1,v2);
    if dist < d
        d = dist;
        ind = i;
    end
end

x = [x; xp];
y = [y; yp];
ind_p = numel(x);

s = [s s(ind)];
t = [t ind_p];

s = [s ind_p];
t = [t t(ind)];

if  findedge(G,t(ind),s(ind)) ~= 0 % Checks for undirected edge on the graph
    s = [s ind_p];
    t = [t s(ind)];
    s = [s t(ind)];
    t = [t ind_p];
end

s(ind) = [];
t(ind) = [];

% Update the graph
weights = sqrt((x(s) - x(t)).^2 + (y(s) - y(t)).^2); % Updates graph weights
G = digraph(s,t,weights);

end


% Computes closest distance from a point to an edge
function dist = distToEdge(pt,v1,v2)
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
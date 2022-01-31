function [q,qd,qdd,tSamples] = trajectorygen()


% Constants for dimensional conversion
bar = 270; % 270 pixel (measured with imagetool)
bar_length = 50; % 50 m
K = bar_length/bar; % Approximately 0.1852 m/pixel

% Shows map for point selection
figure
imshow(map)
hold on
plot(G,'XData',x,'YData',y,'NodeLabelColor','w','NodeColor','w','EdgeColor','w')
hold on
plot(black_coord(2,:),black_coord(1,:),'w.');
plot(red_coord(2,:),red_coord(1,:),'r.');
plot(green_coord(2,:),green_coord(1,:),'g.');
plot(blue_coord(2,:),blue_coord(1,:),'b.');

% Select starting point
[xi,yi] = ginput(1);
hold on
plot(xi,yi,'r.');
text(xi,yi,'IP');

% Select end point
[xf,yf] = ginput(1);
hold on
plot(xf,yf,'g.');
text(xf,yf,'FP');

% Add starting point to the graph
[G,s,t,x,y,ind_pi] = addNode(G,s,t,x,y,xi,yi);

% Add final point to the graph
[G,s,t,x,y,ind_pf] = addNode(G,s,t,x,y,xf,yf);

% Finds shortest path between designated points
init = ind_pi;
final = ind_pf;
[short_path,d] = shortestpath(G,init,final);

% Adds points to path and smoothens
x_s = x(short_path);
y_s = y(short_path);
z = linspace(0, 1, numel(x_s));

m = round(d); % Number of points proportional to total distance (about one every image pixel)
zz = linspace(0, 1, m);
x_zones = pchip(z, x_s, zz); 
y_zones = pchip(z, y_s, zz);

% Shows all points
hold on 
plot(x_zones,y_zones,'m.-');
%%
% Gets corresponding zone for each point
for i = 1:length(x_zones)
    pt_zones(i) = oc_mat(round(y_zones(i)),round(x_zones(i)));
end
%%
% Split path according to zones

% 1 - Check if there is a stop sign along the way
stop = ismember(2,pt_zones);

tic
% 2 - If there is a stop along the way split the path in two 
if stop
    stop_zone = find(pt_zones==2);
    stop_ind = stop_zone(ceil(end/2));
    path1 = [x_zones(1:stop_ind); y_zones(1:stop_ind)]*K;
    path2 = [x_zones(stop_ind:end); y_zones(stop_ind:end)]*K;
    
    plot(path1(1,:)/K,path1(2,:)/K,'y.');
    plot(path2(1,:)/K,path2(2,:)/K,'c.');

%%    
    % Generate both trajectories
    [q1,qd1,qdd1,qddd1,tSamples1] = createTrajectory(path1);
    [q2,qd2,qdd2,qddd2,tSamples2] = createTrajectory(path2);

    % Concatenate the two
    q = [q1, q2];
    qd = [qd1, qd2];
    qdd = [qdd1, qdd2];
    qddd = [qddd1, qddd2];
    tSamples = [tSamples1, tSamples2+tSamples1(end)];

else
    path = [x_zones;y_zones]*K;
    
    % Generate trajectory
    [q,qd,qdd,qddd,tSamples] = createTrajectory(path);

end
toc

%%
figure; imshow(map)
hold on
%plot(q1(1,end)/K,q1(2,end)/K,'*r')
for i = 1:length(qddd)
hold on
plot(q(1,i)/K,q(2,i)/K,'.g')
end



%% Display trajectory data 

fprintf('TRAJECTORY INFORMATION: \n')

% Total estimated time
fprintf('Total estimated time: %f s \n', tSamples(end))

% Total energy  consumption
P0 = 100; % 100 J/s
M = 810; % 810 kg

% totalenergy = 
% fprintf('Total energy consumption: \n')


% Max velocity
lv = sqrt(qd(1,:).^2 + qd(2,:).^2);
max_lv = max(lv);
fprintf('Max linear velocity: %f m/s',max_lv)

% Average velocity


figure
num = length(q);
subplot(3,2,1),plot(tSamples(1:num),q(1,1:num));title('x position');
xlabel('t (s)')
subplot(3,2,2),plot(tSamples(1:num),q(2,1:num));title('y position');
xlabel('t (s)')
subplot(3,2,3),plot(tSamples(1:num),qd(1,1:num));title('x velocity');
xlabel('t (s)')
subplot(3,2,4),plot(tSamples(1:num),qd(2,1:num));title('y velocity');
xlabel('t (s)')
subplot(3,2,5),plot(tSamples(1:num),qdd(1,1:num));title('x acceleration');
xlabel('t (s)')
subplot(3,2,6),plot(tSamples(1:num),qdd(2,1:num));title('y acceleration');
xlabel('t (s)')
end

%% Functions

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


% Adds new point to the graph
function [G,s,t,x,y,ind_p] = addNode(G,s,t,x,y,xp,yp)
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


% Creates minimum jerk trajectory from given waypoints
function [q,qd,qdd,qddd,tSamples] = createTrajectory(waypoints) % Waypoints already in meters!!!

    ptd = sqrt(sum(abs(diff(waypoints,1,2)).^2,1)); % Distance between points

    pathd = sum(ptd,2); % Total path distance (m)

    exp_avg_vel = 4.5; % Expected average velocity 

    ft = pathd/exp_avg_vel; % Compute expected final time

    dt = 1; % Time intervals equaly spaced

    tpts = 0:dt:ft-dt;

    timePoints = tpts;

    % Smoothed path with waypoints matching the time intervals

    x_s = waypoints(1,:);
    y_s = waypoints(2,:);

    z = linspace(0, 1, numel(x_s));

    m = numel(tpts); % Number of points in the trajectory with triple the points
    zz = linspace(0, 1, m);

    xx = pchip(z, x_s, zz); 
    yy = pchip(z, y_s, zz);

    waypoints = [xx; yy];

%     v = round(linspace(1,length(waypoints),round(length(waypoints)/10)));
% 
%     waypoints_v = waypoints(:,v);
%     timePoints_v = timePoints(:,v);

    numSamples = round(pathd)*5;

%     vel_max = 5.5;
%     minsegtime = sqrt(sum(abs(diff(waypoints_v,1,2)).^2,1))/vel_max;
    %maxsegtime = 2*ones(1,length(minsegtime));

    [q,qd,qdd,qddd,~,~,tSamples] = minjerkpolytraj(waypoints,timePoints,numSamples,'TimeAllocation',true,'TimeWeight',1000,'MinSegmentTime',0.01,'MaxSegmentTime',1);
    %  ALTERAR ISTO

    % Uniform trajectory scaling to fit max velocity constraints 
    lv = sqrt(qd(1,:).^2 + qd(2,:).^2);
    max_lv = max(lv); % Determines max linear velocity of generated trajectory
    vel_lim = 5.5; % Max velocity of 5.5 m/s (about 20 km/h)
    
    % Trajectory scaling
    if max_lv > vel_lim
    tSamples = tSamples*max_lv/vel_lim;
    qd = qd*vel_lim/max_lv;
    qdd = qdd*(vel_lim/max_lv)^2;
    qddd = qddd*(vel_lim/max_lv)^3;
    end

end


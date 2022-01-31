function [q,qd,qdd,times]=guidance()

clear 
close all
clc

% Import map and graph data
load('map.mat');
load('graph.mat');

% Constants for dimensional conversion
bar = 270; % 270 pixel (measured with imagetool)
bar_length = 50; % 50 m
K = bar_length/bar; % Approximately 0.1852 m/pixel


%% 1 - Select initial and final positions
[ip,fp] = pointselect();


%% 2 - Add selected points to the graph
[G,s,t,x,y,ind_ip] = addnewnode(G,s,t,x,y,ip(1),ip(2));
[G,s,t,x,y,ind_fp] = addnewnode(G,s,t,x,y,fp(1),fp(2));

% Shows entire updated graph
imshow(map)
title('Complete graph')
hold on
plot(G,'XData',x,'YData',y,'NodeLabelColor','w','NodeColor','w','EdgeColor','w');
pause(5);


%% 3 - Finds the shortest path between the two selected points
[short_path,d] = shortestpath(G,ind_ip,ind_fp);

imshow(map)
title('Shortest path between selected position - Dijkstra algorithm')
hold on
p = plot(G,'XData',x,'YData',y,'NodeLabelColor','w','EdgeLabelColor','w','NodeColor','w','EdgeColor','w');
highlight(p,short_path,'EdgeLabelColor','y','NodeColor','y','EdgeColor','r','LineWidth',2);
pause(5);

%% 4 - Path smoothing 
[x_s,y_s] = smoothenpath(x(short_path),y(short_path),d);

imshow(map)
title('Interpolated smoothed path')
hold on
plot(x_s,y_s,'b.-');
pause(5);


%% 5 - Check zones for events along path (stop signs, crosswalks, etc.)
zones = checkzones(x_s,y_s);

figure
imshow(map)
title('Zones to check events')
hold on
plot(black_coord(2,:),black_coord(1,:),'w.');
plot(red_coord(2,:),red_coord(1,:),'r.');
plot(green_coord(2,:),green_coord(1,:),'g.');
plot(blue_coord(2,:),blue_coord(1,:),'b.');
pause(5);


%% 6 - Generate trajectory according to circulation conditions
[q,qd,qdd,qddd,times] = generatetrajectory(x_s,y_s,zones,K);

figure
sgtitle('Generated trajectory')
num = length(q);
subplot(3,2,1),plot(times(1:num),q(1,1:num));title('x position');
xlabel('t (s)')
subplot(3,2,2),plot(times(1:num),q(2,1:num));title('y position');
xlabel('t (s)')
subplot(3,2,3),plot(times(1:num),qd(1,1:num));title('x velocity');
xlabel('t (s)')
subplot(3,2,4),plot(times(1:num),qd(2,1:num));title('y velocity');
xlabel('t (s)')
subplot(3,2,5),plot(times(1:num),qdd(1,1:num));title('x acceleration');
xlabel('t (s)')
subplot(3,2,6),plot(times(1:num),qdd(2,1:num));title('y acceleration');
xlabel('t (s)')
pause(5);

figure
imshow(map)
title('Trajectory visulization')
hold on
for i = 1:length(q)
plot(q(1,i)/K,q(2,i)/K,'oy')
hold on
xlabel('X')
ylabel('Y')
pause(0.01)
end
pause(5)


%% 7 - Trajectory information

% Total estimated time
fprintf('Total estimated time: %f s \n', times(end))

% Total energy  consumption
P0 = 20; % 20 J/s (static consumption)
M = 810; % 810 kg

v = sqrt(qd(1,:).^2 + qd(2,:).^2);
a = sqrt(qdd(1,:).^2 + qdd(2,:).^2);

totalenergy = sum((M*abs((v(1:end-1)+v(2:end))/2).*abs((a(1:end-1)+a(2:end))/2)+P0).*diff(times));

% Max velocity
max_v = max(v);
fprintf('Max linear velocity: %f m/s \n',max_v)

% Total distance covered
totaldist = sum(sqrt(sum(abs(diff(q,1,2)).^2,1)));

msgbox(sprintf(['Total estimated time: %.3f s \n' ...
    'Estimate of total energy consumption: %.3f J \n' ...
    'Maximum linear velocity: %.2f m/s \n' ...
    'Total distance covered: %.3f m \n'], ...
    times(end),totalenergy,max_v,totaldist),'Trajectory information','modal')

pause(10)
close all hidden
close all force

end

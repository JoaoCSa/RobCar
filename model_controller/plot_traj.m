if exist('fig','var')
	for ii = 1:length(fig)
		clf(fig(ii));
	end
end
%constants;
%out = sim('car_model_with_controller_2020a.slx');


fig = [];
fig = [fig;figure(1)];
hold on;
plot(trajectory.q(1,:),trajectory.q(2,:),'.-');
plot(out.x.Data,out.y.Data,'.-');
title('Path Following');
xlabel('x (m)');
ylabel('y (m)');
legend('reference','trajectory');
%axis equal;
hold off;

points1 = round(linspace(1,length(trajectory.tSamples),1000));
points2 = round(linspace(1,length(out.y.Data),5000));

fig = [fig;figure(2)];
hold on;
quiver(trajectory.q(1,points1),trajectory.q(2,points1),trajectory.qd(1,points1),trajectory.qd(2,points1),0.5);
quiver(out.x.Data(points2),out.y.Data(points2),out.x_dot.Data(points2),out.y_dot.Data(points2), 0.5);
title('Linear Velocity Vectors');
xlabel('x (m)');
ylabel('y (m)');
legend('reference','trajectory');
hold off;


fig = [fig;figure(3)];
hold on;
plot(out.wref.Time,out.wref.Data);
plot(out.w.Time,out.w.Data);
title('Angular Velocity');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('reference','simulated');
hold off;

fig = [fig;figure(4)];
hold on;
plot(out.vref.Time,out.vref.Data);
plot(out.v.Time,out.v.Data);
title('Linear Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('reference','simulated');
hold off;

fig = [fig;figure(5)];
plot(out.Energy.Time,out.Energy.Data);
title('Total Used Energy ');
xlabel('Time (s)');
ylabel('Energy (J)');



map_campus = open('map_campus.mat');
fig = [fig;figure(6)];
imshow(map_campus.map);
hold on;
plot(trajectory.q(1,:)/k,trajectory.q(2,:)/k,'.-');
plot(out.x.Data/k,out.y.Data/k,'.-');
plot(out.x_colision/k,out.y_colision/k);
legend('reference','trajectory','colisions');
hold off;


% if exist('fig','var')
% 	for ii = 1:length(fig)
% 		clf(fig(ii));
% 	end
% end
fig = [];
fig = [fig;figure(1)];
hold on;
plot(trajectory.q(1,:),trajectory.q(2,:),'.-');
plot(out.x.Data,out.y.Data,'.-');
hold off;

points1 = round(linspace(1,length(trajectory.tSamples),1000));
points2 = round(linspace(1,length(out.y.Data),5000));

fig = [fig;figure(2)];

hold off;


fig = [fig;figure(3)];
hold on;
plot(out.x.Time,out.x.Data);
plot(out.y.Time,out.y.Data);
hold off;

fig = [fig;figure(4)];
hold on;
plot(trajectory.tSamples,THETAD);
plot(trajectory.tSamples,OMEGAD);
hold off;


fig = [fig;figure(5)];
hold on;
plot(out.x_dot.Time,out.x_dot.Data);
plot(out.y_dot.Time,out.y_dot.Data);
hold off;

fig = [fig;figure(6)];
hold on;
plot(VD)
hold off;

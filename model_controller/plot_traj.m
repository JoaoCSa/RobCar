for ii = 1:length(fig)
	clf(fig(ii));
end
fig = [];
fig = [fig;figure(1)];
hold on;
plot(trajectory.q(2,:),trajectory.q(1,:),'.-');
plot(out.y.Data,out.x.Data,'.-');
hold off;

points1 = round(linspace(1,length(trajectory.tSamples),100))
points2 = round(linspace(1,length(out.y.Data),100))

fig = [fig;figure(2)];
hold on;
quiver(trajectory.q(1,points1),trajectory.q(2,points1),trajectory.qd(1,points1),trajectory.qd(2,points1),0.5);
quiver(out.x.Data(points2),out.y.Data(points2),out.x_dot.Data(points2),out.y_dot.Data(points2), 0.5);
hold off;
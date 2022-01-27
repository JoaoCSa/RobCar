cla;
fig = [];
fig = [fig;figure(1)];
hold on;
plot(trajectory.q(2,:),trajectory.q(1,:),'.-');
plot(out.y.Data,out.x.Data,'.-');
hold off;
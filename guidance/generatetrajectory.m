function [q,qd,qdd,qddd,times] = generatetrajectory(x,y,zones,K)

msgbox('Generating trajectory. This process may take a few seconds.', 'Trajectory generation','help');

% 1 - Check if there is a stop sign along the way (since there is only one
% in the entire map)
stop = ismember(2,zones);

if stop

    stop_zone = find(zones==2);
    stop_ind = stop_zone(ceil(end/2));
    path1 = [x(1:stop_ind); y(1:stop_ind)]*K;
    path2 = [x(stop_ind:end); y(stop_ind:end)]*K;

    % Generate both trajectories
    [q1,qd1,qdd1,qddd1,tSamples1] = createTrajectory(path1);
    [q2,qd2,qdd2,qddd2,tSamples2] = createTrajectory(path2);

    % Concatenate the two
    q = [q1, q2];
    qd = [qd1, qd2];
    qdd = [qdd1, qdd2];
    qddd = [qddd1, qddd2];
    times = [tSamples1, tSamples2+tSamples1(end)+1];

else
    path = [x;y]*K;
    
    % Generate trajectory
    [q,qd,qdd,qddd,times] = createTrajectory(path);

end

end



% Creates minimum jerk trajectory from given waypoints
function [q,qd,qdd,qddd,tSamples] = createTrajectory(waypoints)

    ptd = sqrt(sum(abs(diff(waypoints,1,2)).^2,1)); % Distance between points

    pathd = sum(ptd,2); % Total path distance

    exp_avg_vel = 4.5; % Expected average velocity 

    ft = pathd/exp_avg_vel; % Expected final time 

    dt = 1; % Time intervals equaly spaced

    timePoints = 0:dt:ft;

    % Smoothed path with waypoints matching the time intervals

    x_s = waypoints(1,:);
    y_s = waypoints(2,:);

    z = linspace(0, 1, numel(x_s));

    m = numel(timePoints); 
    zz = linspace(0, 1, m);

    xx = pchip(z, x_s, zz); 
    yy = pchip(z, y_s, zz);

    waypoints = [xx; yy];

    numSamples = round(pathd)*2;

    % Generate minimum jerk trajectory
    warning off
    [q,qd,qdd,qddd,~,~,tSamples] = minjerkpolytraj(waypoints,timePoints,numSamples,'TimeAllocation',true,'TimeWeight',10000,'MinSegmentTime',0.01,'MaxSegmentTime',5);

    % Determine peak linear velocity
    lv = sqrt(qd(1,:).^2 + qd(2,:).^2);
    max_lv = max(lv); % Determines max linear velocity of generated trajectory
    vel_lim = 5.5; % Max velocity of 5.5 m/s (approximately 20 km/h)
    
    % Uniform trajectory scaling according to speed limits
    if max_lv > vel_lim
    tSamples = tSamples*max_lv/vel_lim;
    qd = qd*vel_lim/max_lv;
    qdd = qdd*(vel_lim/max_lv)^2;
    qddd = qddd*(vel_lim/max_lv)^3;
    end
end
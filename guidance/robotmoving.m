%% Simulation

% Initialize the simulation loop
sampleTime = 0.1; % Choose right sample time (s)
vizRate = rateControl(1/sampleTime); % Visualization frequency
elapsedTime = 0;

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = 1.5; % Adjust robot mesh size [CHECK DIMENSIONS]

% Initialize figure
figure
set(gcf,'Position',[100 100 1500 1500])
% Zoom level
zoom = 60;

while(distanceToGoal > goalRadius)
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose); % [CHECK THIS FUNCTION]
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]); % Computes vx, vy and w
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; % Updates robot pose
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:)); % Updates distance to goal
    
    %% Visual representation
    
    % Update the plot
    hold off    
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0]; % Position (x=robotCurrentPose(1),y=robotCurrentPose(2),z=0)
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]); % Orientation (rotation only around Z axis)
    
    
    % 2D perspective of robot pose
    imshow(I)
    hold on
    plot(path(:,1), path(:,2),"k--.",'MarkerEdgeColor','y')
    %hold all
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 2,'MeshColor','k');
    light;
    % Zoom on vehicle position
    xlim([robotCurrentPose(1)-zoom, robotCurrentPose(1)+zoom])
    ylim([robotCurrentPose(2)-zoom, robotCurrentPose(2)+zoom])
    axis on
    xlabel('X [m]')
    ylabel('Y [m]')
    hold off
    
    %% Display vehicle data (linear velocity, position, angular velocity) [TRY SHOWING THROTTLE, BRAKE, STEERING...]
    
    elapsedTime = elapsedTime + sampleTime;
    
    disp(elapsedTime);
    
    % Pauses simulation according to time rate
    waitfor(vizRate);
end

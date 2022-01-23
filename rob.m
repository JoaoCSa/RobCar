%% Imports image and positions
I = imresize(map,K);

path = [xx' yy'];

%% Defines initial pose 
robotInitialLocation = path(1,:); % Starting position

robotGoal = path(end,:); % Final position

initialOrientation = atan2(path(2,2)-path(1,2),path(2,1)-path(1,1)); % Starting orientation facing trajectory

robotCurrentPose = [robotInitialLocation initialOrientation]'; % Robot pose (position + orientation)


%% Robot Kinematics

robot = differentialDriveKinematics("TrackWidth", 1.508, "VehicleInputs", "VehicleSpeedHeadingRate"); % [ADJUST DIMENSIONS]: W = 1.508m; L = 3.332m; 

controller = controllerPurePursuit; % Selects type of controller [ADJUST PARAMETERS]

controller.Waypoints = path; % Path points (from path plan)

controller.DesiredLinearVelocity = 6; % Robot linear velocity

controller.MaxAngularVelocity = 4; % Robot angular velocity

controller.LookaheadDistance = 0.9; % Controller "range" 

goalRadius = 1; % Radius of final destination area

distanceToGoal = norm(robotInitialLocation - robotGoal); % Distance to final destination


%% Simulation

% Initialize the simulation loop
sampleTime = 0.1; % Choose right sample time (s)
vizRate = rateControl(1/sampleTime); % Visualization frequency
elapsedTime = 0;

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/1; % Adjust robot mesh size [CHECK DIMENSIONS]

% Initialize figure
figure
set(gcf,'Position',[100 100 1500 1500])
% Zoom level
zoom = 60;

tic
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose); % [CHECK THIS FUNCTION]
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]); % Computes vx, vy and w
    
        % Check velocities:
        %     disp(vel)
        %     
        %     totalvel = sqrt(vel(1)^2 + vel(2)^2);
        %     disp(totalvel)
        %     
        %     disp(vel(3))
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; % Updates robot pose
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:)); % Updates distance to goal
    
    %% Visual representation
    
    % Update the plot
    hold off
    
    % Plot path each instance consistent with robot movement on the map
    
    
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
toc
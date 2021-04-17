roslaunch simulator simulator.launch 

rosrun simulator kalman_visualization.py 

rosrun  simulator kalman

rosrun simulator kalman_to_blobs


close all

%%%%% ROBOT SETUP %%%%%%


    % Robot configuration
    R = 0.1;   % Wheels [m]
    L = 0.5;   % Base [m]
    dd = DifferentialDrive(R,L);

    
%%%%% SENSORS SETUP %%%%%


    % Create object Detector sensor
    detector = ObjectDetector;
    detector.fieldOfView = pi/4;
    detector.maxRange = 8; %[m]        
    detector.maxDetections = 25;
    
    % Create lidar sensor
    lidar = LidarSensor;
    lidar.sensorOffset = [0,0];
    lidar.scanAngles = linspace(-pi/2,pi/2,51);
    lidar.maxRange = 5;

    

%%%%% FIELD SETUP %%%%%

    % Initial position
    initPose = [2;2;0];            % xy theta
    
    % Lead image as map
    image = imread('C:\Users\Diego\Pictures\wordx.png');
    grayimage = rgb2gray(imresize(image, 1.0));
    bwimage = grayimage < 0.5;
    map = binaryOccupancyMap(bwimage);

    %objects (as landmarks) in the field
    % Create objects and colors 
    colors = zeros(25,3);
    objects = zeros(25,3);
    cta = 1;
    for row=1:5
        for col=1:5
           objects(cta,1) = row*5;
           objects(cta,2) = col*5;
           objects(cta,3) = cta;
           colors(cta,1) = .3;
           colors(cta,2) = .8;
           colors(cta,3) = .5;
           cta = cta +1;
        end
    end

    % Waypoints
    waypoints = [2 10;
                 2 17
                 9 19;
                 14 8;
                 27 15];

    % Create visualizer
    viz = Visualizer2D;
    viz.hasWaypoints = true;
    viz.mapName = 'map';
    attachLidarSensor(viz,lidar);

    % Colores de los objetos
    attachObjectDetector(viz,detector);
    viz.objectColors = colors;
    viz.objectMarkers = 'so<sd^<>phpo<sd^<>phpo<sd';
    
    
%%%%% ALGORITHMS SETUP

    % Pure Pursuit Controller
    controller = controllerPurePursuit;
    controller.Waypoints = waypoints;
    controller.LookaheadDistance = 0.5;
    controller.DesiredLinearVelocity = 0.75;
    controller.MaxAngularVelocity = 1.5;

    % Vector Field Histogram (VFH) for obstacle avoidance
    vfh = controllerVFH;
    vfh.DistanceLimits = [0.05 3];
    vfh.NumAngularSectors = 36;
    vfh.HistogramThresholds = [5 10];
    vfh.RobotRadius = L;
    vfh.SafetyDistance = L;
    vfh.MinTurningRadius = 0.25;


    %%% EKF
    
    varianceV= .05;
    meanValueV = 0;
    
    varianceW= .08;
    meanValueW = 0;
    
    

 %%% SIMULATION SETUP
 
    % Sample time and time array
    sampleTime = 0.1;              % Sample time [s]
    tVec = 0:sampleTime:65;        % Time array
    r = rateControl(1/sampleTime);

    %Pose Ideal
    pose = zeros(3,numel(tVec));   % Pose matrix
    pose(:,1) = initPose;   % set first pose as the initial position

    %Pose with added noise
    poseWithNoise = zeros(3,numel(tVec)); 
    poseWithNoise(:,1) = initPose;  %Same initial pose
    
  %%% SIMULATION
    
for idx = 2:numel(tVec) 
    
    %Update pose
    curPoseFake = pose(:,idx-1);
    curPoseReal = poseWithNoise(:,idx-1);
    
    % Get the sensor readings from the real pose
    ranges = lidar(curPoseReal);
        
    % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPoseFake);
    targetDir = atan2(lookAheadPt(2)-curPoseFake(2),lookAheadPt(1)-curPoseFake(1)) - curPoseFake(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    
    velBFake = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    
    %%Add Noise
    vRef = vRef + (sqrt(varianceV)*randn(size(vRef)) + meanValueV);
    wRef = wRef + (sqrt(varianceW)*randn(size(vRef)) + meanValueW/2);
    
    % Control the robot
    velBReal = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    
    velFake = bodyToWorld(velBFake,curPoseFake);  % Convert from body to world
    velReal = bodyToWorld(velBReal,curPoseReal);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPoseFake + velFake*sampleTime; 
    poseWithNoise(:,idx) = curPoseReal + velReal*sampleTime; 
    
    
    % Update visualization
    
    % Update object detector and visualization
    detections = detector(poseWithNoise(:,idx),objects);
    viz(poseWithNoise(:,idx),waypoints,ranges,objects);
    
    detections
    
    % Display object detections every 10th iteration  
%     if mod(idx,10) == 0
%         if ~isempty(detections)
%             nearestLabel = detections(1,3);
%             disp(['Nearest object is of label ' num2str(nearestLabel)]); 
%         else
%             disp('No objects detected'); 
%         end
%     end  
    
    waitfor(r);
    
end

hold on

%% show path with noise and ideal path 
plot(pose(1,:),pose(2,:));
plot(poseWithNoise(1,:),poseWithNoise(2,:));

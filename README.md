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
    
    %Atach sensors
    attachLidarSensor(viz,lidar);
    attachObjectDetector(viz,detector);
    
    % Colores de los objetos
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
    
    varianceV= .03;
    meanValueV = 0;
    
    varianceW= .02;
    meanValueW = 0;
    
    sv = initPose; %State vector initialization
    
    sigma = [0 0 0; 0 0 0; 0 0 0];
    
    Q = [0.00001 0 0; 0 0.0001 0; 0 0 0.0001];
    
    

 %%% SIMULATION SETUP
 
    % Sample time and time array
    sampleTime = 0.1;              % Sample time [s]
    tVec = 0:sampleTime:65;        % Time array
    r = rateControl(1/sampleTime);

    %Ideal Pose 
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
    
    % EKF
    
    %Prediction
    [sv, u] = sampleOdometry(pose(:,idx-1),pose(:,idx),sv);
    G = [ 1 0 -u(1)*sin(u(2));
          0 1  u(1)*cos(u(2));
          0 0         1       ];
    sigma = G*sigma*G.' + Q;
    %End prediction
    
    %Actualization
    for i = 1:size(detections,1)
        
        %%Real measurement
        z = [ sqrt(power(detections(i,1), 2) + power(detections(i,2), 2));
              normalize(atan2(detections(i,2), detections(i,1))) ;
              sv(3)];
        
       dist = power(objects(j,1) - sv(1), 2) + power(objects(j,2) - sv(2), 2); % Radicando de la de la distancia entre un landmark y el robot 
        
       
       
        for j = 1:size(objects,1)
            if objects(j,3) == detections(i,3)
                z_hat = [ sqrt(dist);
                  normalize(atan2(objects(j,2) - sv(2), objects(j,1) - sv(1) ) - sv(3)) ;
                  sv(3)];
                id_landmark = j;
            end
        end
        
        
        %%aqui vamos
        H = [ -( landmarks_on_map[id_index].point.x - x_(0) ) / sqrt(dist) , -( landmarks_on_map[id_index].point.y - x_(1) ) / sqrt(dist),0,
			 	  ( landmarks_on_map[id_index].point.y - x_(1) ) / dist, -( landmarks_on_map[id_index].point.x - x_(0) ) / dist,-1,
			 	  0,0,0];
	
    end
    
 
    
    waitfor(r);
    
end

hold on

%% show path with noise and ideal path 
plot(pose(1,:),pose(2,:));
plot(poseWithNoise(1,:),poseWithNoise(2,:));


------------------------------------------------------


function [xk, u] = sampleOdometry(o_1,o,x)
%SAMPLEODOMETRY Summary: Transform two poses in a comand
%   Detailed explanation goes here
% o_1 input parameter is the odometry pose in t-1 is a vector like [ x  y thetha ]  
% o input parameter is the odometry in time t is a vector like [ x  y thetha ]
% x is the state vector 

x_bar_p = o(1);
y_bar_p = o(2);
theta_bar_p = o(3);

x_bar = o_1(1);
y_bar = o_1(2);
theta_bar = o_1(3);

%% c an a just for debugging DETETE IT
%%c = normalizeAngle(3.14+1.57);
a = sqrt(power(x_bar - x_bar_p, 2) + power(y_bar - y_bar_p, 2));

if  a < .01  
    d_rot1 = 0; % Si solo gira  y este valor no es cero entonces  d_rot2 = - d_rot1 y el angulo final es practicamente el mismo  que el inicial :o alv
else
    d_rot1 = normalizeAngle(normalizeAngle(atan2(y_bar_p - y_bar, x_bar_p - x_bar )) - normalizeAngle(theta_bar)); %atan2(y_bar_p - y_bar, x_bar_p - x_bar ) - theta_bar;

d_trans1 = sqrt(  power(x_bar - x_bar_p, 2)  +  power(y_bar - y_bar_p, 2)  );
d_rot2 = normalizeAngle(normalizeAngle(theta_bar_p) - normalizeAngle(theta_bar + d_rot1)); %theta_bar_p - theta_bar - d_rot1;
    
d_rot1_hat =  d_rot1 ;%- pf_ran_gaussian( ALPHA[0] * pow(d_rot1,2) + ALPHA[1] * pow(d_trans1,2) );
d_trans1_hat = d_trans1 ;%- pf_ran_gaussian( ALPHA[2] * pow(d_trans1,2) + ALPHA[3] * pow(d_rot1,2) + ALPHA[3] * pow(d_rot2,2));
d_rot2_hat = d_rot2 ;%- pf_ran_gaussian(ALPHA[0] * pow(d_rot2,2) + ALPHA[1] * pow(d_trans1,2));
    
x_o(1) = x(1) + d_trans1_hat * cos( x(3) + d_rot1_hat );
x_o(2) = x(2) + d_trans1_hat * sin( x(3) + d_rot1_hat );


u = [ x(3)+d_rot1_hat d_trans1_hat ];
%*theta_plus_rotation1 = x_vector(2) + d_rot1_hat;
%*translation = d_trans1_hat;

x_o(3) = x(3) + d_rot1_hat + d_rot2_hat;

xk = [x_o(1) x_o(2) x_o(3)];
  
    
end



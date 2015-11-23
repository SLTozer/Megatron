close all
clc

N = 5;
robot = Robot();
map = [0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];

%% Calibrate
%robot.calibrateTurn(-2*pi, 1)

%% Time rotating scanner
% time = zeros(1,N);
% for i = 1:N          
%     deg = 360.0 - (360.0 / robot.scan_num);
%     % prep motor
%     if robot.scan_clockwise
%         power = 100;
%     else
%         power = -100;
%     end
%     motor = NXTMotor(MOTOR_C, 'Power', power, 'TachoLimit', deg, 'SmoothStart', true, 'ActionAtTachoLimit', 'brake');
%     motor.SendToNXT(); 
%     tic
%     motor.WaitFor();
%     time(i) = toc
%     robot.scan_clockwise = ~robot.scan_clockwise;  
% end
% plot(1:N, time)
% axis([1,N,0,max(time)])

%% Plot scanning results
% result = zeros(N,robot.scan_num);
% figure
% hold on
% plot([map(:,1);map(1,1)],[map(:,2);map(1,2)], 'r');
% scatter(20,20,'r')
% for i = 1:N
%     result(i,:) = robot.ultraScan()
%     Robot.plotScan([20, 20], pi/2, result(i,:));
% end
% result
% figure
% boxplot(result)

%% Run real test
robot = Robot();
target = [25, 45];
testRealbot(robot,map,target);

% 0.02 seconds per scan
% 1.33?
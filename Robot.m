% Class for controlling actual robot. Contains callibration routines but
% the constants hardcoded should be approximate. A new object closes all
% previous conections, which means only the most recent object will work
% (essentially singleton).
%
% move(cm) and turn(radians) work but curve(cm, radians, right) is a work
% in progress.
%
% There is no command to scan in any direction other than forwards at
% present.
%
% Follow command prompts after calling any of calibrate functions.
classdef Robot < handle
    properties
        % default guesses for constants (found using functions bellow)
        scan_constant = 1; % cm shift
        move_constant = 40; % degrees of motor / cm travelled
        turn_constant = 319 % degrees of motor / radians  travelled 
        turn_ultra_constant = 60; % degrees of scan motor / radians scanned
        axle_rad = 7; % cm
        scan_num = 10;
        
        % right and left motors to enable blocking
        right
        left
        
        scan_clockwise = true;
    end
    methods
        % constructor - opens connection to port (closes all previous)
        function obj = Robot()
            % location specific paths
            %addpath(genpath('\RWTHMindstormsNXT'))
            %addpath(genpath('\BotSimLib0.33'))
            %addpath(genpath('\libusb-win32-bin-1.2.6.0\lib'))
            %loadlibrary('libusb.lib')
            %open connection
            COM_CloseNXT all;  
            h = COM_OpenNXT();
            COM_SetDefaultNXT(h);
            OpenUltrasonic(SENSOR_1); %open usensor on port 4
            % set motors to empty call to allow correct blocking operaion
            obj.right = NXTMotor();
            obj.left = NXTMotor();
        end
        
        % automatically calibrate ultrasound. Dist is a vector of test
        % distances, N is the number of readings to take at each distance.
        function calibrateUltra(obj, dist, N)
            % initilise
            TAB = 9;
            result = zeros(1,N);
            fails = 0;
            R = size(dist,2);
            val_mean = zeros(1,R);
            val_std = zeros(1,R);
            val_raw_error = zeros(1,R);
            val_rel_error = zeros(1,R);

            for r = find(dist)
                d = dist(r);
                input(['Put me at ', num2str(d), ' cm from target, then press enter:']); 
                for i = 1:N
                    result(1,i) = GetUltrasonic(SENSOR_1);
                    % print progress bar
                    clc
                    fprintf('Scanning @ %d cm: %d %%\n', d, (i * 100) / N);
                end

                fails = fails + size(find(result == -1),2);
                val_mean(r) = mean(result);
                val_std(r) = std(result);
                val_raw_error(r) = val_mean(r) - d;
                val_rel_error(r) = val_raw_error(r) / d;
            end
            
            obj.scan_constant = mean(val_raw_error);

            % results
            clc
            disp(['Complete with ', num2str((fails * 100) / (R * N)), '% fails.']);
            disp(['Range', TAB,  num2str(dist)]);
            disp(['Mean', TAB, num2str(val_mean)]);
            disp(['Stdev', TAB, num2str(val_std)]);
            disp(['Raw Er', TAB, num2str(val_raw_error)]);
            disp(['Rel Er', TAB, num2str(val_rel_error)]);
            disp(['Calculated constant: ', num2str(obj.scan_constant), 'cm']);
        end
        
        function calibrateUltraTurn(obj, radians, N)
            % initialise
            actual = zeros(1,N);
            
            % take readings
            for i = 1:N
                input('Be prepared to measure how many radians I have turned. Press enter to begin:')
                % convert inputs
                deg = abs(round(radians * obj.turn_ultra_constant));
                obj.turnUltra(deg)
                actual(i) = input('Enter how many radians I have turned:');
            end
            
            % calculate new const and display
            obj.turn_ultra_constant = obj.turn_ultra_constant * radians / mean(actual);
            fprintf('Mean result %f radians.\n', mean(actual));
            fprintf('New ultrascan turn constant set as %f degrees(motor) / radians(robot).\n', obj.turn_ultra_constant);   
        end
        
        function calibrateTurn(obj, radians, N)
            % initialise
            actual = zeros(1,N);
            
            % take readings
            for i = 1:N
                input('Be prepared to measure how many radians I have turned. Press enter to begin:')
                obj.turn(radians)
                actual(i) = input('Enter how many radians I have turned:');
            end
            
            % calculate new const and display
            obj.turn_constant = obj.turn_constant * radians / mean(actual);
            fprintf('Mean result %f radians.\n', mean(actual));
            fprintf('New turn constant set as %f degrees(motor) / radians(robot).\n', obj.turn_constant);   
        end
        
        function calibrateMove(obj, dist, N)
            % initialise
            fprintf('Ensure there is %f cm clear ahead of me!\n', dist);
            actual = zeros(1,N);
            
            % take readings
            for i = 1:N
                input('Be prepared to measure how many far I have travelled. Press enter to begin:')
                obj.move(dist)
                actual(i) = input('Enter how many far I have travelled (cm):');
            end
            
            % calculate new const and display
            obj.move_constant = obj.move_constant * dist / mean(actual);
            fprintf('Mean result %f cm.\n', mean(actual));
            fprintf('New turn constant set as %f degrees(motor) / cm travelled(robot).\n', obj.move_constant);   
        end
                
        % dist: distance in cm to move robot forwards (zero is forever)
        % TODO rounds answer
        % TODO not blocking vs thread errors
        function move(obj, dist)
            % convert inputs
            deg = abs(round(dist * obj.move_constant));
            if dist > 0
                power = 100;
            else
                power = -100;
            end
            
            obj.startMotors(power, deg, power, deg)
        end
        
        % ang: ang in radians to turn clockwise (accepts negative)
        % TODO rounds answer
        % TODO not blocking vs thread errors
        function turn(obj, ang)
            % convert inputs
            deg = abs(round(ang * obj.turn_constant));
            if ang > 0
                rpower = -100;
                lpower = 100;
            else
                rpower = 100;
                lpower = -100;
            end
            
            obj.startMotors(rpower, deg, lpower, deg)
        end
        
        % NOT CORRECT AT ALL - doesnt take into account curvature of wheels
        function curve(obj, rad, ang, right)
            % convert inputs
            deg = abs(round(2 * rad * ang * obj.move_constant));
            if ang > 0
                power = 100;
            else
                power = -100;
            end
            
            ratio = (rad - obj.axle_rad) / (rad + obj.axle_rad)
            
            if right
                lpower = power
                ldeg = deg
                rpower = round(ratio * power)
                rdeg = round(ratio * deg)
            else
                rpower = power
                rdeg = deg
                lpower = round(ratio * power)
                ldeg = round(ratio * deg)
            end
            
            obj.startMotors(lpower, ldeg, rpower, rdeg)
        end
        
        function setScanNumber(obj, num)
            obj.scan_num = num;
        end
                
        function dist = ultraScan(obj)
            % convert inputs
            deg = abs(round(2 * pi * obj.turn_ultra_constant / obj.scan_num));
            dist = zeros(1, obj.scan_num);
            dist(1) = obj.scan_constant + GetUltrasonic(SENSOR_1);
            for i = 2:obj.scan_num
                turnUltra(obj, deg, obj.scan_clockwise)
                dist(i) = obj.scan_constant + GetUltrasonic(SENSOR_1);
            end
            if ~obj.scan_clockwise
                dist = fliplr(dist);
            end
            obj.scan_clockwise = ~obj.scan_clockwise;
        end
        
        function dist = fastUltraScan(obj)
            %turnUltra(obj, 2 * pi, obj.scan_clockwise)
            tic
            turnUltra(obj, 360, ~obj.scan_clockwise)
            toc
        end
        
        function turnUltra(~, deg, clockwise)
            % smoothstart and brake give better precision
            if clockwise
                power = 100;
            else
                power = -100;
            end
            motor = NXTMotor(MOTOR_C, 'Power', power, 'TachoLimit', deg, 'SmoothStart', true, 'ActionAtTachoLimit', 'brake');
            motor.SendToNXT();
            motor.WaitFor();
        end
        
        % wait for end of last commands (if any)
        function startMotors(obj, lpower, ldeg, rpower, rdeg)
            obj.right.WaitFor();
            obj.left.WaitFor();
            obj.right.Stop();
            obj.left.Stop();

            % smoothstart and brake give better precision
            obj.right = NXTMotor(MOTOR_A, 'Power', rpower, 'TachoLimit', rdeg, 'SmoothStart', true, 'ActionAtTachoLimit', 'brake');
            obj.left = NXTMotor(MOTOR_B, 'Power', lpower, 'TachoLimit', ldeg, 'SmoothStart', true, 'ActionAtTachoLimit', 'brake');
            obj.right.SendToNXT();
            obj.left.SendToNXT();
        end
    end
end

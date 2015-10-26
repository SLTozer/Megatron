%necassery paths
addpath(genpath('O:\Documents\Robotics\RWTHMindstormsNXTv4.07\RWTHMindstormsNXT'))
addpath(genpath('\\ads.bris.ac.uk\filestore\MyFiles\StudentUG14\jh12932\Documents\Robotics\BotSimLib0.35\BotSimLib0.35'))

% clearup
close all;
clear;   
clc;    

% basic calibration
robot = Robot();
%robot.calibrateUltra(10:10:50, 100);
robot.calibrateMove(30, 3);
robot.calibrateTurn(2 * pi, 3);
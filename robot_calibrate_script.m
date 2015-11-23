%necassery paths (CHANGE TO OWN)
addpath(genpath('\RWTHMindstormsNXT'))
addpath(genpath('\BotSimLib0.33'))
addpath(genpath('\libusb-win32-bin-1.2.6.0\lib'))
loadlibrary('libusb.lib')

close all;
clear;   
clc;    

% basic calibration
robot = Robot();
%robot.calibrateUltra(10:10:50, 100);
robot.calibrateMove(30, 3);
robot.calibrateTurn(2 * pi, 3);
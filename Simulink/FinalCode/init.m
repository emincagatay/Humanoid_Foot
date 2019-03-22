% parameters and initialisation for the human foot

clear all, close all, clc

SampleTime=0.01


% maximum value for PHI
PHI_t = deg2rad(15)

% scaling factor
scaling = 100;

T_st = 0.4*scaling; % standing still (PHI = 0°)
T_ta = 0.3*scaling; % take-off phase
T_sw = 1.1*scaling; % swing phase
T_la = 0.2*scaling; % landing phase




%PHI_t = 10

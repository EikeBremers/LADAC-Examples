
% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% add to path
addPathMinnie();
clc_clear;

%% initializing waypoint parameters
% Create waypoints

waypoints = [1 -1 -0.2; 2 0 -0.1; 1 1 -0.1; -1 -1 -0.1; -2 0 -0.1; -1 1 -0.2]'*50;
cycle = true;
% degree of polynomial
degree = 5;

% Create emptry trajectory with a maximum space for 10 waypoints
traj_size = size(waypoints,2);
[traj_empty] = trajInit(traj_size, degree);

% Create simulink bus definition
trajectoryBus = struct2bus_(traj_empty);

% compute trajectory
traj = trajFromWaypoints(traj_empty, waypoints, degree, cycle);


%% wrong name workaround
%section = sections;
%trajectoryBus.Elements(3).DataType = "section";
%clear sections;
%

%% load physical copter parameters
copter = copterLoadParams( 'copter_params_Minnie' );

%% environment parameters
envir = envirLoadParams('params_envir','envir',0);

%% controller parameters
% load parameters
fm_Traj = fmCopterLoiterIndiLoadParams( 'fmCopterLoiterIndi_params_Minnie' );

%adding waypoint parameters

%% joystick
jystck = joystickLoadParams( 'joystick_params_Xbox_360', 2, 0 );

%% initial conditions (IC)
% load initial conditions
IC = loadInitialConditionsParams( 'params_initial_conditions_hover' );

% initial motor angular velocity, in rad/s
IC.omega_mot = [ 1; 1; 1; 1 ] * 843;

%% load ground parameters (grnd)
grnd = groundLoadParams( 'params_ground_default' );

%% reference position lat, lon, alt
% initial latitude, in deg
pos_ref.lat = 37.6117;
% initial longitude, in deg
pos_ref.lon = -122.37822;
% initial altitude, in m
pos_ref.alt = 10;

%% Flight Gear settings for UDP connection
% Flight Gear URL
fg.remoteURL = '127.0.0.1';
% fdm receive port of Flight Gear
fg.remotePort = 5502;

%% Set pacer parameters
pacer.pace = 1;
pacer.sample_time = 1/100;

%% Open Simulink model
open_model('QuadcopterSimModel_INDI_Traj')
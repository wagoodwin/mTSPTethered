% limitTest.m

% Objective: base script to push limits of timed method


%% Pre-Setup

% Before running, ensure you have runTimedMethod.m on your MATLAB PATH.
% Also, bring in eil51_nodecoordinates.txt to your PATH so you can generate
% node coordinates

%% Setup

clc
clearvars

% SIM BASIC PARAMETERS ---------------------------------
numOfCities = 5;
tetherLength = 70;
flagNorm = "2_norm";
numOfTimeSteps = 8;
flagTether = "Tethered"; % options: "Tethered" or "Untethered"
numOfRobots = 3;
timeLimitTimedMethod = 1800; % in seconds
% -----------------------------------------------------



% GENERATE NODE COORDINATES ----------------------------

% To load coordinates from a file, use this:
nodecoords = load('eil51_nodecoords.txt');
nodecoords = nodecoords(1:numOfCities,:);

% To generate coordinates, randomly, do this:
% random scaling factor for "box" size
intervalLength = tetherLength/(sqrt(2));
sz = [numOfCities 2];
randomCoords = ...
[(1:1:numOfCities)', round(unifrnd(0,intervalLength, sz))];
% unifrnd() samples numbers from a uniform distribution
% round() sets every number to its respective nearest integer (we enforce
% that cities have integer coordinates)

% -------------------------------------------------------------


%% Timed Method Function Call

% ASSIGN NODE COORDINATES

% Choose if you want random coordinates or TSPLIB coordinates
% coords = nodecoords;
coords = randomCoords; 



% RUN SIMULATION --------------------------------------------------

[soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible] = ...
    runTimedMethod(coords,numOfCities, ...
    numOfRobots, numOfTimeSteps, flagTether,tetherLength,flagNorm,timeLimitTimedMethod);

% --------------------------------------------------------------------









%% Setup
clearvars
numRuns = 50;
numOfRobots = 3;
numOfTimeSteps = 8;
flagTether = "Tethered"; % options: "Tethered" or "Untethered"
tetherLength = 50;
flagNorm = "2-norm";     % options: "1-norm", "2-norm"
flagIterativeNew = 1;    % for Naive Method
timeLimit = 300; % in seconds, so Naive Method gets 5 min before truncating

numOfCities = 5;
% Generate node coordinates
% From a file:
% nodecoords = load('TruncatedEil51NodeCoords.txt'); 
% nodecoords = nodecoords(1:numOfCities,:);
% Randomly:
% random scaling factor for "box" size
boxScale = tetherLength/sqrt(tetherLength^2 + tetherLength^2);
sz = [numOfCities 2];

%% troubleshooting

objs1 = [];
objs2 = [];

for j = 1:50

    rng(j,'twister');
    % So note that we use seeds 1 through 10. That gives us 10 different
    % sets of random coordinates that we can individually replicate using
    % the corresponding seed.

    randomCoords = ...
    [(1:1:numOfCities)', round(unifrnd(0,tetherLength*boxScale, sz))];
    % unifrnd() samples numbers from a uniform distribution
    % round() sets every number to its respective nearest integer

    [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible,routeMatrix] = ...
        runNaiveMethod(randomCoords,numOfCities, flagTether, ...
        tetherLength,flagNorm,flagIterativeNew,timeLimit);

    objs1 = [objs1, value(objectiveMinMax)];


    [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible] = ...
        runTimedMethod(randomCoords,numOfCities, ...
        numOfRobots, numOfTimeSteps, flagTether,tetherLength,flagNorm);

    objs2 = [objs2, value(objectiveMinMax)];


end

%% testing naive updates

% clearvars
% 
% numRuns = 50;
% numOfRobots = 3;
% numOfTimeSteps = 8;
% flagTether = "Tethered"; % options: "Tethered" or "Untethered"
% tetherLength = 50;
% flagNorm = "2-norm";     % options: "1-norm", "2-norm"
% flagIterativeNew = 1;    % for Naive Method
% 
% numOfCities = 5;
% % Generate node coordinates
% % From a file:
% % nodecoords = load('TruncatedEil51NodeCoords.txt'); 
% % nodecoords = nodecoords(1:numOfCities,:);
% % Randomly:
% % random scaling factor for "box" size
% boxScale = tetherLength/sqrt(tetherLength^2 + tetherLength^2); 
% sz = [numOfCities 2];
% timeLimit = 30; %seconds
% 
% rng(1,'twister');
% randomCoords = ...
% [(1:1:numOfCities)', round(unifrnd(0,tetherLength*boxScale, sz))];
% 
% 
% randomCoords = load('TruncatedEil51NodeCoords.txt'); 
% randomCoords = randomCoords(1:numOfCities,:);
% 
% [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible, routeMatrix] = ...
%     runNaiveMethod(randomCoords,numOfCities, flagTether, ...
%     tetherLength,flagNorm,flagIterativeNew,timeLimit);

%% Run Loop, Timed


allRunInfo = cell(1,numRuns);

for j = 1:numRuns
    
    % Initialize a constant seed. The 'seed' determines how the computer will
    % generate random numbers. By setting the seed constant here, we will get
    % the same "random" numbers each run. We're doing this maneuver to keep
    % some consistency for testing.
    rng(j,'twister');
    % So note that we use seeds 1 through 10. That gives us 10 different
    % sets of random coordinates that we can individually replicate using
    % the corresponding seed.
    
    randomCoords = ...
    [(1:1:numOfCities)', round(unifrnd(0,tetherLength*boxScale, sz))];
    % unifrnd() samples numbers from a uniform distribution
    % round() sets every number to its respective nearest integer
    
    [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible] = ...
        runTimedMethod(randomCoords,numOfCities, ...
        numOfRobots, numOfTimeSteps, flagTether,tetherLength,flagNorm);

    % We store the solution, agent distances, objective function value, given
    % coordinates, tether length, and norm type for each run. To distinguish
    % between runs, we put all of this information, for each run, in its own
    % cell array. So for n runs, we have n cell arrays, and each cell array has
    % all that information.

    % To store all of these cell arrays, we put them in a larger cell array.

    valSol            = value(soln);
    valRobotDistances = value(robotDistances);
    valObj            = value(objectiveMinMax);

    runTitles = {'valSol', 'valRobotDistances', 'valObj', 'randomCoords', ...
        'tetherLength', 'flagNorm', 'numOfTimeSteps', 'flagIsFeasible', 'totTime'};

    runData = {valSol, valRobotDistances, valObj, randomCoords, ...
        tetherLength, flagNorm, numOfTimeSteps, flagIsFeasible, totTime};
    
    runInfo = cell(2,numel(runTitles));
    
    % Assigning titles and and respective data for that run
    for i = 1:numel(runTitles)
        runInfo{1,i} = runTitles{i}; runInfo{2,i} = runData{i};
    end
    
    % Put all the information for this single run into a slot for all of
    % the run information
    allRunInfo{j} = runInfo;

    save("/home/walter/Repositories/mTSPAlgorithms/MTSPTethered/mTSPTethered/randomRunsTimed70.mat", ...
        "allRunInfo")
    
%     % Save figure for that run as well:
%     folderName = '/home/walter/Repositories/mTSPAlgorithms/MTSPTethered/mTSPTethered/randomRuns';
%     figName = sprintf('Run%d.jpg',j);
%     fullFileName = fullfile(folderName,figName);
%     exportgraphics(axes, fullFileName, 'Resolution', '1000');
%     Not doing plotting because it's gonna take a while to make work and
%     I'm lazy. But if we need plots, I can make them with the output data.


end


% Statistics

% We want means, medians, stdevs, mins, and maxes for objective values and
% computation times.

objValsData   = zeros(1,numel(runTitles));
compTimesData = zeros(1,numel(runTitles));

for i = 1:numel(runTitles)
    objValsData(i)    = allRunInfo{i}{2,3};
    compTimesData(i)  = allRunInfo{i}{2,9};
end

objVals.mean   = mean(objValsData);
objVals.median = median(objValsData);
objVals.stdev  = std(objValsData);
objVals.min    = min(objValsData);
objVals.max    = max(objValsData);

compTimes.mean   = mean(compTimesData);
compTimes.median = median(compTimesData);
compTimes.stdev  = std(compTimesData);
compTimes.min    = min(compTimesData);
compTimes.max    = max(compTimesData);


%% Run Naive Method Loop

numRuns = 50;

allRunInfoNaive = cell(1,numRuns);

for j = 1:numRuns % doing 10  runs arbitrarily
    
    
    % Initialize a constant seed. The 'seed' determines how the computer will
    % generate random numbers. By setting the seed constant here, we will get
    % the same "random" numbers each run. We're doing this maneuver to keep
    % some consistency for testing.
    rng(j,'twister');
    % So note that we use seeds 1 through 10. That gives us 10 different
    % sets of random coordinates that we can individually replicate using
    % the corresponding seed.
    
    randomCoords = ...
    [(1:1:numOfCities)', round(unifrnd(0,tetherLength*boxScale, sz))];
    % unifrnd() samples numbers from a uniform distribution
    % round() sets every number to its respective nearest integer
    
    [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible,routeMatrix] = ...
        runNaiveMethod(randomCoords,numOfCities, flagTether, ...
        tetherLength,flagNorm,flagIterativeNew,timeLimit);

    % We store the solution, agent distances, objective function value, given
    % coordinates, tether length, and norm type for each run. To distinguish
    % between runs, we put all of this information, for each run, in its own
    % cell array. So for n runs, we have n cell arrays, and each cell array has
    % all that information.

    % To store all of these cell arrays, we put them in a larger cell array.

    valSol            = value(soln);
    valRobotDistances = value(robotDistances);
    valObj            = value(objectiveMinMax);

    runTitles = {'valSol', 'valRobotDistances', 'valObj', 'randomCoords', ...
        'tetherLength', 'flagNorm', 'numOfTimeSteps', 'flagIsFeasible', 'totTime', 'timeLimit'};

    runData = {valSol, valRobotDistances, valObj, randomCoords, ...
        tetherLength, flagNorm, numOfTimeSteps, flagIsFeasible, totTime, timeLimit};
    
    runInfo = cell(2,numel(runTitles));
    
    % Assigning titles and and respective data for that run
    for i = 1:numel(runTitles)
        runInfo{1,i} = runTitles{i}; runInfo{2,i} = runData{i};
    end
    
    % Put all the information for this single run into a slot for all of
    % the run information
    allRunInfoNaive{j} = runInfo;
% 
%     save("/home/walter/Repositories/mTSPAlgorithms/MTSPTethered/mTSPTethered/randomRunsNaive50.mat", ...
%         "allRunInfoNaive")
    
%     % Save figure for that run as well:
%     folderName = '/home/walter/Repositories/mTSPAlgorithms/MTSPTethered/mTSPTethered/randomRuns';
%     figName = sprintf('Run%d.jpg',j);
%     fullFileName = fullfile(folderName,figName);
%     exportgraphics(axes, fullFileName, 'Resolution', '1000');
%     Not doing plotting because it's gonna take a while to make work and
%     I'm lazy. But if we need plots, I can make them with the output data.


end

% Statistics

% We want means, medians, stdevs, mins, and maxes for objective values and
% computation times.

objValsDataNaive   = zeros(1,numRuns);
compTimesDataNaive = zeros(1,numRuns);

for i = 1:numRuns
    objValsDataNaive(i)    = allRunInfoNaive{i}{2,3};
    compTimesDataNaive(i)  = allRunInfoNaive{i}{2,9};
end

objValsNaive.mean   = mean(objValsDataNaive);
objValsNaive.median = median(objValsDataNaive);
objValsNaive.stdev  = std(objValsDataNaive);
objValsNaive.min    = min(objValsDataNaive);
objValsNaive.max    = max(objValsDataNaive);

compTimesNaive.mean   = mean(compTimesDataNaive);
compTimesNaive.median = median(compTimesDataNaive);
compTimesNaive.stdev  = std(compTimesDataNaive);
compTimesNaive.min    = min(compTimesDataNaive);
compTimesNaive.max    = max(compTimesDataNaive);








%% Setup
clearvars
numRuns = 25;
numOfRobots = 3;
numOfTimeSteps = 8;
flagTether = "Tethered"; % options: "Tethered" or "Untethered"
tetherLength = 70;
flagNorm = "1_norm";   % options: "1_norm", "2_norm"
flagIterativeNew = 1;    % for Naive Method

% Default time limit for Naive Method before truncation using a 'smarter'
% version near Naive Method function call
timeLimitTimedMethod = 1800; % in seconds

numOfCities = 10;
% Generate node coordinates
% From a file:
% We use these node coordinates for a few sets of runs. Most runs are with
% random sets of coordinates, but these random sets make computation time
% way longer, and so it makes the method look worse than it is compared to
% other tsp solvers. So we also throw in some runs with established coords
% that other methods use.
nodecoords = load('TruncatedEil51NodeCoords.txt');
nodecoords = nodecoords(1:numOfCities,:);
% Randomly:
% random scaling factor for "box" size
intervalLength = tetherLength/(sqrt(2));
sz = [numOfCities 2];

% Idea: create a bunch of sets of random coordinates. If all of the
% coordinates have distances that are the tether length or less, keep it.
% If they aren't, redo. In the report, we can say we uniformly sampled the
% coordinates, discarding all infeasible ones

% Run timed method first, get all the rns for that. Look at the
% staticstics, ande see how long it's taking typically, and then set a time
% limit to be around that average time. Maybe a bit longer.

% Ideally, the more we can run, the better. Do anything you can

% After you save the data, clear as much as you can. Save them as .mat
% files, delete as much as you can, and run it again. do a yalmip 'clear'
% and wipe out any other data you get. if Yong did it, he'd clear all
% basically

% Universal path where all data will go
path = "C:\Users\alexg\OneDrive\Documents\MATLAB" + ...
        "\Repositories\mTSPTethered\allRunInfo\";
    
% Make directory based on setup information:
mkdir(path + ...
sprintf("TSPCOORDS_Cities%d_Tether%d_Runs%d_%s",numOfCities, tetherLength, + ...
numRuns, flagNorm) + "\" )

%% test runs

%     randomCoords = ...
%     [(1:1:numOfCities)', round(unifrnd(0,intervalLength, sz))];
%     % unifrnd() samples numbers from a uniform distribution
%     % round() sets every number to its respective nearest integer
% 
%     [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible,routeMatrix] = ...
%         runNaiveMethod(randomCoords,numOfCities, flagTether, ...
%         tetherLength,flagNorm,flagIterativeNew,timeLimit);
% 
%     [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible] = ...
%         runTimedMethod(randomCoords,numOfCities, ...
%         numOfRobots, numOfTimeSteps, flagTether,tetherLength,flagNorm);


%% troubleshooting
% 
% objs1 = [];
% objs2 = [];
% 
% numOfCities = 8;
% intervalLength = tetherLength/(sqrt(2));
% sz = [numOfCities 2];
% 
% for j = 1:1
% 
%     rng(j,'twister');
%     % So note that we use seeds 1 through 10. That gives us 10 different
%     % sets of random coordinates that we can individually replicate using
%     % the corresponding seed.
% 
%     randomCoords = ...
%     [(1:1:numOfCities)', round(unifrnd(0,intervalLength, sz))];
%     % unifrnd() samples numbers from a uniform distribution
%     % round() sets every number to its respective nearest integer
% 
% %     [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible,routeMatrix] = ...
% %         runNaiveMethod(randomCoords,numOfCities, flagTether, ...
% %         tetherLength,flagNorm,flagIterativeNew,timeLimit);
% % 
% %     objs1 = [objs1, value(objectiveMinMax)];
% 
% 
%     [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible] = ...
%         runTimedMethod(randomCoords,numOfCities, ...
%         numOfRobots, numOfTimeSteps, flagTether,tetherLength,flagNorm,timeLimitTimedMethod);
% 
%     objs2 = [objs2, value(objectiveMinMax)];
% 
% 
% end
% 
% % Plan: run just 1 sim each for naive and timed with random coords and see
% % how it long it's taking. Then run old stuff to see how long that is
% % taking
% 
% % nodecoords = load('TruncatedEil51NodeCoords.txt');
% % nodecoords = nodecoords(1:numOfCities,:);
% % randomCoords = nodecoords;
% 
% % One thing to note is that most tsp papers use 
% % coordinates from tsplip-- they're using coordinates designed
% % for these problems. We're using absolutely unreasonable and random 
% % setups of coordinates, which is probably affecting computation time


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
    [(1:1:numOfCities)', round(unifrnd(0,intervalLength, sz))];
    coords = randomCoords;
    coords = nodecoords;
    % unifrnd() samples numbers from a uniform distribution
    % round() sets every number to its respective nearest integer
    
    [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible] = ...
        runTimedMethod(coords,numOfCities, ...
        numOfRobots, numOfTimeSteps, flagTether,tetherLength,flagNorm,timeLimitTimedMethod);

    % We store the solution, agent distances, objective function value, given
    % coordinates, tether length, and norm type for each run. To distinguish
    % between runs, we put all of this information, for each run, in its own
    % cell array. So for n runs, we have n cell arrays, and each cell array has
    % all that information.

    % To store all of these cell arrays, we put them in a larger cell array.

    valSol            = value(soln);
    valRobotDistances = value(robotDistances);
    valObj            = value(objectiveMinMax);

    runTitles = {'valSol', 'valRobotDistances', 'valObj', 'coords', ...
        'tetherLength', 'flagNorm', 'numOfTimeSteps', 'flagIsFeasible', 'totTime'};

    runData = {valSol, valRobotDistances, valObj, coords, ...
        tetherLength, flagNorm, numOfTimeSteps, flagIsFeasible, totTime};
    
    runInfo = cell(2,numel(runTitles));
    
    % Assigning titles and and respective data for that run
    for i = 1:numel(runTitles)
        runInfo{1,i} = runTitles{i}; runInfo{2,i} = runData{i};
    end
    
    % Put all the information for this single run into a slot for all of
    % the run information
    allRunInfo{j} = runInfo;

    %save("/home/walter/Repositories/mTSPAlgorithms/MTSPTethered/mTSPTethered/randomRunsTimed70.mat", ...
        %"allRunInfo")

    fullpath = path + sprintf("TSPCOORDS_Cities%d_Tether%d_Runs%d_%s",numOfCities, + ...
        tetherLength, numRuns, flagNorm) + "\";

    save(fullpath + sprintf("randomRunsTimed%d.mat",tetherLength), "allRunInfo")

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

for i = 1:numRuns
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

% And finally, save these computation times in a .mat file:
save(fullpath + "objValsTimed.mat", "objVals");
save(fullpath + "compTimesTimed.mat", "compTimes");

%% Run Naive Method Loop

allRunInfoNaive = cell(1,numRuns);
% Set computation time to be the median Timed Method computation time times
% 1.5, but only if the timed method computation times are less than 1800s.
% If the timed method computation times are longer, then the assumption
% that the timed method is quick is gone, so just set this time limit to be
% the same:
if (compTimes.median <= timeLimitTimedMethod)
    timeLimit = compTimes.median*1.5;
else
    timeLimit = timedLimitTimedMethod;
end

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
    [(1:1:numOfCities)', round(unifrnd(0,intervalLength, sz))];
    coords = randomCoords;
    coords = nodecoords;
    % unifrnd() samples numbers from a uniform distribution
    % round() sets every number to its respective nearest integer
    
    [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible,routeMatrix] = ...
        runNaiveMethod(coords,numOfCities, flagTether, ...
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

    runTitles = {'valSol', 'valRobotDistances', 'valObj', 'coords', ...
        'tetherLength', 'flagNorm', 'numOfTimeSteps', 'flagIsFeasible', 'totTime', 'timeLimit'};

    runData = {valSol, valRobotDistances, valObj, coords, ...
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

    
    fullpath = path + sprintf("TSPCOORDS_Cities%d_Tether%d_Runs%d_%s",numOfCities, + ...
        tetherLength, numRuns, flagNorm) + "\";

    save(fullpath + sprintf("randomRunsNaive%d.mat",tetherLength), "allRunInfo")
    
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

save(fullpath + "objValsNaive.mat", "objValsNaive");
save(fullpath + "compTimesNaive.mat", "compTimesNaive");

%%

%% tether length unifrom sampling interval proof
% 
% numOfCities = 5;
% tetherLength = 50;
% % sz tells unifrnd how many numbers to make. [numOfCities 2] means, 
% % "make numOfCities rows and 2 columns." That way we can get a set of
% % coordinates that are random and formatted like a list of points.
% sz = [numOfCities 2]; 
% lengths = [];
% % Here's the intervalLength finding from the proof (the variable T)
% intervalLength = tetherLength/(sqrt(2));
% 
% 
% for j = 1:1000
%     
%     % Use rng(j,'twister') ensures we don't get the same set of 
%     % random coordinates for all 1000 runs. Now, we get different
%     % coordinates, and they're dictated by the jth seed.
%     rng(j,'twister');
% 
%     randomCoords = ...
%         [(1:1:numOfCities)', round(unifrnd(0,intervalLength, sz))];
% 
%     C = createDistanceMatrix(5,randomCoords);
%     % get the largest distance out of the distance matrix
%     maxLength = max(max(C));
%     
%     % Check to see if any of the distances are larger than the 
%     % tether length
%     if maxLength >= tetherLength
%         lengths = [lengths, maxLength];
%         
%     end
%     
% end
% 
% lengths

%% Utility Functions

function C = createDistanceMatrix(numOfCities,nodecoords)

C = zeros(numOfCities);

for i = 1:numOfCities
    for j = 1:numOfCities   
             C(i,j) = distance(nodecoords(i,2), nodecoords(i,3), ...
             nodecoords(j,2), nodecoords(j,3));
    end
end

C = real(C);

end


function [d] = distance(x1,y1,x2,y2)
    d = sqrt( (y2 - y1)^2 + (x2 - x1)^2 );
end


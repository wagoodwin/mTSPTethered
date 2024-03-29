function [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible] = ...
    runTimedMethod(nodecoords,numOfCities, ...
    numOfRobots, numOfTimeSteps, flagTether,tetherLength,flagNorm, timeLimit)


clc; 
clearvars -except nodecoords numOfCities numOfRobots numOfTimeSteps ...
    flagTether tetherLength flagNorm timeLimit;
close all;
yalmip('clear');

%% mTSP Tethers, Reformulated
% Walter Goodwin

% To change between norms, you need to 
% 1. Change the distance function when building the C matrix
% 2. Change the tether constraint
% 3. Change the objective function


% flagNorm = "2_norm"; % options: "1_norm", "2_norm Squared", "2_norm"
% flagTether = "Tethered"; % options: "Tethered" or "Untethered"


%% Setup and Easy Constraints

% numOfTimeSteps corresponds to i
% numOfCities    corresponds to j
% numOfRobots    corresponds to k

% Sometimes, the node coordinates will be set up such that the problem
% can't be solved with the given tether length (i.e., cities are too far
% away). In practice, I actually haven't seen it yet, as if the tether
% length is too short, the robots will visit all of the same cities in the 
% same order (like "holding hands"). But in the case that problem cannot
% be solve, for this reason or others, the model is rendered infeasible.
% flagIsFeasible documents this fact. It's set to 1 (feasible) by default
% and is set to 0 through an exception at the solver call.
flagIsFeasible = 1;


% tetherLength = 50;

% Truncated eil51 node coords further to just 5 cities
% nodecoords = load('ToyProblemNodeCoords.txt');
% nodecoords = load('TruncatedEil51NodeCoords.txt');
% nodecoords = nodecoords(1:numOfCities,:);
% nodecoords = [(1:4)' randi(300,4,2)];

C = zeros(numOfCities);

for i = 1:numOfCities
    for j = 1:numOfCities   
        
        if( (strcmp(flagNorm,"1_norm") == 1) )
            C(i,j) = distance1(nodecoords(i,2), nodecoords(i,3), ...
            nodecoords(j,2), nodecoords(j,3));
        end
        
        if ( (strcmp(flagNorm,"2_norm Squared") == 1) || ...
                (strcmp(flagNorm,"2_norm") == 1) )
             C(i,j) = distance(nodecoords(i,2), nodecoords(i,3), ...
             nodecoords(j,2), nodecoords(j,3));
        end
        
    end
end
  
C = real(C); 


% Question: how do we know how many rows the simulation is gonna be?
% That is, how do we know how many time steps there are gonna be?
% For now, we have to mandate that the robots visit a node and then
% leave after the next time step, I think. 
% The main option might be to make the number of rows larger certain.
% Like make a rule of thumb where numOfTimeSteps = 2*numOfCities. 


% mTSP constraints, reformulated

x = binvar(numOfTimeSteps,numOfCities,numOfRobots,'full');

% Constraints 1 and 2 (starting and ending node constraint)
% Robots must start and end at node 1
constraint1 = [];
constraint2 = [];

for k = 1:numOfRobots % Robots start and end at node 1
    constraint1 = [constraint1, x(1,1,k) == 1];
    constraint2 = [constraint2, x(numOfTimeSteps,1,k) == 1];
end

% Constraint 3 (visitation constraint)
% Each node must be visited at least once
constraint3 = [];
for j = 1:numOfCities
    xijkTotal = 0;
    for k = 1:numOfRobots
        for i = 1:numOfTimeSteps
            
            xijkTotal = xijkTotal + x(i,j,k);
            
        end
    end
    constraint3 = [constraint3, (xijkTotal >= 1)];
end
xijkTotal = 0;

% Constraint 4 
% Ensure any robot is not at more than one node at once.
% i.e. one robot cannot be at multiple locations simulatenously
constraint4 = [];
for i = 1:numOfTimeSteps
    for k = 1:numOfRobots
            xijkTotal = 0;
        for j = 1:numOfCities
            xijkTotal = xijkTotal + x(i,j,k);
        end
        constraint4 = [constraint4, (xijkTotal == 1)];
    end
end
xijkTotal = 0;

% Constraints 5 and 6 (tether constraints between r1,r2 and r2,r3)
constraint5 = [];
constraint6 = [];
for i = 1:numOfTimeSteps
    % between robot 1 and 2:
    constraint5 = [constraint5, (x(i,:,1)*C*(x(i,:,2)') <= tetherLength)];
    % between robot 2 and 3:
    constraint6 = [constraint6, (x(i,:,2)*C*(x(i,:,3)') <= tetherLength)];
end


%% Objective Function and Tether Constraint: p-norm method

% We make a variable called robotLocation for each robot. This
% variable stores the node coordinate location of the robot at each time
% step. We can then compare the distance between the robots at each time
% step by subtracting these vectors at each time step and taking the norm. 

tetherConstraints = [];
robotDistances = [];
temp = [];

robotDistTraveled = []; % for objective function


% I think we'll need to split this into separate loops.
% Loop 1: get all robot locations
% Loop 2: 1 and 2_norm implementations of tether constraints



% Loop 1: get all robot locations
for k = 1:numOfRobots

    for i = 1:numOfTimeSteps
        
        % Very weird thing with YALMIP: Assigning things to
        % RobotLocation(i,:) works but not with RoobotLocation(i,:,k). I'm
        % guessing YALMIP doesn't support multi-dim arrays in these cases.
        temp0(i,:) = sum([x(i,:,k)' x(i,:,k)'].*nodecoords(:,(2:3)),1);
        
        
        % ANOTHER NOTE: assignment is very weird with YALMIP. 
        % temp(i,:) = stuff   will not give an sdpvar, but 
        % temp = stuff        will. 
        
    end
    
    % multi-dimensional matrices don't allow for assignment with YALMIP, 
    % but cell arrays do:
    robotLocation{k} =  temp0;

end



% Gets the distance between the k and k-1 robot at each step
% in time for all robots. So for k = 2, we'll get the distances
% between robots 1 and 2 over a period of time. Store it in this
% temp variable, and then we'll load that into our robotDistances
% variable.

% Example on the indexing: robotLocation{k}(i,:) grabs the kth
% robot's position data, and looks at only the ith row.

% So at the end of the i loop, temp will be a numOfTimeSteps x 1
% vector. Each entry says the distance between robots k-1 and k.

for k=2:numOfRobots
    for i = 1:numOfTimeSteps
                
        if (strcmp(flagNorm,"1_norm") == 1)  
            temp = [temp; ...
                    norm( (robotLocation{k-1}(i,:)-robotLocation{k}(i,:)),1) ];
%                     sum(abs(robotLocation{k-1}(i,:)-robotLocation{k}(i,:)))];
        end
        
        if (strcmp(flagNorm,"2_norm Squared") == 1)
            temp = [temp; ...
            (norm( (robotLocation{k-1}(i,:)-robotLocation{k}(i,:)),2))^2 ];
%             sum((robotLocation{k-1}(i,:)-robotLocation{k}(i,:)).^2)]; 
        end
        
        if (strcmp(flagNorm,"2_norm") == 1)
            temp = [temp; ...
                    norm( (robotLocation{k-1}(i,:)-robotLocation{k}(i,:)),2) ];
%                               sqrt(sum((robotLocation{k-1}(i,:)-robotLocation{k}(i,:)).^2)) ];
        end
    end
    robotDistances = [robotDistances, temp];
    temp = [];
end




% Now build the objective function variable:
for k = 1:numOfRobots
    temp = 0;
    for i = 2:numOfTimeSteps
        
        % one-norm implementation
        if (strcmp(flagNorm,"1_norm") == 1) 
            temp = temp + ...
            sum(abs(robotLocation{k}(i-1,:)-robotLocation{k}(i,:)));
%             norm( (robotLocation{k}(i-1,:)-robotLocation{k}(i,:)),1 );
        end
        
        % squared two-norm implementation
        if (strcmp(flagNorm,"2_norm Squared") == 1)
            temp = temp + ...
            sum((robotLocation{k}(i-1,:)-robotLocation{k}(i,:)).^2);            
        end
        
        % squared two-norm implementation
        if (strcmp(flagNorm,"2_norm") == 1)
            temp = temp + ...
                              norm( (robotLocation{k}(i-1,:)-robotLocation{k}(i,:)),2 );
%                               sqrt(sum((robotLocation{k}(i-1,:)-robotLocation{k}(i,:)).^2));




        end
        
    end
    robotDistTraveled = [robotDistTraveled; temp];
end

    
% Enforce tether constraints. We need every entry to be less than
% tetherLength, so an element-wise inequality works here.
for i = 1:numOfTimeSteps
    for j = 1:(numOfRobots-1)
        
        if (strcmp(flagNorm,"1_norm") == 1 || strcmp(flagNorm,"2_norm") == 1)
            tetherConstraints = [tetherConstraints, ...
            (robotDistances(i,j) <= tetherLength)];     
        end
        
        if (strcmp(flagNorm,"2_norm Squared") == 1)      
            tetherConstraints = [tetherConstraints, ...
            (robotDistances(i,j) <= tetherLength^2)];
        end
        
    end
end


% Build objectives:
% objective = distRobot1+distRobot2+distRobot3;%sum(distRobot);% sum(sum(bound));
% objectiveMinMax = max([distRobot1 distRobot2 distRobot3]);


% Pro-Tip from Dr. Yong: add in a tiny contribution of a MinSum term to the
% objective function. That'll ensure we're still mostly focused on the
% MinMax objective, but it'll also sprinkle in a bit to minimize the total
% distance traveled. It works.
% objectiveMinMax = 0.999*max(robotDistTraveled) + ...
%     0.001*sum(robotDistTraveled); 
objectiveMinMax = max(robotDistTraveled);
% objectiveMinMax = max([distRobot1 distRobot2 distRobot3]);


 

%% Run the Simulation


% All constraints:
constraints = [constraint1, constraint2, constraint3, constraint4, ...
];

% Set time limit from function argument:
opts = sdpsettings;
opts.gurobi.TimeLimit = timeLimit;

if strcmp(flagTether,"Tethered") == 1
    constraints = [constraints, tetherConstraints];
end

% Solve system
% options = sdpsettings('verbose',1,'solver','Gurobi');
tic;
try
    sol = optimize(constraints,objectiveMinMax, opts);
catch
    disp("The problem is infeasible");
    flagIsFeasible = 0;
end
sol.info;
totTime = toc;
if (toc >= timeLimit)
    totTime = timeLimit;
    disp("Simulation exceeded time limit of " + num2str(timeLimit) + ...
        " s. returning feasible, but not necessarily optimal, solution")
else
    totTime = toc;
end

% get total distance traveled by all robots:
totalDistanceSum = 0;
for k = 1:numOfRobots
    for i = 1:numOfTimeSteps-1
        totalDistanceSum = totalDistanceSum + value(x(i,:,k))*C*value(x(i+1,:,k))';
    end
end

totalDistanceSum
value(objectiveMinMax)
value(x)

soln = value(x);

totalDistanceInd = zeros(numOfRobots,1);
for k = 1:numOfRobots
    for i = 1:numOfTimeSteps-1
        totalDistanceInd(k) = totalDistanceInd(k) + value(x(i,:,k))*C*value(x(i+1,:,k))';
    end
end

totalDistanceInd;







%% Plotting
clc; clear gca;

figure(1)

% Plot the robots' tours:
hold on
box on
% To get unique colors for each robot's path, use colors from a colormap.
% Initiate that colormap here:
cmap = cool(numOfRobots);
cmapTethers = hsv(numOfRobots);
for k = 1:numOfRobots
    
    plotRoute(nodecoords, int32(value(x(:,:,k))), numOfTimeSteps, ...
        numOfCities, cmap(k,:), flagNorm, flagTether)

end

% Plot the tethers between the robots:

if (strcmp(flagTether,"Tethered"))
    
    for k = 2:numOfRobots
           
        plotTethers(numOfRobots, numOfTimeSteps, numOfCities, ... 
        nodecoords, int32(value(x)),k-1,k, cmapTethers(k,:), '--')
        
    end 
    
end

hold off

% Note: In this formulation, we always have
% the tether between robots 1 and 2 and robots 1 and 3, but that's 
% arbitrary because the computer can't tell the difference between which
% robot is which in the formulation. We could've done 1T2 and 2T3, and 
% we would get the same result.

% Bold latex axes from this source:
% https://www.mathworks.com/matlabcentral
% /answers/406685-ticklabelinterpreter-axis-ticks-bold


% Set text interpreter to Latex and save figure:
set(0,'defaulttextinterpreter','latex');

axes = gca;

axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.TickLabelFormat  = '\\textbf{%g}';

axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.TickLabelFormat = '\\textbf{%g}';

% exportgraphics(axes, ...
%     '/home/walter/Desktop/ThesisFigures/Timed/Tether70/Timing_70teth_2norm.jpg', ...
%     'Resolution', '1000')


%% Utility Functions


% DISTANCE FUNCTION
% Finds the Euclidean norm between two points
function [d] = distance(x1,y1,x2,y2)
    d = sqrt( (y2 - y1)^2 + (x2 - x1)^2 );
end

% Finds the 1_norm between two points
function [d1] = distance1(x1,y1,x2,y2)
    d1 = abs(x2 - x1) + abs(y2 - y1);
end

% PLOTTING FUNCTION

% nothing to return here
function[] = plotRoute(nodecoords, xBinvar, ...
                       numOfTimeSteps, numOfCities, cmapCol, flagNorm, flagTether) 
    % for xBinvar, just enter a single slice of the x matrix-- i.e., 
    % value(x(:,:,1)).
    

    % first plot just the node coordinates by themselves. Code influenced
    % by the "I want to create a plot using XY coordintes" post on 
    % the MathWorks forums.
    xPoints = nodecoords(:,2);
    yPoints = nodecoords(:,3);
    n = numel(xPoints);
    plot(graph(1:n,1:n), 'LineStyle', 'none', 'Marker', 'd', ...
       'NodeColor','black','XData',xPoints,'YData',yPoints);
   
   % Then take the binvar matrix and put it in terms of row and column
   % indices:
   
   colIndex = zeros(1,numOfTimeSteps);
   rowIndex = zeros(1,numOfTimeSteps);
   
   for i = 1:numOfTimeSteps
       for j = 1:numOfCities
           if xBinvar(i,j) == 1
               % Situation is here. Need a way to convert
               % that 1 value in the matrix to an index from
               % 1:numOfCities. Trying with linear indices now.
               colIndex(i) = j;
               rowIndex(i) = j;
           end
       end
   end
   
   % colIndex is defined to not have the first entry and rowIndex is 
   % defined to not have the last entry, so implement that rule here:
   colIndex = colIndex(1,2:end);
   rowIndex = rowIndex(1,1:end-1);
   
   

    for i = 1:length(rowIndex)
      
        point1 = [nodecoords(rowIndex(i),2) nodecoords(rowIndex(i),3)];
        point2 = [nodecoords(colIndex(i),2) nodecoords(colIndex(i),3)];
        
        % One way to plot the points:
        % Code here taken from "How can I draw a line with arrow head
        % between 2 data points in a plot" off MathWorks forums
%         dp = point2 - point1;
%         quiver(point1(1),point1(2),dp(1),dp(2),string(color))
        
        % An alternative way to do it. Doesn't have arrows but looks
        % cleaner:
 
        dp = point2 - point1;
        quiver(point1(1),point1(2),dp(1),dp(2), 'Color', cmapCol)
%         set(h1,'AutoScale','on', 'AutoScaleFactor', 0.07)
        % x components of point 1 and 2, 
        % then y components of point 1 and 2:
        plot([point1(1) point2(1)], [point1(2) point2(2)], 'Color', cmapCol)
        
        hold on
        pause(0.1);
        drawnow
    end

    % No title for IROS paper
%     title("Robot Tours: Timing Formulation, " + ...
%         num2str(flagTether) + ", " + num2str(flagNorm));

    xlabel("\textbf{$$\mathbf{x}$$ distance (arbitrary units)}", ...
        'fontweight','bold','FontSize',16);
    ylabel("\textbf{$$\mathbf{y}$$ distance (arbitrary units)}",'fontweight', ...
        'bold','FontSize',16);
    
    delta = 5;
    xlim([min(nodecoords(:,2))-delta,max(nodecoords(:,2))+delta])
    ylim([min(nodecoords(:,3))-delta/3,max(nodecoords(:,3))+delta/3])


end

% No outputs (void return type). Inputs: robot tours and the two
% robots you want.
function [] = plotTethers(numOfRobots, numOfTimeSteps, numOfCities, ... 
              nodecoords, totalxBinvar, r1, r2, cmapCol, lineStyle)

    
                      
    % Build the RobotColIdxs and RobotRowIdxs arrays. Same idea as in
    % route plotting function but with three robots:
    
   RobotColIdxs = [];
   RobotRowIdxs = [];
   
    for k = 1:numOfRobots
       for i = 1:numOfTimeSteps
           for j = 1:numOfCities
               if totalxBinvar(i,j,k) == 1
                   % Situation is here. Need a way to convert
                   % that 1 value in the matrix to an index from
                   % 1:numOfCities. Trying with linear indices now.
                   RobotColIdxs(1,i,k) = j;
                   RobotRowIdxs(1,i,k) = j;
               end
           end
       end
    end
   
    % Reformat RobotColIdxs and RobotRowIdxs in the same way we did
    % before (removing and adding ones):
    
    RobotColIdxs = RobotColIdxs(:,2:end,:);
    RobotRowIdxs = RobotRowIdxs(:,1:end-1,:);
                      
                      
    % Plot tethers. That is, we want to plot a line 
    % between some two robots at every point in time. In this case,
    % we'd want to show the length between robots 1 and 2 and 
    % robots 1 and 3 at every time step.
    for i = 1:length(RobotRowIdxs(1,:,1))

        point1 = [nodecoords(RobotColIdxs(1,i,r1),2) ...
                  nodecoords(RobotColIdxs(1,i,r1),3)];
        point2 = [nodecoords(RobotColIdxs(1,i,r2),2) ...
                  nodecoords(RobotColIdxs(1,i,r2),3)];
              
%         point3 = [nodecoords(RobotRowIdxs(1,i,r2),2) ...
%                   nodecoords(RobotRowIdxs(1,i,r2),3)];
%         point4 = [nodecoords(RobotColIdxs(1,i,r2),2) ...
%                   nodecoords(RobotColIdxs(1,i,r2),3)];

%         tetherPoint1 = point4 - point2;
%         tetherPoint2 = point3 - point1;

        tetherPoint1 = point1;
        tetherPoint2 = point2;


        plot([tetherPoint1(1) tetherPoint2(1)], ...
             [tetherPoint1(2) tetherPoint2(2)], 'Color', ...
              cmapCol, 'LineStyle', char(lineStyle), ...
              'LineWidth', 1.5)

        hold on
        pause(0.1);
        drawnow

    end
end

%%

end


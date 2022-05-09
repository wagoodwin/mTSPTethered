
%% mTSP Tethers, Reformulated
% Walter Goodwin

% To change between norms, you need to 
% 1. Change the distance function when building the C matrix
% 2. Change the tether constraint
% 3. Change the objective function


%% Setup and Easy Constraints

clc; 
clear all;
yalmip('clear');

numOfTimeSteps = 8; % corresponds to i
numOfCities = 10; % corresponds to j
numOfRobots = 3; % corresponds to k

tetherLength = 60;

% Truncated eil51 node coords further to just 5 cities
% nodecoords = load('ToyProblemNodeCoords.txt');
nodecoords = load('TruncatedEil51NodeCoords.txt');
nodecoords = nodecoords(1:10,:);
% nodecoords = [(1:4)' randi(300,4,2)];

C = zeros(numOfCities);

for i = 1:numOfCities
    for j = 1:numOfCities   
        C(i,j) = distance(nodecoords(i,2), nodecoords(i,3), ...
           nodecoords(j,2), nodecoords(j,3));
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

% x11kTotal = 0;
% xn1kTotal = 0;
% 
% for k = 1:numOfRobots
%     x11kTotal = x11kTotal + x(1,1,k);
%     xn1kTotal = xn1kTotal + x(numOfTimeSteps,1,k);
% end
% 
% constraint1 = [(x11kTotal == 1)];
% constraint2 = [(xn1kTotal == 1)];


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

% Update tether constraint stuff to use bounds and stuff like the 
% objective function 


% tetherConstraints = 
%  

%% Objective Function: Entry Selection Method (Abolish)

% Objective Function
totalDistance = 0;
for k = 1:numOfRobots
    for i = numOfCities - 1
            totalDistance = totalDistance +  x(i+1,:,k)*C*x(i,:,k)';
    end
end
objective = totalDistance;


%% Objective Function and Tether Constraint: p-norm method

% We'll now implement the tether constraint. We'll use the taxicabDistances
% matrix to enforce that the distance between the robots at every point 
% in time should be less than a set tether length.




tetherConstraints = [];
% robotLocation=zeros(numOfTimeSteps,2,numOfRobots);
for i = 1:numOfTimeSteps
    
%     robotDistances12 = [];
    tetherConstraint = zeros(1,2);
%     robotLocation=zeros(numOfRobots,2);
%     for k = 1:numOfRobots
% %         for j = 1:numOfCities
%             robotLocation(i,:,k) = sum([x(i,:,k)' x(i,:,k)'].*nodecoords(:,(2:3)),1);%x(i,j,k)*nodecoords(j,(2:3));
%             
% %             % Enforce tether constraint between robots 1 and 2:
% %             robotDistances12 = x(i,j,1)*nodecoords(j,(2:3)) - ...
% %                 x(i,j,2)*nodecoords(j,(2:3));
% %             
% %             % Enforce tether constraint between robots 1 and 3:
% %             robotDistances13 = x(i,j,1)*nodecoords(j,(2:3)) - ...
% %                 x(i,j,3)*nodecoords(j,(2:3));
%             
% %         end
%     end

    % At each time step, 
    robotLocation1(i,:) = sum([x(i,:,1)' x(i,:,1)'].*nodecoords(:,(2:3)),1);
    robotLocation2(i,:) = sum([x(i,:,2)' x(i,:,2)'].*nodecoords(:,(2:3)),1);
    robotLocation3(i,:) = sum([x(i,:,3)' x(i,:,3)'].*nodecoords(:,(2:3)),1);%x(i,j,k)*nodecoords(j,(2:3));

    % 1-norm
    robotDistances12 = sum(abs(robotLocation1(i,:)-robotLocation2(i,:)));
    robotDistances13 = sum(abs(robotLocation1(i,:)-robotLocation3(i,:)));
    tetherConstraints = [tetherConstraints; ...
                             (robotDistances12 <= tetherLength); ...
                             (robotDistances13 <= tetherLength)];
    % 2-norm
%     robotDistances12 = sum((robotLocation1(i,:)-robotLocation2(i,:)).^2);
%     robotDistances13 = sum((robotLocation1(i,:)-robotLocation3(i,:)).^2);
%     tetherConstraints = [tetherConstraints; ...
%                              (robotDistances12 <= tetherLength^2); ...
%                              (robotDistances13 <= tetherLength^2)];
                         
                         
    % Actual 2-norm (including square root)
%     robotDistances12 = sqrtm(sum((robotLocation1(i,:)-robotLocation2(i,:)).^2));
%     robotDistances13 = sqrtm(sum((robotLocation1(i,:)-robotLocation3(i,:)).^2));
%     tetherConstraints = [tetherConstraints; ...
%                              (robotDistances12 <= tetherLength); ...
%                              (robotDistances13 <= tetherLength)];
           
end


% Objective Function


% Idea: we want to minimize the total distance traveled by all of the
% robots. The total distance traveled is the sum of the absolute 
% values of the robots' distances between each other.

% So we want to sum the absolute values of a bunch of distances. However,
% The function abs(x) is not linear so it's not super nice and doesn't
% guarantee our feasible region to be convex. So we want to use YALMIP's
% "bound" method to keep the objective function linear.

% "Bound" method: define a bound sdpvar object that will constrain the
% the robot distances. We then minimize that bound object as opposed to
% the distances.




% This loop makes a matrix of distances between the robots at each timestep.
% The matrix has numOfTimeStep-1 rows and 2 columns. The first column
% holds the x distance at a step and the second column holds the y distance
% at that step.

% The distances are found by taking the vector difference between the
% robots at each time step.

% Note! this matrix of distances doesn't actually have distances in it
% because we don't know where the robots are going to go. Rather, it's
% and sdpvar object that symbolically represents the path traveled
% by the robots in their route.


    % Initialize D (taxicabDistances) matrix
    taxicabDistances = []; 
     

distRobot1=0;
distRobot2=0;
distRobot3=0;
% for k = 1:numOfRobots
    
    for i = 1:numOfTimeSteps-1

% 2-norm
%         distRobot1=distRobot1+(sum((robotLocation1(i,:)-robotLocation1(i+1,:)).^2));
%         distRobot2=distRobot2+(sum((robotLocation2(i,:)-robotLocation2(i+1,:)).^2));
%         distRobot3=distRobot3+(sum((robotLocation3(i,:)-robotLocation3(i+1,:)).^2));

% 2-norm actual (square root)
%         distRobot1=distRobot1+sqrtm((sum((robotLocation1(i,:)-robotLocation1(i+1,:)).^2)));
%         distRobot2=distRobot2+sqrtm((sum((robotLocation2(i,:)-robotLocation2(i+1,:)).^2)));
%         distRobot3=distRobot3+sqrtm((sum((robotLocation3(i,:)-robotLocation3(i+1,:)).^2)));
        
% 1-norm
        distRobot1=distRobot1+(sum(abs(robotLocation1(i,:)-robotLocation1(i+1,:))));
        distRobot2=distRobot2+(sum(abs(robotLocation2(i,:)-robotLocation2(i+1,:))));
        distRobot3=distRobot3+(sum(abs(robotLocation3(i,:)-robotLocation3(i+1,:))));
        
        % To get 2-norm: 
        % distRobot1=distRobot1+(sum((robotLocation1(i,:)-robotLocation1(i+1,:)).^2));
        
        % (re)set dCoord variable
%         dCoord = zeros(1,2);
        % distance coordinate calculation at a single time step:
%         for j = 1:numOfCities
%             dCoord = dCoord + (x(i,j,k)*nodecoords(j,2:3)) - ... 
%                 (x(i+1,j,k)*nodecoords(j,2:3));
%         end
        %taxicabDistances(i,,k) = dCoord;  % assign dCoord vector to ith entry 
        
%         taxicabDistances = [taxicabDistances; dCoord(1) dCoord(2)];
        % when we assign, we get a NaN. instead of sdpvar for
        % taxicabDistances. Gotta assign one at a time

    end
    
% end





% create a bound object to be minimzed. It's a 3-dimensional array, with
% one dimension per robot.
% Each dimension has a (numOfTimeSteps-1)x(2) matrix. The first column 
% bounds the x distances of taxicanDistances from above and below, while
% the second column bounds the y distances.

% bound = sdpvar(length(taxicabDistances(:,1)),2);
% 
% objConstraints = [];
%     for j = 1:2 % go through the x and then the y distances
%         objConstraints = [objConstraints,  ...
%             (-bound(:,j) <= taxicabDistances(:,j) <= bound(:,j))];
%     end

% objConstraints = [];
% for i = 1:numOfRobots*2
%     objConstraints = [objConstraints,  ...
%         (-bound(:,i) <= taxicabDistances(:,i) <= bound(:,i))];
% end

% odd columns have robot x coords; even columns have robot y coords

% 
% constraint1 = [-bound(:,1) <= taxicabDistances(:,1) <= bound(:,1)];
% constraint2 = [-bound(:,2) <= taxicabDistances(:,2) <= bound(:,2)];
% constraint2 = [-bound(:,3) <= taxicabDistances(:,3) <= bound(:,3)];
% 
% 
% constraint1 = [-bound1 <= taxicabDistances(:,1) <= bound1];
% constraint2 = [-bound2 <= taxicabDistances(:,2) <= bound2];
% 
% robotOneBound = sdpvar(length(taxicabDistances(:,1)),2);
% objConstraint = [-robotOneBound(:,1) <= taxicabDistances(:,1) <= robotOneBound(:,1)];



% bound has three dimensions, so we need three sum() commands
objective = distRobot1+distRobot2+distRobot3;%sum(distRobot);% sum(sum(bound));
objectiveMinMax = max([distRobot1 distRobot2 distRobot3]);

% 

%% Run the Simulation



% All constraints:
constraints = [constraint1, constraint2, constraint3, constraint4, ...
];%,objConstraints];

% Solve system
% options = sdpsettings('verbose',1,'solver','Gurobi');
tic;
sol = optimize(constraints,objectiveMinMax);
sol.info;
toc;

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

totalDistanceInd = zeros(numOfRobots,1);
for k = 1:numOfRobots
    for i = 1:numOfTimeSteps-1
        totalDistanceInd(k) = totalDistanceInd(k) + value(x(i,:,k))*C*value(x(i+1,:,k))';
    end
end

totalDistanceInd

%% Plotting

figure(1)

% Plot the robots' tours:
hold on
box on
plotRoute(nodecoords, int32(value(x(:,:,1))), numOfTimeSteps, numOfCities, 'red')
plotRoute(nodecoords, int32(value(x(:,:,2))), numOfTimeSteps, numOfCities, 'blue')
plotRoute(nodecoords, int32(value(x(:,:,3))), numOfTimeSteps, numOfCities, 'magenta')

% Plot the tethers between the robots:
% 
% plotTethers(numOfRobots, numOfTimeSteps, numOfCities, ... 
%     nodecoords, int32(value(x)),1,2, 'black', ':')
% plotTethers(numOfRobots, numOfTimeSteps, numOfCities, ... 
%     nodecoords, int32(value(x)), 1,3, 'black', '--')  
hold off

% Note: In this formulation, we always have
% the tether between robots 1 and 2 and robots 1 and 3, but that's 
% arbitrary because the computer can't tell the difference between which
% robot is which in the formulation. We could've down 1T2 and 2T3, and 
% we would get the same result.

ax = gca
exportgraphics(ax, ...
    '/home/walter/Desktop/ThesisFigures/Timing_Untethered_1norm.jpg', 'Resolution', '1000')
% 

%% Utility Functions


% DISTANCE FUNCTION
% Finds the Euclidean norm between two points
function [d] = distance(x1,y1,x2,y2)
    d = sqrt( (y2 - y1)^2 + (x2 - x1)^2 );
end

% Finds the 1-norm between two points
function [d1] = distance1(x1,y1,x2,y2)
    d1 = abs(x2 - x1) + abs(y2 - y1);
end

% PLOTTING FUNCTION

% nothing to return here
function[] = plotRoute(nodecoords, xBinvar, ...
                       numOfTimeSteps, numOfCities, color) 
    % for xBinvar, just enter a single slice of the x matrix-- i.e., 
    % value(x(:,:,1)).
    

    % first plot just the node coordinates by themselves. Code influenced
    % by the "I want to create a plot using XY coordintes" post on 
    % the MathWorks forums.
    x = nodecoords(:,2);
    y = nodecoords(:,3);
    n = numel(x);
    plot(graph(1:n,1:n), 'LineStyle', 'none', 'Marker', 'd', ...
       'NodeColor','black','XData',x,'YData',y);
   
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
        quiver(point1(1),point1(2),dp(1),dp(2),string(color))
%         set(h1,'AutoScale','on', 'AutoScaleFactor', 0.07)
        % x components of point 1 and 2, 
        % then y components of point 1 and 2:
        plot([point1(1) point2(1)], [point1(2) point2(2)], string(color))
        
        hold on
        pause(0.1);
        drawnow
    end
    
%     title("Robot Tours: 60 Unit Tether, MinMax Objective");
%     title("Robot Tours: 70 Unit Tether, MinSum Objective, 1-norm");
    title("Robot Tours: Timing Formulation, Untethered, 1-norm");
    xlabel("x distance (arbitrary units)");
    ylabel("y distance (arbitrary units)");


end

% No outputs (void return type). Inputs: robot tours and the two
% robots you want.
function [] = plotTethers(numOfRobots, numOfTimeSteps, numOfCities, ... 
              nodecoords, totalxBinvar, r1, r2, color, lineStyle)

    
                      
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
             [tetherPoint1(2) tetherPoint2(2)], 'color', ...
              string(color), 'LineStyle', char(lineStyle), ...
              'LineWidth', 1.5)

        hold on
        pause(0.1);
        drawnow

    end
end


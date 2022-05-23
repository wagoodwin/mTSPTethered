
%% mTSP Tethers, Reformulated
% Walter Goodwin

% To change between norms, you need to 
% 1. Change the distance function when building the C matrix
% 2. Change the tether constraint
% 3. Change the objective function

%{

    Current issue: how do we generalize the algo to any number of robots?

    Here are the details: right now, we have to build a new tether
    constraint every time we want to enforce tethers between 2 robots. We
    want a way that lets us enforce tethers between robots without adding
    more code. 

    


%}


clc; 
clear all;
yalmip('clear');


flagNorm = "one"; % options: "one" or "two-squared"
flagTether = "tethered"; % options: "tethered" or "untethered"


%% Setup and Easy Constraints

numOfTimeSteps = 8; % corresponds to i
numOfCities = 10; % corresponds to j
numOfRobots = 4; % corresponds to k

tetherLength = 60;

% Truncated eil51 node coords further to just 5 cities
% nodecoords = load('ToyProblemNodeCoords.txt');
nodecoords = load('TruncatedEil51NodeCoords.txt');
nodecoords = nodecoords(1:numOfCities,:);
% nodecoords = [(1:4)' randi(300,4,2)];

C = zeros(numOfCities);

for i = 1:numOfCities
    for j = 1:numOfCities   
        
        if( (strcmp(flagNorm,"one") == 1) )
            C(i,j) = distance1(nodecoords(i,2), nodecoords(i,3), ...
            nodecoords(j,2), nodecoords(j,3));
        end
        
        if ( (strcmp(flagNorm,"two-squared") == 1) )
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

%% Objective Function and Tether Constraint: p-norm method

% Idea: we make a variable called robotLocation for each robot. This
% variable stores the node coordinate location of the robot at each time
% step. We can then compare the distance between the robots at each time
% step by subtracting these vectors at each time step and taking the norm. 

tetherConstraints = [];
robotDistances = [];
temp = [];

robotDistTraveled = []; % for objective function


% I think we'll need to split this into separate loops.
% Loop 1: get all robot locations
% Loop 2: 1 and 2-norm implementations of tether constraints



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




% Loop 2: set up 1- and 2-norm versions of tether constraints



        
if (strcmp(flagNorm,"one") == 1) 

    for k = 2:numOfRobots

        for i = 1:numOfTimeSteps

        % Gets the distance between the k and k-1 robot at each step
        % in time for all robots. So for k = 2, we'll get the distances
        % between robots 1 and 2 over a period of time. Store it in this
        % temp variable, and then we'll load that into our robotDistances
        % variable.

        % Example on the indexing: robotLocation{k}(i,:) grabs the kth
        % robot's position data, and looks at only the ith row.
        temp = [temp; ...
        sum(abs(robotLocation{k-1}(i,:)-robotLocation{k}(i,:)))];

        % So at the end of the i loop, temp will be a numOfTimeSteps x 1
        % vector. Each entry says the distance between robots k-1 and k.

    

        
        end
        
        robotDistances = [robotDistances, temp];
        temp = [];

    end
    


end

    
    

if (strcmp(flagNorm,"two-squared") == 1)

    for i = 1:numOfTimeSteps

    % 2-norm code would go here


    end

end
           



% Now build objective function variables:
distcheck = 0;
    
    % 1-norm implementation
if (strcmp(flagNorm,"one") == 1)
    temp = 0;

    for k = 1:numOfRobots

        for i = 2:numOfTimeSteps

            % Store the distance traveled by the kth robot in a temp
            % variable. Allows us to build the robotDistTraveled vector.
            % That vector will hold how far each robot has traveled total
            % in each entry.   
%             distcheck = distcheck + sum(abs(robotLocation(i-1,:,1)-robotLocation(i,:,1)));
            
            % Gives us total distance traveled by the kth robot
            temp = temp + ...
                sum(abs(robotLocation{k}(i-1,:)-robotLocation{k}(i,:)));
            
        end

    robotDistTraveled = [robotDistTraveled; temp];
    temp = 0;

    end
   
    

end


    
    
% two-squared norm implementation
if (strcmp(flagNorm,"two-squared") == 1)

    for i = 1:numOfTimeSteps

    end


end
    
        
    
%{    
    
    
    
%     for i = 1:numOfTimeSteps
% 
% %         % At each time step, 
% %         robotLocation1(i,:) = sum([x(i,:,1)' x(i,:,1)'].*nodecoords(:,(2:3)),1);
% %         robotLocation2(i,:) = sum([x(i,:,2)' x(i,:,2)'].*nodecoords(:,(2:3)),1);
% %         robotLocation3(i,:) = sum([x(i,:,3)' x(i,:,3)'].*nodecoords(:,(2:3)),1);
% 
%         % Generalize robotLocation as a multi-dim array:
%         
% %         temp = sum([x(i,:,k)' x(i,:,k)'].*nodecoords(:,(2:3)),1);
% %         robotLocation(i,:,k) = temp;
% %         robotLocation(i,:,k) = sum([x(i,:,k)' x(i,:,k)'].*nodecoords(:,(2:3)),1);
% 
%         % 1-norm implementation
%         if (strcmp(flagNorm,"one") == 1) 
%             
%             robotDistances = [robotDistances;  ...
%                 sum(abs(robotLocation(i,:,k)-robotLocation(i,:,k+1)));
% 
%     %         robotDistances12 = sum(abs(robotLocation1(i,:)-robotLocation2(i,:)));
%     %         robotDistances13 = sum(abs(robotLocation1(i,:)-robotLocation3(i,:)));
% 
%         end
% 
% 
%         % Squared 2-norm implementation (sum of squared distances)  
%         if (strcmp(flagNorm,"two-squared") == 1)
% 
%             robotDistances12 = sum((robotLocation1(i,:)-robotLocation2(i,:)).^2);
%             robotDistances13 = sum((robotLocation1(i,:)-robotLocation3(i,:)).^2);
%             tetherConstraints = [tetherConstraints; ...
%                                      (robotDistances12 <= tetherLength^2); ...
%                                      (robotDistances13 <= tetherLength^2)];
% 
%         end
% 
% 
%     end



% % Now loop the tether constraint over all robots:
% for k = 1:numOfRobots
%     tetherConstraints = [tetherConstraints; ...
%         robotDistances(k) <= tetherLength];
% %         tetherConstraints = [tetherConstraints; ...
% %                                  (robotDistances12 <= tetherLength); ...
% %                                  (robotDistances13 <= tetherLength)];
% end

% Idea: to enforce the objective function, we need
% each robot's distance traveled. We use robotLocation to get each robot's
% displacement vector, and then we take the norm of the difference in
% robots' individual displacement vectors over time to get the the robot
% distances. We end up with total distances traveled for each robot, and we
% use these distances for our objective function.







% Initialize D (taxicabDistances) matrix
% taxicabDistances = []; 
%      
% % Initialize robot distances:
% distRobot1=0;
% distRobot2=0;
% distRobot3=0;
% 
% % Initialize distRobot vector:
% distRobots = zeros(numOfRobots,1);

% 
% % for k = 1:numOfRobots
%     
%     for i = 1:numOfTimeSteps-1
% 
%     % 1-norm implementation
%     if (strcmp(flagNorm,"one") == 1) 
% % 
% %         distRobot1=distRobot1+(sum(abs(robotLocation1(i,:)-robotLocation1(i+1,:))));
% %         distRobot2=distRobot2+(sum(abs(robotLocation2(i,:)-robotLocation2(i+1,:))));
% %         distRobot3=distRobot3+(sum(abs(robotLocation3(i,:)-robotLocation3(i+1,:))));
%         
%         for k = 1:numOfRobots
%             distRobots(k) = distRobots(k) + ...
%                 (sum(abs(robotLocation(i,:,k)-robotLocation(i+1,:,k))));
%         end
%         
%     end
%     
%     % Squared 2-norm implementation (sum of squared distances) 
%     if (strcmp(flagNorm,"two-squared") == 1)
% 
%         distRobot1=distRobot1+(sum((robotLocation1(i,:)-robotLocation1(i+1,:)).^2));
%         distRobot2=distRobot2+(sum((robotLocation2(i,:)-robotLocation2(i+1,:)).^2));
%         distRobot3=distRobot3+(sum((robotLocation3(i,:)-robotLocation3(i+1,:)).^2));
%          
%     end
%         
% 
%     end
    
%}
    
% Enforce tether constraints. We need every entry to be less than
% tetherLength, so an element-wise inequality works here.

% THIS IS DOING ITS JOB, SO I'M GUESSING ROBOTDISTANCES ISN'T
% HOLDING THE RIGHT DISTANCES. SO EITHER ROBOTDISTANCES IS WRONG OR
% ROBOTLOCATIONS IS WRONG

for i = 1:numOfTimeSteps
    for j = 1:(numOfRobots-1)
        tetherConstraints = [tetherConstraints, ...
            (robotDistances(i,j) <= tetherLength)];
    end
end


% SO HERE'S WHAT MAKES NO SENSE. IT LOOKS LIKE THE CONSTRAINTS ARE WORKING.
% I CHECKED THE DISTANCES BETWEEN ROBOTS 1 AND 2 AND ROBOTS 2 AND 3, AND
% THEY'RE ALL LESS THAN 60 (THE TETHERS ARE CURRENTLY PLOTTED BETWEEN THE
% WRONG ROBOTS, AND WE'LL ALSO HAVE TO GENERALIZE THOSE AFTER THIS).

% SO WE'RE SOMEHOW GETTING A SLIGHTLY WORSE RESULT THAT BEFORE. WE STILL
% GET THE SAME OBJECTIVE FUNCTION VALUE. THAT MAKES ME THINK IT'S AGAIN
% SOME ISSUE WITH THE TETHER CONSTRAINT. THESE TETHER CONSTRAINTS ARE
% TIGHTENING THE FEASIBLE REGION MORE THAN THHEY SHOULD, IOW. REALLY WEIRD.
% 



% Build objectives:
% objective = distRobot1+distRobot2+distRobot3;%sum(distRobot);% sum(sum(bound));
% objectiveMinMax = max([distRobot1 distRobot2 distRobot3]);

objectiveMinMax = max(robotDistTraveled); % SEEMS TO BE WORKING
% objectiveMinMax = max([distRobot1 distRobot2 distRobot3]);

 

%% Run the Simulation


% All constraints:
constraints = [constraint1, constraint2, constraint3, constraint4, ...
];

if strcmp(flagTether,"tethered") == 1
    constraints = [constraints, tetherConstraints];
end

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

if (strcmp(flagTether,"tethered"))

    plotTethers(numOfRobots, numOfTimeSteps, numOfCities, ... 
        nodecoords, int32(value(x)),1,2, 'black', ':')
    plotTethers(numOfRobots, numOfTimeSteps, numOfCities, ... 
        nodecoords, int32(value(x)), 2,3, 'black', '--')  
    
end

hold off

% Note: In this formulation, we always have
% the tether between robots 1 and 2 and robots 1 and 3, but that's 
% arbitrary because the computer can't tell the difference between which
% robot is which in the formulation. We could've done 1T2 and 2T3, and 
% we would get the same result.

% For saving the figure locally:
% ax = gca
% exportgraphics(ax, ...
%     '/home/walter/Desktop/ThesisFigures/Timing_Untethered_1norm.jpg', 'Resolution', '1000')
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


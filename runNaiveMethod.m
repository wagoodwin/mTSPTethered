function [soln, robotDistances, totTime, objectiveMinMax, flagIsFeasible, routeMatrix] = ...
    runNaiveMethod(nodecoords,numOfCities, flagTether, ...
    tetherLength,flagNorm, flagIterativeNew, timeLimit) 

% Background

%{
We want to take the mTSP formulation and incorporate constraints that
represent tethers
We've got two routes here.
ROUTE 1: Brute force
Make a toy example (2 robots, 3 or 4 cities) and add a bunch of tether
constraints like we talked about in our meeting. Run it and observe
the results
RESULT 2: Also Brute force, but different
Run the whole simulation and then check to see if a tether constraint was
violated. If a constraint was violated, exclude that solution, add a 
constraint that prevents that solution from happening, and resolve. 
The constraint would involve just saying x(i,j,1) and x(i,j,2) can't 
both be 1 at the same time.
In this case, we'd also make a toy problem to speed things up. 2 robots
and 5 cities.
To change between norms, just change the distance function in the 
% cost matrix.
%}


%% Setup

% TO RUN: Press "Run" in the MATLAB editor

% clc; 
clearvars -except nodecoords numOfCities ...
    flagTether tetherLength flagNorm flagIterativeNew timeLimit;
close all;
yalmip('clear');

flagIsFeasible = 1; % default this flag to 1
% If flagIterative == 1, the iterative method will run. Otherwise, only the
% original simulation will run.

% flagIterativeNew = 1;
flagIterative = 0;
flagSolveBaseProblem = 0;

% If flagTether == 1, the plotting tool will include tethers. 
% flagTether = 1;
% Note that if you turn on flagIterative, you should probably turn on
% flagTether, as the Iterative Method is the version of the sim that
% implements the tether constraints.
% flagNorm = "2_norm"; % options: "2_norm", "1_norm"

numOfRobots = 3;
% numOfCities = 5; %
% tetherLength = 70;
% Distance value used for dummy node creation. Ensures that our 'zeroth'
% node has zero cost from 0 to node 1 but doesn't go to other nodes
% during optimization.
M = 10e4; % Tried M = inf, but YALMIP poops itself.

% Set that represents all nodes but the first. Used for generating a set of
% all possible subtours with Dantzig-Fulkerson-Johsnon (DFJ) subtour
% eliminiation constraints (SECs).
VminusFirst =  num2cell(2:numOfCities);
V0 = num2cell(1:numOfCities); % set of nodes that includes dummy node

% Initialize bin for SECs before solving problem
% So first get all possible subtours:
S = PowerSet(VminusFirst);
% Ensure the cardinality of the subsets lays between 2 and (numOfCities-1)
% S = S(2:end-1);
S = S(2:end);
bin = binvar(numel(S),numOfRobots,'full'); 
assign(bin,ones(numel(S),numOfRobots))

% Initialize sdpvar object for use in constraints 3,4. Will be 
% included in objective function. Traversal: any entrance or exit
numNodeTraversals = sdpvar(numOfCities, numOfRobots, 'full');
x = intvar(numOfCities, numOfCities, numOfRobots, 'full');
u = sdpvar(numOfCities, 1, 'full');
p = numOfCities - numOfRobots;

% Truncated eil51 node coords further to just 5 cities
% nodecoords = load('ToyProblemNodeCoords.txt');
% nodecoords = load('TruncatedEil51NodeCoords.txt');
% nodecoords = nodecoords(1:numOfCities,:);

C = zeros(numOfCities);

for i = 1:numOfCities
    for j = 1:numOfCities   
        
        if( (strcmp(flagNorm,"1_norm") == 1) )
            C(i,j) = distance1(nodecoords(i,2), nodecoords(i,3), ...
            nodecoords(j,2), nodecoords(j,3));
        end
        
        if ( (strcmp(flagNorm,"2_norm") == 1) )
             C(i,j) = distance(nodecoords(i,2), nodecoords(i,3), ...
             nodecoords(j,2), nodecoords(j,3));
        end
        
    end
end
  
C = real(C); 



%% DEBUG
% 
% x = zeros(numOfCities,numOfCities,numOfRobots);
% 
% x(:,:,1) = ...
%     [0     1     0     0     0     0;
%      1     0     0     0     1     0;
%      0     0     0     0     0     0;
%      0     1     0     0     0     0;
%      0     0     0     1     0     0;
%      0     0     0     0     0     0];
% 
% x(:,:,2) = ...
%     [0     1     0     0     0     0;
%      1     1     0     0     0     1;
%      0     0     0     0     0     0;
%      0     0     0     0     0     0;
%      0     0     0     0     0     0;
%      0     1     0     0     0     1;];
% 
%  x(:,:,3) = ...
%     [0     1     0     0     0     0;
%      1     1     1     0     0     0;
%      0     1     0     0     0     0;
%      0     0     0     0     0     0;
%      0     0     0     0     0     0;
%      0     0     0     0     0     0;];

%% mTSP constraints


% When setting j = 1 with no x(1,1,k) == 0 for all k constraint:


% 
%      1     2     2     5     4     2     1
%      1     2     2     4     3     2     1
%      1     2     6     2     2     2     1


% When setting j=2 with x(1,1,k) == 0 for all k
% 
% value(objective3)
% 
% ans =
% 
%   187.6801
% 
% value(objective2)
% 
% ans =
% 
%   187.6621


% routeMatrix =
% 
%      1     2     4     2     5     2     1
%      1     2     2     6     3     2     1
%      1     2     4     2     4     2     1

% ConstraintS 1 and 2 (eqns 10 and 11 in the ICTAI paper)
% Ensure robots start and end at the first (dummy node)
constraint1 = [];
constraint2 = [];
x1jkTotal = 0;
xj1kTotal = 0;
for k = 1:numOfRobots
    x1jkTotal = 0;
    xj1kTotal = 0;
    for j = 1:numOfCities
        x1jkTotal = x1jkTotal + x(1,j,k);
        xj1kTotal = xj1kTotal + x(j,1,k);
    end
    constraint1 = [constraint1, (x1jkTotal >= 1)];
    constraint2 = [constraint2, (xj1kTotal >= 1)];
end
% constraint1 = [];
% constraint2 = [];

% Constraints 3 and 4 (eqns 12 and 13)
% Ensures robots, as a whole, enter and exit each node once except for the
% first node. We want all robots to go to the first node, so we want the
% robots to enter and exit that node 3 times, not just once. We implement
% that change in the following constraints after these 2.

constraint3 = []; % constraint 3  start
% Then, build a vector to hold the sum of all of
% the values of each numOfCities x numOfRobots matrix.
% See handwritten work for a more intuitive visualiztion.
planeTotal_ik = 0;
% for j = 1:numOfCities
%     % reset the value of planeTotal_ik to 0 for the next constraint:
%     planeTotal_ik = 0;
%     for i = 1:numOfCities
%         for k = 1:numOfRobots
%                   planeTotal_ik = planeTotal_ik + x(i,j,k);
%         end
%     end
%     % Build the constraint here
%     constraint3 = [constraint3, (planeTotal_ik == numNodeTraversals(j)) ];
% end
for k = 1:numOfRobots
    for j = 1:numOfCities
        planeTotal_ik = 0;
        for i = 1:numOfCities
            planeTotal_ik = planeTotal_ik + x(i,j,k);
        end
        constraint3 = [constraint3, (planeTotal_ik == numNodeTraversals(j,k))];
    end
end

constraint4 = []; % constraint 4 start
planeTotal_jk = 0;
% for i = 1:numOfCities
%     planeTotal_jk = 0;
%     for j = 1:numOfCities
%         for k = 1:numOfRobots
%                   planeTotal_jk = planeTotal_jk + x(i,j,k);
%         end
%     end
%     constraint4 = [constraint4, (planeTotal_jk == numNodeTraversals(i)) ];
% end
for k = 1:numOfRobots
    for i = 1:numOfCities
        planeTotal_jk = 0;
        for j = 1:numOfCities
            planeTotal_jk = planeTotal_jk + x(i,j,k);
        end
        constraint4 = [constraint4, (planeTotal_jk == numNodeTraversals(i,k))];
    end
end


% Constraint 5 (eqn 14)
% Prevent robots from entering and exiting more than one city at a time
lhsSum = 0;
rhsSum = 0;
constraint5 = [];
for j = 2:numOfCities
    for k = 1:numOfRobots
        for i = 1:numOfCities
            if i ~= j
                  lhsSum = lhsSum + x(i,j,k);
                  rhsSum = rhsSum + x(j,i,k);
            end
        end
        constraint5 = [constraint5, (lhsSum == rhsSum)];
        % 0 out sums for next constraint
        lhsSum = 0;
        rhsSum = 0;
    end  
end
% constraint5 = [];

% Constraint 6: subtour elimination constraints (SECs)
xS = 0;
constraint6 = [];
for j = 2:numOfCities
    for i = 2:numOfCities 
        for k = 1:numOfRobots
            xS = xS + x(i,j,k);
            if i ~= j
              constraint6 = [constraint6, (u(i)-u(j) + p*xS <= p -1)];
            end
            xS = 0; % reset to 0 for next iteration of constraint 
        end
    end
end
constraint6 = [];

% Ensure a robot can't visit itself. Remove when instituting lingering
constraint7= [];
for i = 1:numOfRobots
    constraint7 = [constraint7, trace(x(:,:,i)) == 0];
end


% Ensure the number of node traversals for each city is
% at least equal to one:

% Changed so the node traversals are for each robot instead of total:
% We use sum so the number of node traversals is at least 1 for at least
% one robot. I.e., you could have [1 0 0] [0 1 0] or [0 2 1] but NOT [0 0 0]
constraint8 = [(sum(numNodeTraversals,2) >= 1), (numNodeTraversals(1,:) >= 1)];
constraint8 = [(sum(numNodeTraversals,2) >= 1)];

%% Dantzig-Fulkerson-Johnson Subtour Elminination Constraint
% 
% % We want to outrule every possible subtour. For a 10-city system,
% % we'll have 2e10 constraints. Each constraint outrules a specific subtour.
% 
% % First thing we need to do: get a set of all subsets of the set 
% % V\{1} = {1,2,3,...,numOfCities}\{1}. This set of all subsets of V is 
% % just the set of all subtours! We don't include 1 because, if we did,
% % then we'd have tours with 1 in them, making them NOT subtours by
% % definition.
% 
% 
% 
% % Definition. A subtour is a tour that does not include the origin.
% 
% % So first get all possible subtours:
% S = PowerSet(V);
% 
% % Ensure the cardinality of the subsets lays between 2 and (numOfCities-1)
% S = S(2:end-1);
% 
% % bin represents whether a robot is inside or outside a subset. If bin = 1,
% % that means the robot is in the subset S{t}. Otherwise,
% % if bin = 0, the robot is not in the subset S{t} (can't have any tours
% % inside S). bin is per robot. 
% bin = binvar(numel(S),numOfRobots,'full');
% 
% % Now implement the constraints:
% constraintSEC = [];
% xijkSum = 0;
% xijkSum2 = 0;
% xijkSum3 = 0;
% % For each robot
% for k = 1:numOfRobots
%     % For each subtour (each set in S)
%     for t = 1:numel(S)
%         % NOW we do our double sum
%         for i = 1:numel(S{t})
%             % We sum all j's that aren't in the set S:
%             for j = 1:numel(V0)
%                 % for j NOT in S, 
%                 if( sum((ismember(cell2mat(S{t}),j))) == 0 ) 
%                     xijkSum = xijkSum + x(S{t}{i},j,k); %exits (wrt S{t})
%                     xijkSum2 = xijkSum2 + x(j,S{t}{i},k); %entrances
%                 % for j IN S, 
%                 else
%                     % Counts the number of traversals inside the subtour
%                     % S{t}:
%                     xijkSum3 = xijkSum3 + x(S{t}{i},j,k);
%                 end
%             end
%         end
%         % If the robot IS in the set, then bin(t,k) is 1, so the
%         % constraints with xijkSum3 are made. These constraints force the robot,
%         % when it's in the set, to enter at least once, exit at least once,
%         % and have a number of entrances and exits equal to each other.
%         constraintSEC = [constraintSEC, implies((bin(t,k)), ... 
%             [(xijkSum >=1), (xijkSum2 >= 1), (xijkSum == xijkSum2) ]) ];
%         % If the robot is NOT in the set, ensure that the number of
%         % traversals (measured by the number of times xijk = 1 for i,j in
%         % the set S) is equal to 0:
%         constraintSEC = [constraintSEC,implies(1-(bin(t,k)),xijkSum3 ==0)];
%         xijkSum = 0;
%         xijkSum2 = 0;
%         xijkSum3 = 0;
%         % Note where these constraints are in this series of loops.
%         % We want the constraints to be made for each robot
%         % and each subtour. 
%     end
% end
% 
% 
% % Note on summation in website: i not in S really means i in V0\S, where
% % V0 is the set of ALL nodes (including the dummy node)
% 
% % The xijkSums don't have to hold for each k. Only has to hold for one k.
% % Reason: if we did it for each k, for each subset, we'd make the robots
% % obey the constraint for each subset, forcing the robots to visit all of
% % the cities, which results in them having the same tours because that one
% % tour is the shortest for a single robot.
% 
% % Add another condition: if the robots don't visit the specific S{t}, then
% % the sum of the xijks in S{t} must be 0. Use an if-else statement here to
% % implement it. HOW DO WE KNOW IF THE ROBOTS VISITED THE NODES IN THAT
% % SUBTOUR BEFORE RUNNING THE SUM?
% 
% 
% % Couple notes after working on it:
% 
% % We do just want x >= 1. That's cause we want tour retracing, and if 
% % we force every subtour to have at least 2 different connections, we could
% % lose tours where a robot marches out to a city and then takes the same
% % route back (hence just have one retracing).


%% Final constraints and objectives

% Constraint on x: must be greater than or equal to 0 (so we don't get 
% negative integers) and less than some upper bound M
M = 3;
constraintx = [(x >= 0), (x <= M)];
% So we're saying that we don't want the robots to visit a city more than
% M (3) times for all cities.

constraintDummyNode = [(x(1,1,1) == 0), (x(1,1,2) == 0), (x(1,1,3) == 0)];
constraintDummyNode = [];


% Complete constraints             
constraints = [constraint1, constraint2, constraint3, constraint4, ...
               constraint5, constraint6, constraint8, constraintx, ...
               constraintDummyNode];

% initialize a combined set of constraints that will constraints and
% constraintsSEC
constraintSEC = []; % starts as empty
constraintsCombined = [constraints constraintSEC];
           
% Objectives
SalesmanDistances = [];
for k = 1:numOfRobots
    SalesmanDistances = [SalesmanDistances ...
                         sum(sum(C(1:end,1:end).*x(1:end,1:end,k)))];
%     SalesmanDistances = [SalesmanDistances ...
%                          sum(sum(C(1:end,1:end).*x(1:end,1:end,k)))];
end


objective1 = sum(SalesmanDistances); % Minimize total distance of all tours
objective2 = max(SalesmanDistances); % Minimize max individual robot tour
% Min weighted sum of max single robot tour and number of node revisits
objective3 = max(SalesmanDistances) + 0.001*sum(sum(numNodeTraversals));


%% Solve Original System

if (flagSolveBaseProblem == 1)

    tic 
    options = sdpsettings('verbose',1,'solver','Gurobi');
    sol = optimize(constraints,objective3,options);
    value(objective3)
    value(x)
    toc

%     [RobotDistances, RobotRowIdxs, RobotColIdxs] = ...
%         getAllRobotTourInfo(C, numOfCities, numOfRobots, value(x2));
end

%% New Retraced Naive Method w/Lazy Constraints
% This method uses lazy constraints in the sense that we don't add DFJ
% constraints unless the solution violates them. The only DFJ constraints
% we add are ones that are directly violated by that specific solution.

if (flagIterativeNew == 1)
    totTime = 0;
    tic;
    
    routeMatrix = []; 
    distanceMatrix = [];
    
    flag = 0;
    flagViolation = 0;
    
  
    while(1)
        % run simulation
        options = sdpsettings;
        options.gurobi.TimeLimit = timeLimit;
        try
            sol = optimize(constraintsCombined,objective2,options);
        catch
            disp("All feasible solutions have been excluded");
            flagIsFeasible = 0;
            totTime = toc;
            soln = value(x);
            objectiveMinMax = value(objective2);
            robotDistances = distanceMatrix;
            return
        end
        % Right after solving just the first time, check the total time. If
        % it took longer than the time limit, stop the sim. Here, we
        % return whatever we got from this first sim run. We don't know if
        % the solution is feasible, but it doesn't matter since we went
        % over the time limit. So list feasibility as unknown with a 2.
        if (toc > timeLimit)          
            disp(['An individual solving instance took longer than the overall' ...
                ' time limit.' ]);
            disp('Terminating and returning incomplete solution');
            flagIsFeasible = 2;
            totTime = toc;
            soln = value(x);
            objectiveMinMax = value(objective2);
            robotDistances = distanceMatrix;
            return
        end
        sol.problem;
        
        xSol   = value(x);
%         binSol = value(bin);
        
        xijkSum  = 0;  xijkSumVal  = 0;
        xijkSum2 = 0;  xijkSum2Val = 0;
        xijkSum3 = 0;  xijkSum3Val = 0;
        
        disp('Forming DFJ Constraints...')
        for k = 1:numOfRobots
            % For each subtour (each set in S)
            for t = 1:numel(S)
                % NOW we do our double sum. For all i IN S,
                for i = 1:numel(S{t})
                    % We sum all j's that aren't in the set S:
                    for j = 1:numel(V0)
                        % for j NOT in S,
                        if( sum((ismember(cell2mat(S{t}),j))) == 0 )
                            xijkSum = xijkSum + x(S{t}{i},j,k); %exits (wrt S{t})
                            xijkSum2 = xijkSum2 + x(j,S{t}{i},k); %entrances
                            
                            xijkSumVal = xijkSumVal + xSol(S{t}{i},j,k);
                            xijkSum2Val = xijkSum2Val + xSol(j,S{t}{i},k);
                            
                        else
                            % for j NOT in S,
                            % Counts the number of traversals inside the subtour
                            % S{t}:
                            xijkSum3 = xijkSum3 + x(S{t}{i},j,k);
                            
                            xijkSum3Val = xijkSum3Val + xSol(S{t}{i},j,k);
                        end
                    end
                end
                
                if(value(bin(t,k))==1)
                    
                    inSubtourConstraint = [(xijkSumVal >=1), ...
                        (xijkSum2Val >= 1), (xijkSumVal == xijkSum2Val)];
                    
                    if ( sum(inSubtourConstraint) ~= 3 )
                        constraintSEC = [constraintSEC, implies((bin(t,k)), ...
                            [(xijkSum >=1), (xijkSum2 >= 1), (xijkSum == xijkSum2) ]) ];
                    end
                    
                else
                    
                    outOfSubtourConstraint = [(xijkSum3Val == 0)];
                    
                    if (outOfSubtourConstraint(1) ~= 1)
                        constraintSEC = [constraintSEC, ...
                            implies(1-(bin(t,k)),xijkSum3==0)];
                    end
                    
                end
                xijkSum  = 0;  xijkSumVal  = 0;
                xijkSum2 = 0;  xijkSum2Val = 0;
                xijkSum3 = 0;  xijkSum3Val = 0;
                
            end
        end

        % If we didn't get new constraints, then run the model as usual. If
        % we DID get new constraints, rerun the sim right now with the new
        % constraints. The "rerun the sim right now" happens in the else
        % statement down below
        if (numel(constraintSEC) == 0)
%         if(flagSubtour(value(x)) == false)
        
%             value(objective3);
%             value(x);

            adjMatrices = value(x);

            robot1Graph = Graph2(numOfCities);
            robot2Graph = Graph2(numOfCities);
            robot3Graph = Graph2(numOfCities);

            robot1Graph.loadAdjacencyMatrix(adjMatrices(:,:,1));
            robot2Graph.loadAdjacencyMatrix(adjMatrices(:,:,2));
            robot3Graph.loadAdjacencyMatrix(adjMatrices(:,:,3));

            robot1Graph.printAllPaths(1,1);
            robot2Graph.printAllPaths(1,1);
            robot3Graph.printAllPaths(1,1);

            robot1Paths = robot1Graph.m_visitedRoutes;
            robot2Paths = robot2Graph.m_visitedRoutes;
            robot3Paths = robot3Graph.m_visitedRoutes;

            temp1 = robot1Paths;
            for i = 1:numel(robot1Paths)
            robot1Paths{i} = cell2mat(temp1{i});
            end
            robot1Paths = robot1Paths';
            [~,yolo] = sort(cellfun(@length,robot1Paths));
            robot1Paths = robot1Paths(yolo);

            temp2 = robot2Paths;
            for i = 1:numel(robot2Paths)
            robot2Paths{i} = cell2mat(temp2{i});
            end
            robot2Paths = robot2Paths';
            [~,yolo] = sort(cellfun(@length,robot2Paths));
            robot2Paths = robot2Paths(yolo);

            temp3 = robot3Paths;
            for i = 1:numel(robot3Paths)
            robot3Paths{i} = cell2mat(temp3{i});
            end
            robot3Paths = robot3Paths';
            [~,yolo] = sort(cellfun(@length,robot3Paths));
            robot3Paths = robot3Paths(yolo);

            for i = 1:numel(robot1Paths)
                if flag == 1
                    break
                end
                for j = 1:numel(robot2Paths)
                    if flag == 1
                        break
                    end
                    for k = 1:numel(robot3Paths)
                    if flag == 1
                        break
                    end

                        % Attempt to concatenate the paths from each robot into a 
                        % matrix. If that doesn't work, skip to the next iteration:
                        try
                            routeMatrix = [robot1Paths{i}; robot2Paths{j}; ...
                                           robot3Paths{k}];
                        catch
    %                         disp('Tour lengths mismatch. skipping this iteration...')
                            continue
                        end

                        % Now, begin checks that every city has been visited in this
                        % chosen set of tours:

                        % ASSERT that the number unique elements among the set is not
                        % greater than the number of cities (impossible)
                        assert(numel(unique(routeMatrix)) <= numOfCities, ...
                            'routeMatrix has more unique elements (cities) than' ...
                            ,'the number of unique cities (impossible).')

                        % If the number of unique cities visited is less than the 
                        % total number of cities, then we didn't visit every city, so
                        % skip this iteration.
                        if(numel(unique(routeMatrix)) < numOfCities)
                            continue
                        end

                        % Now, assemble a matrix of distances between the robots'
                        % tours:
                        distanceMatrix = getOnlyDistances(routeMatrix,C);

                        % Finally, check the inter-city distances. If every
                        % inter-robot distance, for all simulation steps, is
                        % less than the tether length, assign the flag to 1,
                        % allowing us to break out of the nested for loops and
                        % end the algorithm:


                        if ( sum(sum(distanceMatrix([1,2],:) > tetherLength)) == 0 || ...
                            sum(sum(distanceMatrix([1,3],:) > tetherLength)) == 0 || ...
                            sum(sum(distanceMatrix([2,3],:) > tetherLength)) == 0)

                            flag = 1;
                            
                        elseif (toc > timeLimit)    
                            % Contingency: if the sim is taking longer than the
                            % set runtime limit, just stop the sim here and
                            % return the solution we do have. It won't be
                            % feasible, but just have something. Then return.
                           disp(['Could not find a tether-satisfying ', ...
                                'solution in time. Returning the last ', ...
                                'non-tether feasible solution.']);
                            flagIsFeasible = 0;
                            totTime = toc;
                            soln = xSol;
                            objectiveMinMax = value(objective2);
                            robotDistances = distanceMatrix;
                           return
                        end
                        % Otherwise, continue iterating. If we get to the end
                        % of the set of iterations and still don't have a
                        % solution that works, we'll have to rerun the sim. The
                        % command to rerun the sim is based on the flag, and it
                        % happens outside of these iterations.
                        if (flag == 1)
                            disp('found a feasible solution')
                        end

                    end
                end
            end

            % If the flag is 0, it means no solution worked from the previous
            % set, so update the combined constraints and rerun:
            if (flag == 0)
                for ii = 1:numOfRobots
                    constraintsCombined = [constraintsCombined, ...
                                   (x(:,:,ii) ~= value(x(:,:,ii))) ];
                    % Also, clear out contraintSEC. We only want to be
                    % eliminating subtours for this specific solution.
                    % Otherwise, constraintSEC will included subtour
                    % constraints from a previous solution that may not
                    % matter here, consequently slowing down the solver:
                    constraintSEC = [];
                    % Note that constraints is static. It only has the
                    % constraints from the original formulation and nothing
                    % else
                end
                % But check running time here: if we've taken beyond our
                % allowable time limit, end it here:
                if (toc > timeLimit)
                    disp('Global time limit reached.');
                    disp('Terminating and returning incomplete solution');
                    flagIsFeasible = 2;
                    totTime = toc;
                    soln = value(x);
                    objectiveMinMax = value(objective2);
                    robotDistances = distanceMatrix;
                    return
                end
            % Otherwise, we found a solution that works, so break out and
            % end.
            else
                break
            end

        % If we DID find subtours, constraintSEC will be populated, so add
        % that in and rerun. But first, check to see how long that
        % individual instance of the sim ran. If that one solving iteration
        % ran over the time limit, cut off the sim here.
        else
            try
                constraintsCombined = [constraintsCombined, constraintSEC];
                constraintSEC = [];
            catch
                disp("cannot add SECs")
            end
            % But check running time here: if we've taken beyond our
            % allowable time limit, end it here. Don't know if feasible.
            if (toc > timeLimit)
                disp('Global time limit reached.');
                disp('Terminating and returning incomplete solution');
                flagIsFeasible = 2;
                totTime = toc;
                soln = value(x);
                objectiveMinMax = value(objective2);
                robotDistances = distanceMatrix;
                return
            end   
        end
        
    end
    totTime = toc;
    soln = xSol;
    objectiveMinMax = value(objective2);
    robotDistances = distanceMatrix;
end

%% Adding Tether Constraints Iteratively-- Getting the New System

if (flagIterative == 1)

% Set RobotDistances initially to a value greater than the tetherLength
% so we can break into the loop
% RobotDistances = [(tetherLength+1); (tetherLength+1)];
% What it is normally: RobotDistances = (tetherLength+1);

% Initialize pastAnswers vector. Will be used for breaking out 
% of the loop, if necessary.
pastAnswers = 0;

% RobotDistances > tetherLength returns a boolean vector
% on that condition. If that vector's sum is not 0, then at least one
% distance is greater than the tether length. As such, the code 
% will run.

% old: while( sum(sum(RobotDistances > tetherLength)) ~= 0 ) 
% (covers every robot distance)

% New: while( sum(sum(RobotDistances(1:2,:) > tetherLength)) ~= 0 ) 
% (covers distances only between robots 1 and 2 and 1 and 3


% while( sum(sum(RobotDistances(1:2,:) > tetherLength)) ~= 0 ) 


totTime = 0;
tic % to see how long the sim takes
while(1)

    % run simulation
    options = sdpsettings('verbose',0,'solver','Gurobi');
    opt = sdpsettings;
    opt.gurobi.TimeLimit = timeLimit;
    sol = optimize(constraints_2,objective2,options);
    sol.problem;
    value(objective2);
    value(x);

    % Record previous values of the max tour length of some robot
    pastAnswers = [pastAnswers value(objective1)];

%     [RobotDistances, RobotRowIdxs, RobotColIdxs] = ...
%         getAllRobotTourInfo(C, numOfCities, numOfRobots, value(x2));
       
    
    [RobotDistances, RobotRowIdxs, RobotColIdxs] = ...
       getAllRobotTourInfo(C, numOfCities, numOfRobots, value(x));
    % RobotDistances gives the distances between the robots at each
    % time step. Row 1 gives distance(r1,r2), row 2 gives
    % distance(r1,r2), and row 3 gives distance(r2,r3).
    
    
    % SO HERE WE'D TAKE IN RobotRowIdxs AND TURN IT IN TO ALL OF OUR
    % TOUR COMBOS. THAT IS, WE'D DO THIS HERE:

    % check every case. Now, to an exclude a solution, all
    % possibilities of tether configurations between the robots
    % must be ruled to violate the tether constraint. In other
    % words, we're checking every possible option.
    
    [finalTourOneCombos, finalTourTwoCombos, ...
    finalTourThreeCombos] = getAllTourCombos(RobotColIdxs,numOfRobots);

    % use flags to break out of the loops:
    flag = 0;
    for i = 1:numel(finalTourOneCombos(:,1))
        
        if flag == 1
            break
        end
        
        for j = 1:numel(finalTourTwoCombos(:,1))
            
            if flag == 1
                break
            end
            
            for k = 1:numel(finalTourThreeCombos(:,1))
                
                % Package the finalTourXCombos as a single matrix
                % for use in the getOnlyDistances function:
                newColIdxs = [];
                newColIdxs(:,:,1) = finalTourOneCombos(i,:);
                newColIdxs(:,:,2) = finalTourTwoCombos(j,:);
                newColIdxs(:,:,3) = finalTourThreeCombos(k,:);
                
                % Call function to get robot distances out of this set of 
                % tours
                [RobotDistancesTwo] = ...
                getOnlyDistances(newColIdxs,C);
                
                % Or what we could do is just say, "if this solution
                % satisfies the tether constraint, keep it and you're 
                % done." Like this:
                if ( sum(sum(RobotDistancesTwo(1:2,:) > tetherLength)) == 0  ||...
                    sum(sum(RobotDistancesTwo([1,3],:) > tetherLength)) == 0  ||...
                    sum(sum(RobotDistancesTwo([2,3],:) > tetherLength)) == 0 )
                
                    flag = 1;
                end
                
                % if the solution doesn't violate the constraint, break
                % out. if it does violate the tether constraint , keep
                % going:
                if flag == 1
                    break
                end
            
                
            end
        end
    end
    
    
 
            
%         % If any combination of robot tether configurations has 
%         % any inter-robot distance greater than the tether length,
%         % then throw out the tour.
%         if ( sum(sum(RobotDistances(1:2,:) > tetherLength)) ~= 0  &&...
%             sum(sum(RobotDistances([1,3],:) > tetherLength)) ~= 0  &&...
%             sum(sum(RobotDistances([2,3],:) > tetherLength)) ~= 0 )
%         
%         end

    

        % Then include this at the end. If nothing worked from above,
        % just exclude the solution entirely and move on. We don't need
        % to add anything below here because if we got this line of code,
        % that means nothing worked above, so we can just exclude that
        % solution with no questions asked.
        
        % If any distance between two robots is greater than the tether
        % length, exclude that specific solution and rerun.
        
    if flag == 0
        for k = 1:numOfRobots
             constraints_2 = [constraints_2, exclude(x(:,:,k), ...
             value(x(:,:,k)))];
        end
        
    else
        % Save the tour info, get the final RobotDistancesTwo matrix,
        % and break out of the while loop:
        for i = 1:numOfRobots
            RobotColIdxs(:,:,i) = newColIdxs(:,:,i);
            RobotRowIdxs(:,:,i) = [ones(1,1,1) newColIdxs(:,1:end-1,i)];
        end
        [RobotDistancesTwo] = ...
                getOnlyDistances(RobotColIdxs,C);        
        break
    end


end


% Run simultion. Get tour. Call function that gets all the different tours.
% Use three nested for loops to check every
% differeint combination of the tour. If one combo works, take that one and
% that's the answer. If no combos work, rerun the sim and do it again.

% Once you include the ones in your other code, you're basically done
% there. Then, you need to include the three nested for loops in this code
% to check all the combinations.

totTime = toc;

end

%


%% Plotting

set(0,'defaulttextinterpreter','latex');
% We want to visualize the paths of the robots. We could
% do this by drawing lines of the robots' paths on a grid. 
% The robots would have different colored-paths to help distinguish
% the paths. This strategy would work for a low number of robots only,
% but the project will involve only a low number of robots anyway, so 
% no problem.   

% Putting the routeMatrix in terms of robotRowIdxs and colIdxs to make
% plotting work.

RobotRowIdxs = [];
RobotColIdxs = [];

RobotRowIdxs(:,:,1) = routeMatrix(1,1:(numel(routeMatrix(1,:))-1));
RobotColIdxs(:,:,1) = routeMatrix(1,2:(numel(routeMatrix(1,:))));

RobotRowIdxs(:,:,2) = routeMatrix(2,1:(numel(routeMatrix(1,:))-1));
RobotColIdxs(:,:,2) = routeMatrix(2,2:(numel(routeMatrix(1,:))));

RobotRowIdxs(:,:,3) = routeMatrix(3,1:1:(numel(routeMatrix(1,:))-1));
RobotColIdxs(:,:,3) = routeMatrix(3,2:1:(numel(routeMatrix(1,:))));

figure(1)
box on
plotRoute(nodecoords, RobotRowIdxs(1,:,1) , RobotColIdxs(1,:,1), 'red')
hold on
plotRoute(nodecoords, RobotRowIdxs(1,:,2) , RobotColIdxs(1,:,2), 'magenta')
plotRoute(nodecoords, RobotRowIdxs(1,:,3) , RobotColIdxs(1,:,3), 'blue')

% 1,2,3 specifcy the two robots with which you want the tethers drawn
if (flagTether == "Tethered")
    
    % The robots that meet the tether constraints change every
    % run (e.g., 1 and 3 may be tethered one run, 1 and 2 the next, etc.), so
    % figure out between which robots are meeting the tether constraints:
    
    % row 1: between 1 and 2. r2: 1,3. r3: 2,3.
    if (sum(distanceMatrix(1,:) < tetherLength) == ...
            numel(distanceMatrix(1,:)))
        
        plotTethers(nodecoords, RobotRowIdxs, ...
        RobotColIdxs, 1, 2, 'black', '--')
    end
    
    if (sum(distanceMatrix(2,:) < tetherLength) == ...
            numel(distanceMatrix(2,:)))
        
        plotTethers(nodecoords, RobotRowIdxs, ...
        RobotColIdxs, 1, 3, 'black', ':' )
    end
    
    if (sum(distanceMatrix(3,:) < tetherLength) == ...
            numel(distanceMatrix(3,:)))
        
        plotTethers(nodecoords, RobotRowIdxs, ...
        RobotColIdxs, 2, 3, 'black', ':' )
    end    
    hold off
    
    % Still a bit of a hacky fix (depends on not being able to change the
    % number of robots), but better than before
    
end


% Set text and axes to LaTeX-interpreted characters, and assign current,
% and save plot onto computer:
axes = gca;

axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.TickLabelFormat  = '\\textbf{%g}';

axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.TickLabelFormat = '\\textbf{%g}';

% Save the plot onto your computer:
ax = gca;
% exportgraphics(ax, ...
%     '/home/walter/Desktop/ThesisFigures/NaiveBacktracking/Tether70/LAZYnaiveBack2Norm.jpg', 'Resolution', '1000')


%%
% 
% % matrix = ones(11,11,1);
% % 
% % matrix =   [0     1     0     0     0     0     0     0     0     0   0;
% %             1     0     0     0     0     0     0     1     0     0   0;
% %             0     0     0     1     0     0     0     1     0     0   0;
% %             0     0     1     0     1     0     0     0     0     0   0;
% %             0     0     0     1     0     0     0     0     0     0   0;
% %             0     0     0     0     0     0     0     0     0     0   0;
% %             0     0     0     0     0     0     0     0     0     0   0;
% %             0     1     1     0     0     0     0     0     0     0   0;
% %             0     0     0     0     0     0     0     0     0     0   0;
% %             0     0     0     0     0     0     0     0     0     0   0;
% %             0     0     0     0     0     0     0     0     0     0   0];
% 
% % Example with no subtour
% %
% 
% x = [];
% 
% x(:,:,1) = [
%      1     0     1     0     0;
%      0     0     0     0     0;
%      0     0     0     1     0;
%      1     0     0     0     0;
%      0     0     0     0     0;];
% 
% 
% x(:,:,2)= [
%      1     1     0     0     0;
%      1     0     0     0     0;
%      0     0     0     0     0;
%      0     0     0     0     0;
%      0     0     0     0     0;];
% 
% 
% x(:,:,3) = [
%      3     0     0     0     1;
%      0     0     0     0     0;
%      0     0     0     0     0;
%      0     0     0     0     0;
%      1     0     0     0     0;];
%         
% % subtour = grabSubtour(x);
% subtour = grabAllSubtours(x);

% 
% Example with subtour (cooked it up)

% xsub = zeros(5,5,3);
% xsub(1,1,1) = 1; 
% xsub(1,1,2) = 1; %xsub(2,3,2) = 1; xsub(3,2,2) = 0;
% % xsub(4,5,3) = 1; xsub(5,4,3) = 1;
% xsub(2,3,3) = 1; xsub(3,4,3) = 1; xsub(4,5,3) = 1; xsub(5,2,3) = 1; 

% subtour2 = grabAllSubtours(xsub)

% 
% x = [];
% 
% 
% x(:,:,1) = [
%      1     0     0     0     0;
%      0     1     0     0     0;
%      0     0     1     0     0;
%      0     0     0     1     0;
%      0     0     0     0     0;];
% 
% 
% x(:,:,2) = [
%      1     0     0     0     0;
%      0     0     0     0     0;
%      0     0     0     0     0;
%      0     0     0     0     0;
%      0     0     0     0     0;];
% 
% x(:,:,3) = [
%      1     0     0     0     0;
%      0     0     0     0     0;
%      0     0     0     0     0;
%      0     0     0     0     0;
%      0     0     0     0     1;];
%  
%  
% subtour3 = grabAllSubtours(x)
%  
%  
%  
%  
%  
%  subtour3 = grabSubtour(x)
% 
% graphx.resetObject(); 
% numCities = numel(x(1,:,1));
% numRobots = numel(x(1,1,:));
% graphx = Graph2(numCities);
% graphx.loadAdjacencyMatrix(x(:,:,1));
% paths = graphx.printAllPaths(3,4); % i and j represent city indices here
% pathTemp = graphx.m_visitedRoutes;
% % % If you don't 'reset the object,' the route nubmer, cnv, and
% % % visitedRoutes still stay from old run, screwing things up. Will
% % % need to fix Graph2.m later more thoroughly
% graphx.resetObject(); 

%% Utility Functions

function[flag] = flagSubtour(x)


    numCities = numel(x(1,:,1));
    numRobots = numel(x(1,1,:));

    graphX1 = Graph2(numCities);
    graphX1.loadAdjacencyMatrix(x(:,:,1));
    
    graphX2 = Graph2(numCities);
    graphX2.loadAdjacencyMatrix(x(:,:,2));
    
    graphX3 = Graph2(numCities);
    graphX3.loadAdjacencyMatrix(x(:,:,3));
    
    % GRAB LIST OF UNVISITED CITIES
    graphX1.printAllPaths(1,1); 
    graphX2.printAllPaths(1,1); 
    graphX3.printAllPaths(1,1);
    
    allPathsFromHome = [graphX1.m_visitedRoutes, ...
        graphX2.m_visitedRoutes, graphX3.m_visitedRoutes];
    
    graphX1.resetObject();
    graphX2.resetObject(); 
    graphX3.resetObject();
    
    % Grab list of visited cities from Path-DFS solution:
    citiesVisited = [];
    for i = 1:numel(allPathsFromHome)
        citiesVisited = [citiesVisited,cell2mat(allPathsFromHome{i})];
    end
    
    % Discern unvisited cities:
    uniqueCitiesVisited = unique(citiesVisited);
    citiesList = 1:1:numCities;
    citiesUnvisited = setdiff(citiesList,uniqueCitiesVisited);
    
    if (isempty(citiesUnvisited) == false)
        flag = true;
        return
    end

end



function[solutionSubtours] = grabAllSubtours(x)


    numCities = numel(x(1,:,1));
    numRobots = numel(x(1,1,:));

    graphX1 = Graph2(numCities);
    graphX1.loadAdjacencyMatrix(x(:,:,1));
    
    graphX2 = Graph2(numCities);
    graphX2.loadAdjacencyMatrix(x(:,:,2));
    
    graphX3 = Graph2(numCities);
    graphX3.loadAdjacencyMatrix(x(:,:,3));
    
    % GRAB LIST OF UNVISITED CITIES
    graphX1.printAllPaths(1,1); 
    graphX2.printAllPaths(1,1); 
    graphX3.printAllPaths(1,1);
    
    allPathsFromHome = [graphX1.m_visitedRoutes, ...
        graphX2.m_visitedRoutes, graphX3.m_visitedRoutes];
    
    graphX1.resetObject();
    graphX2.resetObject(); 
    graphX3.resetObject();
    
    % Grab list of visited cities from Path-DFS solution:
    citiesVisited = [];
    for i = 1:numel(allPathsFromHome)
        citiesVisited = [citiesVisited,cell2mat(allPathsFromHome{i})];
    end
    
    % Discern unvisited cities:
    uniqueCitiesVisited = unique(citiesVisited);
    citiesList = 1:1:numCities;
    citiesUnvisited = setdiff(citiesList,uniqueCitiesVisited);
    
    if (isempty(citiesUnvisited) == true)
        solutionSubtours = {};
        return
    end

    % LIST SUBTOURS
    solutionSubtours = {};
    for k = 1:numRobots
        for i = citiesUnvisited(1):1:citiesUnvisited(end)
            for j = citiesUnvisited(1):1:citiesUnvisited(end)

                % OBVIOUSLY NOT EFFICIENT. JUST MAKE IT WORK FOR NOW
                % Print all subtours as individual cells
                if (k == 1)
                    paths = [graphX1.printAllPaths(i,j)]; % i and j represent city indices here
                    pathTemp = graphX1.m_visitedRoutes;
                    % If you don't 'reset the object,' the route nubmer, cnv, and
                    % visitedRoutes still stay from old run, screwing things up. Will
                    % need to fix Graph2.m later more thoroughly
                    graphX1.resetObject(); 
                end
                
                if (k == 2)
                    paths = [graphX2.printAllPaths(i,j)]; % i and j represent city indices here
                    pathTemp = graphX2.m_visitedRoutes;
                    graphX2.resetObject(); 
                end
                
                if (k == 3)
                    paths = [graphX3.printAllPaths(i,j)]; % i and j represent city indices here
                    pathTemp = graphX3.m_visitedRoutes;
                    graphX3.resetObject(); 
                end
                
                
                solutionSubtours{end+1} = pathTemp;


            end
        end
    end

end


% IDENTIFY SHORTEST SUBTOUR IN BINVAR SOLUTION
% Input: binvar solution x
% Ouput: subtour. Should return an empty cell array if no subtour.
function[subtour] = grabSubtour(x)


    numCities = numel(x(1,:,1));
    numRobots = numel(x(1,1,:));

    graphX1 = Graph2(numCities);
    graphX1.loadAdjacencyMatrix(x(:,:,1));
    
    graphX2 = Graph2(numCities);
    graphX2.loadAdjacencyMatrix(x(:,:,2));
    
    graphX3 = Graph2(numCities);
    graphX3.loadAdjacencyMatrix(x(:,:,3));
    
    % GRAB LIST OF UNVISITED CITIES
    graphX1.printAllPaths(1,1); 
    graphX2.printAllPaths(1,1); 
    graphX3.printAllPaths(1,1);
    
    allPathsFromHome = [graphX1.m_visitedRoutes, ...
        graphX2.m_visitedRoutes, graphX3.m_visitedRoutes];
    
    graphX1.resetObject();
    graphX2.resetObject(); 
    graphX3.resetObject();
    
    % Grab list of visited cities from Path-DFS solution:
    citiesVisited = [];
    for i = 1:numel(allPathsFromHome)
        citiesVisited = [citiesVisited,cell2mat(allPathsFromHome{i})];
    end
    
    % Discern unvisited cities:
    uniqueCitiesVisited = unique(citiesVisited);
    citiesList = 1:1:numCities;
    citiesUnvisited = setdiff(citiesList,uniqueCitiesVisited);
    
    if (isempty(citiesUnvisited) == true)
        subtour = {};
        return
    end

    % LIST SUBTOURS
    subtour = {0};
    for k = 1:numRobots
        for i = citiesUnvisited(1):1:citiesUnvisited(end)
            for j = citiesUnvisited(1):1:citiesUnvisited(end)

                % OBVIOUSLY NOT EFFICIENT. JUST MAKE IT WORK FOR NOW
                % Print all subtours as individual cells
                if (k == 1)
                    paths = [graphX1.printAllPaths(i,j)]; % i and j represent city indices here
                    pathTemp = graphX1.m_visitedRoutes;
                    % If you don't 'reset the object,' the route nubmer, cnv, and
                    % visitedRoutes still stay from old run, screwing things up. Will
                    % need to fix Graph2.m later more thoroughly
                    graphX1.resetObject(); 
                end
                
                if (k == 2)
                    paths = [graphX2.printAllPaths(i,j)]; % i and j represent city indices here
                    pathTemp = graphX2.m_visitedRoutes;
                    graphX2.resetObject(); 
                end
                
                if (k == 3)
                    paths = [graphX3.printAllPaths(i,j)]; % i and j represent city indices here
                    pathTemp = graphX3.m_visitedRoutes;
                    graphX3.resetObject(); 
                end

                % Grab smallest subtour (in terms of number of elements) from
                % pathTemp. The '2' means that the min function looks at the number
                % of columns. We look at the nubmer of columns to see the size
                % because each cell is configured to be a row vector
%                 [maxSize, maxIdx] = max(cellfun('size', pathTemp,2));
%     % 
%     %             if (isempty(setdiff(cell2mat({1 4 2}),cell2mat(pathTemp{maxIdx}) == true)))
%     %                 disp('yonkers')
%     %             end
% 
                [maxSize, maxIdx] = max(cellfun('size', pathTemp,2));
    % 
    %             if (isempty(setdiff(cell2mat({1 4 2}),cell2mat(pathTemp{maxIdx}) == true)))
    %                 disp('yonkers')
    %             end

                % If this next path is longer, take it over the current
                % path
                if (numel(pathTemp{maxIdx}) > numel(subtour{1}))
                    subtour = pathTemp(maxIdx);
                end


            end
        end
    end
%     if ( isempty(setdiff(cell2mat(subtour),1:1:10))==true )
%         subtour = {};
%     end

end




% FIND A TOUR IN A CELLARRAY
% Inputs: set of sets (power set), desired tour
% Outputs: index of desired tour in power set
function [idx] = findSubtourInArray(powerSet, tour)
    idx = 0;
    for i = 1:numel(powerSet)
        if sum(ismember(cell2mat(powerSet{i}), tour)) == numel(tour) ...
                && numel(powerSet{i}) == numel(tour)
            idx = i;
        end
    end
end

% By Paulo Abelha
% Returns the powerset of set S
% S is a cell array
% P is a cell array of cell arrays
function [ P ] = PowerSet( S )
    n = numel(S);
    x = 1:n;
    P = cell(1,2^n);
    p_ix = 2;
    for nn = 1:n
        a = combnk(x,nn);
        for j=1:size(a,1)
            P{p_ix} = S(a(j,:));
            p_ix = p_ix + 1;
        end
    end
end


% DISTANCE FUNCTION
% Finds the Euclidean norm between two points
function [d] = distance(x1,y1,x2,y2)
    d = sqrt( (y2 - y1)^2 + (x2 - x1)^2 );
end


% Finds the 1_norm between two points
function [d1] = distance1(x1,y1,x2,y2)
    d1 = abs(x2 - x1) + abs(y2 - y1);
end


% GET ONLY ROBOT DISTANCES
% function [RobotDistancesTwo] = getOnlyDistances(RobotColIdxs,C)
% 
%     distanceTwoRobots = [];
%     for i = 1:length(RobotColIdxs(1,:,1))
%         distanceTwoRobots = [distanceTwoRobots, ...
%             C(RobotColIdxs(1,i,1), RobotColIdxs(1,i,2))];
%     end
%     RobotDistancesTwo(1,:) = distanceTwoRobots;
% %     display(RobotDistancesTwo);
%     
%     
%     distanceTwoRobots = [];
%     for i = 1:length(RobotColIdxs(1,:,1))
%         distanceTwoRobots = [distanceTwoRobots, ...
%             C(RobotColIdxs(1,i,1), RobotColIdxs(1,i,3))];
%     end
%     RobotDistancesTwo(2,:) = distanceTwoRobots;
% %     display(RobotDistancesTwo);
%     
%     
%     distanceTwoRobots = [];
%     for i = 1:length(RobotColIdxs(1,:,2))
%         distanceTwoRobots = [distanceTwoRobots, ...
%             C(RobotColIdxs(1,i,2), RobotColIdxs(1,i,3))];
%     end
%     RobotDistancesTwo(3,:) = distanceTwoRobots;
% %     display(RobotDistancesTwo);
% 
% end

% Updated version for updated Iterative Method:
function [distanceMatrix] = getOnlyDistances(routeMatrix, C)

    distanceR1R2 = [];
    for i = 1:length(routeMatrix(1,:))
        distanceR1R2 = [distanceR1R2, ...
            C(routeMatrix(1,i), routeMatrix(2,i))];
    end
    
    distanceR2R3 = [];
    for i = 1:length(routeMatrix(2,:))
        distanceR2R3 = [distanceR2R3, ...
            C(routeMatrix(2,i), routeMatrix(3,i))];
    end
    
    distanceR1R3 = [];
    for i = 1:length(routeMatrix(3,:))
        distanceR1R3 = [distanceR1R3, ...
            C(routeMatrix(1,i), routeMatrix(3,i))];
    end
    
    distanceMatrix = [distanceR1R2; distanceR2R3; distanceR1R3];

    
end



% GET ALL TOUR COMBINATIONS
function [finalTourOneCombos, finalTourTwoCombos, ...
    finalTourThreeCombos] = getAllTourCombos(RobotColIdxs,numOfRobots)
    
    % Now here, we're going to override the results of the sim and 
    % enforce the idea that a robot has stayed at a certain node for
    % an extended period of time. For example, making a robot stay 
    % at say, node 8, for more than one time step might look like this:
    
    %  RobotColIdxs = [9 8 8 10 1]
    %  RobotRowIdxs = [1 9 8 8 10]
    
    % The reason we care about robots persisting at a certain node is
    % because there's a chance an optimal tour for the robots (a tour 
    % that minimizes total travel distance or max individual tour length)
    % could involve having the robots persist at certain nodes for certain
    % periods of time.
    
    % We're going to be using MATLAB's built-in function nchoosek
    % that creates combinations for us. Note that the original tour WILL
    % be included in this list. As such, we're simply expanding the pool
    % of total feasible solutions-- we're not taking away any solutions.
    
    % Add ones at the start to complete the tours:
    totalRobotTours = [ones(1,1,numOfRobots) RobotColIdxs(:,:,:)];
    %  totalRobotTours = [RobotColIdxs(:,:,:)];

    % Designate number of slots open in each vector when we do nchoosek.
    % The number of slots is equal to 1 if ni = nmax and nmax if 
    % ni != nmax. ni is the number of non-one values in a tour and 
    % nmax is the largest number of non-one values out of a set of tours.
    % Ex. if n1 =1, n2=3, and n3=2, nmax = max(n1,n2,n3) = 3.
    
    % The number of times to repeat is related to the number of empty 
    % slots that have to be filled. Ex. [1 3 5 7 1], [1 2 1], [1 6 4 1].
    % Max number of slots nm = 3. Then, the number of repeats is found
    % by nm - ni. So for the tour 1, r1 = nm - n1 = 3 - 3 = 0 repeats.
    % r2 = 3 - 1 = 2 repeats. r3 = 3 - 2 = 1 repeats. So the repeats
    % look like [1 3 5 7 1], [1 2 2 2 1], [1 6 6 4 4 1].
    
    % So, count the number of non-one entries and put that in a vector 
    % numSlots. numSlots holds the number of non-one entries for each
    % tour. But first, get the max number of non-one entries for a certain 
    % tour:
    
    numSlots = [];
    for i = 1:numOfRobots
        numSlots(i) = sum(totalRobotTours(:,:,i) ~= 1) + 2;
    end
    
    numRepeats = [];
    for i = 1:numOfRobots
        % repelem(v,1) actually means no repeating (just copies vector),
        % so we manually adjust for that by adding 1.
        numRepeats(i) = (max(numSlots) - numSlots(i)) + 1;
    end
    
    % Repeat entries in the tour using numRepeats. numel = # of elements
    repeatVectors1 = [ 1 1 repelem(totalRobotTours(:,:,1),numRepeats(1))];
    repeatVectors2 = [ 1 1 repelem(totalRobotTours(:,:,2),numRepeats(2))];
    repeatVectors3 = [ 1 1 repelem(totalRobotTours(:,:,3),numRepeats(3))];
    % We include the 1s at the start to allow for 1s to show up in the 
    % beginning of the tour combos after doing nchoosek.
    
    
    % Now, apply nchoosek to get all combinations of tours:
    % Here's the issue with nchoosek: it gives you all combinations, NOT
    % all permutations. As such, you if you have [1 8 10 9] as a tour,
    % you'll never get [1 9 8 10] because MATLAB isn't calculating two
    % tours that have the same nodes but different orders.
    
    tourOneCombos = nchoosek(repeatVectors1, max(numSlots));
    tourTwoCombos = nchoosek(repeatVectors2, max(numSlots));
    tourThreeCombos = nchoosek(repeatVectors3, max(numSlots));

    % filter out duplicate rows (tours). Disgusting amounts of hardcoding
    % coming up:
    tourOneCombos = unique(tourOneCombos,'rows');
    tourTwoCombos = unique(tourTwoCombos,'rows');
    tourThreeCombos = unique(tourThreeCombos,'rows');

    % And now only get tours where all non-one values are in the rows.
    % That is, if a tour is [1 6 4 1] initially, we can't make a tour 
    % like [1 6 6 1] because we're throwing out 4. If we throw out 4,
    % we're not adding a time delay to the robot's touring-- we're making
    % a completely different tour!

    % find the intersection between row n in totalRobotTours and 
    % the values of row n that aren't 1. So if tour X = [8 9 10 1],
    % the code will give you tourXNoOnes = [8 9 10].
    tourOneNoOnes = intersect(totalRobotTours(:,:,1), ...
             (totalRobotTours(:,:,1) ~= 1).*totalRobotTours(:,:,1));
    tourTwoNoOnes = intersect(totalRobotTours(:,:,2), ...
             (totalRobotTours(:,:,2) ~= 1).*totalRobotTours(:,:,2));
    tourThreeNoOnes = intersect(totalRobotTours(:,:,3),...
             (totalRobotTours(:,:,3) ~=1).*totalRobotTours(:,:,3));
         
    finalTourOneCombos = [];
    finalTourTwoCombos = [];
    finalTourThreeCombos = [];
    
    % final task: get rid of 1s sandwiched between non-1 terms.
    % Approach: cycle through each row. In each row, cycle each entry
    % and check the neighbors of that entry. Example: you'd first move
    % to finalTourThreeCombos(1,:), and within that row, you'd look 
    % at each entry of that row
    
    % ROBOT 1
    for i = 1:numel(tourOneCombos(:,1))
        if sum((ismember(tourOneNoOnes,tourOneCombos(i,:)))) == numel(tourOneNoOnes) ...
                && tourOneCombos(i,1) == 1 && tourOneCombos(i,end) == 1
           
             finalTourOneCombos(i,:) = tourOneCombos(i,:);
        end
    % If this matrix happens to be the one with the largest number of
    % non-one entries, that one can't be changed, so just set that matrix
    % equal to the one tour.
    end
    if  numel(totalRobotTours(totalRobotTours(:,:,1) ~= 1)) == max(numSlots) -2 
        finalTourOneCombos = totalRobotTours(:,:,1);
    end
     
    % ROBOT 2
    % Ensure the first and last nodes are 1s
    for i = 1:numel(tourTwoCombos(:,1))     
        if sum((ismember(tourTwoNoOnes,tourTwoCombos(i,:)))) == numel(tourTwoNoOnes) ...
                && tourTwoCombos(i,1) == 1 && tourTwoCombos(i,end) == 1
           
             finalTourTwoCombos(i,:) = tourTwoCombos(i,:);
        end
        % Ensure the tour doesn't go to a node, go to a 1, and then go 
        % back to a node. Ex. [1 10 8 1 9 1]. We have to get rid of those
        % trades because they're not allowed by the constraints.
    end
    if  numel(totalRobotTours(totalRobotTours(:,:,2) ~= 1)) == max(numSlots) -2 
        finalTourTwoCombos = totalRobotTours(:,:,2);
    end
     
    % ROBOT 3
    for i = 1:numel(tourThreeCombos(:,1))     
        if sum((ismember(tourThreeNoOnes,tourThreeCombos(i,:)))) == numel(tourThreeNoOnes) ...
                && tourThreeCombos(i,1) == 1 && tourThreeCombos(i,end) == 1
            
              finalTourThreeCombos(i,:) = tourThreeCombos(i,:);
        end
    end
    if  numel(totalRobotTours(totalRobotTours(:,:,3) ~= 1)) == max(numSlots) -2 
        finalTourThreeCombos = totalRobotTours(:,:,3);
    end
    
    % Then, reshape the matrices to get rid of zero rows:
    finalTourOneCombos = finalTourOneCombos(any(finalTourOneCombos,2),:);
    finalTourTwoCombos = finalTourTwoCombos(any(finalTourTwoCombos,2),:);
    finalTourThreeCombos = ...
                     finalTourThreeCombos(any(finalTourThreeCombos,2),:);
    % Honestly, I'm not sure why the code above works. Here's the link
    % to where I got it: 
    % https://www.mathworks.com/matlabcentral/answers/
    % 40390-remove-rows-with-all-zeros
    
    % Finally, turn these tours into  ColIdxs to make them work with
    % other code:
    finalTourOneCombos = finalTourOneCombos(:,2:end);
    finalTourTwoCombos = finalTourTwoCombos(:,2:end);
    finalTourThreeCombos = finalTourThreeCombos(:,2:end);

end

% GET ROBOT TOUR INFO FUNCTION
% 
function [RobotDistances, RobotRowIdxs, RobotColIdxs] = ...
    getAllRobotTourInfo(C, numOfCities, numOfRobots, x2)

    % Make a multi-dimensional MATLAB arrray to store the output of
    % the multi-dim. binvar matrix. Imagine a bunch of sheets of paper
    % on top of each other. Each sheet of paper is a binvar matrix
    % for a certain robot.
    robotTours = ones(numOfCities, numOfCities, numOfRobots);
    % Fill up the array
    for i = 1:numOfRobots
        robotTours(:,:,i) = value(x2(:,:,i));
    end

    % Preallocate a multi-dimensional array to all robots' traveled routes
    % in the form of row and column indices
    RobotColIdxs = ones(1, numOfCities, numOfRobots);
    RobotRowIdxs = ones(1, numOfCities, numOfRobots);
    % Preallocate an array to collect the number of nodes each robot
    % travels. Will be used later to trim the RobotColIdxs array
    numberOfNodesTraveled = ones(1,numOfRobots);

    % Now, start getting the actual solved values for the robots' tours
    for i = 1:numOfRobots
        % Run function
        [toursDistances, colIdxs, rowIdxs] = ...
            buildTourList(C,robotTours(:,:,i),numOfCities);
        % Drop in the extracted col and row indices for a specific robot 
        RobotColIdxs(1,1:length(colIdxs),i) = colIdxs;
        RobotRowIdxs(1,1:length(colIdxs),i) = rowIdxs;
        % record the length of the robot's tour
        numberOfNodesTraveled(i) = length(colIdxs);
    end


    % Now shave down the matrix to only be as long as the longest
    % tour length taken by a specific robot:
    RobotColIdxs = RobotColIdxs(1,1:max(numberOfNodesTraveled),:);
    RobotRowIdxs = RobotRowIdxs(1,1:max(numberOfNodesTraveled),:);

    % Get RobotDistances. RobotDistances is a vector that tells you the 
    % distance between two robots at each city. For example, the first entry
    % tells you the distance between Robot 1 and Robot 2 when Robot 1 is 
    % at city 4 and Robot 2 is at city 2.
    % We have the indices of these distance values from the colIdxs vectors.
    
    % Ugly bit of code here. Haven't figured out how to cover all
    % combinations of robot distances yet in a nice way. This code
    % covers robot distance combinations for three robots. ONLY works
    % for three robots. Yes, I know it's bad.
    
    % New way to do it. It turns out we can assume the robots exist
    % as a chain. So we only need to care about Distance(robot1,robot2)
    % and Distance(robot2,robot3).
    
    distanceTwoRobots = [];
    for i = 1:length(RobotColIdxs(1,:,1))
        distanceTwoRobots = [distanceTwoRobots, ...
            C(RobotColIdxs(1,i,1), RobotColIdxs(1,i,2))];
    end
    RobotDistances(1,:) = distanceTwoRobots;
%     display(RobotDistances);
    
    
    distanceTwoRobots = [];
    for i = 1:length(RobotColIdxs(1,:,1))
        distanceTwoRobots = [distanceTwoRobots, ...
            C(RobotColIdxs(1,i,1), RobotColIdxs(1,i,3))];
    end
    RobotDistances(2,:) = distanceTwoRobots;
%     display(RobotDistances);
%     
    
    distanceTwoRobots = [];
    for i = 1:length(RobotColIdxs(1,:,2))
        distanceTwoRobots = [distanceTwoRobots, ...
            C(RobotColIdxs(1,i,2), RobotColIdxs(1,i,3))];
    end
    RobotDistances(3,:) = distanceTwoRobots;
%     display(RobotDistances);

end

% To do next: Create a function with our test code. The function will
% take RobotColIdxs and spit out finalTourOneCombos, finalTourTwoCombos...
% The function will then turn the finalTourCombos into ColIdxs.

% We'll then make another function that ONLY gets RobotDistances. We'll
% copy the code for that from getAllRobotTourInfo. From there, we can
% finally go into the main function code. 

% In the main function code, we'll
% do what Yong said and cycle throuh each tour with the four loops. In 
% the deepest layer of the for loop, we'll call the RobotDistances function
% And then do the comparison right there.

% TOUR LIST FUNCTION
% Takes where the robot went in terms of its tour and puts it
% in one ordered list. Used for comparing tour lengths in the 
% meat and potatoes of the code.
function[tourVector, colIndex, rowIndex] = ...
    buildTourList(costMatrix, binVarMatrix, matrixSize)

    tourVector = [];
    tourMatrix = costMatrix.*binVarMatrix;
    rowIndex = [];
    colIndex = [];
    
    row = 1;
    newRow = 0;

    while(newRow ~= 1)
        [tourEntry, newRow] = getOneEntry(tourMatrix, row, matrixSize);
        tourVector = [tourVector, tourEntry];
        % set current row equal to the next row. Lets us run the next
        % iteration.
        % Intuitively, we're following the robot to its next city.
        row = newRow(end);
        % Also, record the row and column indecies:
        colIndex = [colIndex newRow(end)];
    end
   
    colIndex = [colIndex];
    rowIndex = [1 colIndex(1:end-1)]; 
        
end

% GET_ENTRY FUNCTION
% Finds the entry in the binvar matrix and records its location
function[tourEntry, newRow] = getOneEntry(tourMatrix, oldRow, matrixSize)

    i = oldRow;
    for j = 1:matrixSize % doesn't change
        if (tourMatrix(i,j) ~= 0)
            % record value
            tourEntry = tourMatrix(i,j);
            row = i;
            col = j;
            % Grab the col index and assign is to newRow. That way,
            % we'll jump to that row next.
            newRow = col;
        end
    end
        
end


% PLOTTING FUNCTION

% nothing to return here
function[] = plotRoute(nodecoords, rowIndex, colIndex, color) 

    % first plot just the node coordinates by themselves. Code influenced
    % by the "I want to create a plot using XY coordintes" post on 
    % the MathWorks forums.
    x = nodecoords(:,2);
    y = nodecoords(:,3);
    n = numel(x);
    plot(graph(1:n,1:n), 'LineStyle', 'none', 'Marker', 'd', ...
       'NodeColor','black','XData',x,'YData',y);

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
        % x components of point 1 and 2, 
        % then y components of point 1 and 2:
        plot([point1(1) point2(1)], [point1(2) point2(2)], string(color))
        
        hold on
        pause(0.1);
        drawnow
    end
    
%     title("Robot Tours: 60 Unit Tether, MinMax Objective");
%     title("Robot Tours: mTSP Formulation, Tethered, 2_norm Squared");
%     xlabel("x distance (arbitrary units)");
%     ylabel("y distance (arbitrary units)");
    
    
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
function [] = plotTethers(nodecoords, RobotRowIdxs,RobotColIdxs, ...
                          r1, r2, color, lineStyle)

    
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
              string(color), 'LineStyle', char(lineStyle),'LineWidth', 1.5)

        hold on
        pause(0.1);
        drawnow

    end
end


end

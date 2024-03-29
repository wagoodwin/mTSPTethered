%% Background

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

clc; 
clear all;
yalmip('clear');

% If flagIterative == 1, the iterative method will run. Otherwise, only the
% original simulation will run.
flagIterative = 1;
% If flagTether == 1, the sim will run while incorporating tether
% constraints.
flagTether = 1;
if flagIterative == 1
    flagTether = 1;
end

% TODO: CONSOLIDATE FLAGITERATIVE AND FLAGTETHER INTO ONE FLAG

% One more thing to add: a flag to designate what kind of norm we're using
flagNorm = "two"; % OPTIONS: "two-squared", "one", "two"

numOfRobots = 3;
numOfCities = 6;
tetherLength = 50;

% Truncated eil51 node coords further to just 5 cities
% nodecoords = load('ToyProblemNodeCoords.txt');
nodecoords = load('TruncatedEil51NodeCoords.txt');
nodecoords = nodecoords(1:numOfCities,:);

C = zeros(numOfCities);

for i = 1:numOfCities
    for j = 1:numOfCities
        if(strcmp(flagNorm,"one") == 1)
           C(i,j) = distance1(nodecoords(i,2), nodecoords(i,3), ...
           nodecoords(j,2), nodecoords(j,3));
        end
        
        if(strcmp(flagNorm,"two") == 1 || ...
                strcmp(flagNorm,"two-squared") == 1)
            C(i,j) = distance(nodecoords(i,2), nodecoords(i,3), ...
            nodecoords(j,2), nodecoords(j,3));
        end

    end
end

C = real(C);
if(strcmp(flagNorm,"two-squared") == 1)
    C = C.^2; % We're using the sum of squared distances (SSD) as opposed 
    % to the raw 2-norm to stay consistent between the two approaches
end



%% mTSP constraints

x2 = binvar(numOfCities, numOfCities, numOfRobots, 'full');

u = sdpvar(numOfCities, 1, 'full');
p = numOfCities - numOfRobots;

% ConstraintS 1 and 2 (eqns 10 and 11)
constraint1_2 = [];
constraint2_2 = [];
x1jkTotal = 0;
xj1kTotal = 0;
for k = 1:numOfRobots
    x1jkTotal = 0;
    xj1kTotal = 0;
    for j = 2:numOfCities
        x1jkTotal = x1jkTotal + x2(1,j,k);
        xj1kTotal = xj1kTotal + x2(j,1,k);
    end
    constraint1_2 = [constraint1_2, (x1jkTotal == 1)];
    constraint2_2 = [constraint2_2, (xj1kTotal == 1)];
end


% Constraint 3 (eqn 12)
% First, build the constraint vector:
constraint3_2 = [];
% Then, build a vector to hold the sum of all of 
% the values of each numOfCities x numOfRobots matrix.
% See handwritten work for a more intuitive visualiztion.
% planeTotal_ik = zeros(1, numOfCities, 1);
planeTotal_ik = 0;
for j = 2:numOfCities
    % reset the value of planeTotal_ik to 0 for the next constraint:
    planeTotal_ik = 0;
    for i = 1:numOfCities
        for k = 1:numOfRobots
            if i ~=j
%                 planeTotal_ik(1,j,1) = planeTotal_ik(1,j,1) + x(i,j,k);
                  planeTotal_ik = planeTotal_ik + x2(i,j,k);
            end
            
        end
    end
    % Build the constraint here
    constraint3_2 = [constraint3_2, (planeTotal_ik == 1)]; % legal? I think so 
    
end


% New issue: we can't assign an sdpvar to a vector. As such,
% the loop I had for constraint3 had to be reworked such that we're
% only storing the planeTotal_ik value as a single variable.


% Constraint 4 (eqn 13)
constraint4_2 = [];
% planeTotal_jk = zeros(numOfcities, 1, 1);
planeTotal_jk = 0;
for i = 2:numOfCities
    planeTotal_jk = 0;
    for j = 1:numOfCities
        for k = 1:numOfRobots
            if i ~=j
%                 planeTotal_jk(i,1,1) = planeTotal_jk(i,1,1) + x(i,j,k);
                  planeTotal_jk = planeTotal_jk + x2(i,j,k);
            end
            
        end
    end
    constraint4_2 = [constraint4_2, (planeTotal_jk == 1)];
    
end

% Constraint 5 (eqn 14)
% lhsSum = zeros(1,numOfCities, numOfRobots);
% rhsSum = zeros(numOfCities, 1, numOfRobots);
lhsSum = 0;
rhsSum = 0;
constraint5_2 = [];
% Build LHS sum (ignore this comment now)
for j = 2:numOfCities
  
    for k = 1:numOfRobots
        for i = 1:numOfCities
            if i ~= j
%                 lhsSum(1,j,k) = lhsSum(1,j,k) + x(i,j,k);
                  lhsSum = lhsSum + x2(i,j,k);
                  rhsSum = rhsSum + x2(j,i,k);
            end
        end
        constraint5_2 = [constraint5_2, (lhsSum == rhsSum)];
        % 0 out sums for next constraint
        lhsSum = 0;
        rhsSum = 0;
        %constraint5 = [constraint5, lhsSum(1,j,k) == 1];
    end
    
      
end


% Subtour elimination constraint (SEC)
%xS = sum(x,3); % p*x(i,j) becomes p*sum(x,3)? Should make a matrix of
% numOfCities by numOfCities, where each entry is the sum of all of the
% binvars for each salesman at each curve length.

% Wait: xS = sum(x,3) won't work because we won't be able to rule out
% situations where i == j.
xS = 0;

constraint6_2 = [];
for j = 2:numOfCities
    
    for i = 2:numOfCities
        
        for k = 1:numOfRobots
            xS = xS + x2(i,j,k);

            if i ~= j
%             constraint5 = [constraint5, u(i)-u(j) + p*x(i,j) <= p -1];
              constraint6_2 = [constraint6_2, (u(i)-u(j) + p*xS <= p -1)];
            end
            
            xS = 0; % reset to 0 for next iteration of constraint 
        end

    end
    
end

constraint7_2 = [];
for i = 1:numOfRobots
    constraint7_2 = [constraint7_2, trace(x2(:,:,i)) == 0];
end


% Complete constraints
constraints_2 = [constraint1_2, constraint2_2, constraint3_2, ...
                 constraint4_2, constraint5_2, ...
                 constraint6_2, constraint7_2];
 
% Objectives
SalesmanDistances = [];
for i = 1:numOfRobots
    SalesmanDistances = [SalesmanDistances sum(sum(C.*x2(:,:,i)))];
end


objective1 = sum(SalesmanDistances); % Minimize total distance of all tours
objective2 = max(SalesmanDistances); % Minimize max individual robot tour
% This objective is confirmed to give us the same result as the 
% first formulation. Absolutely massive.


%% Solve Original System

if (flagIterative == 0)

    tic 
    options = sdpsettings('verbose',0,'solver','Gurobi');
    sol = optimize(constraints_2,objective2,options);
    value(objective2)
    value(x2)
    toc

    [RobotDistances, RobotRowIdxs, RobotColIdxs] = ...
        getAllRobotTourInfo(C, numOfCities, numOfRobots, value(x2));
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



tic % to see how long the sim takes
while(1)

    % run simulation
    % Set 'verbose' to 0 when timing the sim
    options = sdpsettings('verbose',1,'solver','Gurobi');
    sol = optimize(constraints_2,objective2,options);
    sol.problem;
    value(objective2);
    value(x2);

    % Record previous values of the max tour length of some robot
    pastAnswers = [pastAnswers value(objective1)];

%     [RobotDistances, RobotRowIdxs, RobotColIdxs] = ...
%         getAllRobotTourInfo(C, numOfCities, numOfRobots, value(x2));
       
    
    [RobotDistances, RobotRowIdxs, RobotColIdxs] = ...
       getAllRobotTourInfo(C, numOfCities, numOfRobots, value(x2));
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
            
                if(flagNorm == "two-squared")
                    % Or what we could do is just say, "if this solution
                    % satisfies the tether constraint, keep it and you're 
                    % done." Like this:
                    if ( sum(sum(RobotDistancesTwo(1:2,:) > tetherLength^2)) == 0  ||...
                        sum(sum(RobotDistancesTwo([1,3],:) > tetherLength^2)) == 0  ||...
                        sum(sum(RobotDistancesTwo([2,3],:) > tetherLength^2)) == 0 )

                        flag = 1;
                    end 
                end
                
                if(flagNorm == "two" || flagNorm == "one")
                    if ( sum(sum(RobotDistancesTwo(1:2,:) > tetherLength)) == 0  ||...
                        sum(sum(RobotDistancesTwo([1,3],:) > tetherLength)) == 0  ||...
                        sum(sum(RobotDistancesTwo([2,3],:) > tetherLength)) == 0 )

                        flag = 1;
                    end 
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
        
        % solution binvar sometimes is cast into an sdpvar somehow. In
        % that case, exclude the value with the second method shown in the
        % catch block:
        try 
            for k = 1:numOfRobots
                 constraints_2 = [constraints_2, exclude(x2(:,:,k), ...
                 value(x2(:,:,k)))];
            end
        catch
            for ii = 1:numOfRobots
                constraints_2 = [constraints_2, ...
                               (x2(:,:,ii) ~= value(x2(:,:,ii))) ];
            end
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

toc

end

%


%% Plotting 

% We want to visualize the paths of the robots. We could
% do this by drawing lines of the robots' paths on a grid. 
% The robots would have different colored-paths to help distinguish
% the paths. This strategy would work for a low number of robots only,
% but the project will involve only a low number of robots anyway, so 
% no problem.

figure(1)
box on
plotRoute(nodecoords, RobotRowIdxs(1,:,1) , RobotColIdxs(1,:,1), ...
    'red', flagNorm, flagTether)
hold on
plotRoute(nodecoords, RobotRowIdxs(1,:,2) , RobotColIdxs(1,:,2), ...
    'magenta', flagNorm, flagTether)
plotRoute(nodecoords, RobotRowIdxs(1,:,3) , RobotColIdxs(1,:,3), ...
    'blue', flagNorm, flagTether)

% 1,2,3 specifcy the two robots with which you want the tethers drawn
if (flagTether == 1)
    
    % Another thing we'll need to do: we'll need to determine how to
    % automatically draw the correct tethers between the robots. This task
    % shouldn't be too hard once we figure out how to generalize the algo
    % to any number of robots.
    
    plotTethers(nodecoords, RobotRowIdxs, ...
        RobotColIdxs, 1, 2, 'black', '--')
    plotTethers(nodecoords, RobotRowIdxs, ...
        RobotColIdxs, 2, 3, 'black', ':' )
    hold off
    
end


% Set text and axes to LaTeX-interpreted characters, and assign current,
% and save plot onto computer:
set(0,'defaulttextinterpreter','latex');

axes = gca;

axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.TickLabelFormat  = '\\textbf{%g}';

axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.TickLabelFormat = '\\textbf{%g}';

% exportgraphics(gca, ...
%     '/home/walter/Desktop/ThesisFigures/Naive/Tether70/naive_unteth_2.jpg', 'Resolution', '1000')


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

% GET ONLY ROBOT DISTANCES
function [RobotDistancesTwo] = getOnlyDistances(RobotColIdxs,C)

    distanceTwoRobots = [];
    for i = 1:length(RobotColIdxs(1,:,1))
        distanceTwoRobots = [distanceTwoRobots, ...
            C(RobotColIdxs(1,i,1), RobotColIdxs(1,i,2))];
    end
    RobotDistancesTwo(1,:) = distanceTwoRobots;
%     display(RobotDistancesTwo);
    
    
    distanceTwoRobots = [];
    for i = 1:length(RobotColIdxs(1,:,1))
        distanceTwoRobots = [distanceTwoRobots, ...
            C(RobotColIdxs(1,i,1), RobotColIdxs(1,i,3))];
    end
    RobotDistancesTwo(2,:) = distanceTwoRobots;
%     display(RobotDistancesTwo);
    
    
    distanceTwoRobots = [];
    for i = 1:length(RobotColIdxs(1,:,2))
        distanceTwoRobots = [distanceTwoRobots, ...
            C(RobotColIdxs(1,i,2), RobotColIdxs(1,i,3))];
    end
    RobotDistancesTwo(3,:) = distanceTwoRobots;
%     display(RobotDistancesTwo);

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
function[] = plotRoute(nodecoords, rowIndex, colIndex, color, ...
    flagTether, flagNorm) 

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
    
%     title("Robot Tours: Naive Method, Untethered, 1-norm");
%     title("Robot Tours: Naive Method, " + ...
%     num2str(flagTether) + ", " + num2str(flagNorm));
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



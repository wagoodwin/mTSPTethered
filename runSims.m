%% Setup

numOfRobots = 3;
numOfCities = 10;
flagNorm = "2-norm";



%% Generate Random Cities

% Generate node coordinates
boxScale = 1.2; % random scaling factor for "box" size
sz = [numOfCities 2];
randomCoords = ...
    [(1:1:numOfCities)', unifrnd(0,tetherLength*boxScale, sz)]; 
    % unifrnd samples from a uniform random variable between two points. We
    % use this command to get our random node coordinates

C = zeros(numOfCities);

for i = 1:numOfCities
    for j = 1:numOfCities   
        
        if( (strcmp(flagNorm,"1-norm") == 1) )
            C(i,j) = distance1(randomCoords(i,2), randomCoords(i,3), ...
            randomCoords(j,2), randomCoords(j,3));
        end
        
        if ( (strcmp(flagNorm,"2-norm Squared") == 1) || ...
                (strcmp(flagNorm,"2-norm") == 1) )
             C(i,j) = distance(randomCoords(i,2), randomCoords(i,3), ...
             randomCoords(j,2), randomCoords(j,3));
        end
        
    end
end
  
C = real(C); 



%% Run Loop


% get outputs from timed Method and save them in a .mat file
 results = ...
     runTimedMethod(nodeCoords,numOfCities,numOfRobots,tetherLength,norm)

 
 
%  save("fileWIthData.mat","flagIsFeasible","anyOtherVariables")
 
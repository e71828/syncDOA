%% Track with Five Sensors
% In this section, detections from all five sensors are used for tracking
% and a measurement noise of 2 degrees squared is used.
[detectionBuffer,truthLog,theaterDisplay] = helperGenerateStaticFusionScenarioData;
% keyboard;
showScenario(theaterDisplay); 
measNoise = 2; % Same noise as 3 sensors
numSensors = 3;
[trackingMetrics,errorMetrics] = helperRunStaticFusionSimulation(detectionBuffer,truthLog,numSensors,measNoise,theaterDisplay,true);
axes(theaterDisplay.TheaterPlot.Parent);
%%
% ylim([0 1.5]);

%%
% The assignment results from tracking using five sensors show that all
% truths were assigned a track during the entire simulation. There were
% also no track drops in the simulation as compared to 4 track drops in the
% low-accuracy three sensor simulation.
assignmentTable = trackMetricsTable(trackingMetrics);
assignmentTable(:,{'TrackID','AssignedTruthID','TotalLength','FalseTrackStatus'})

%%
% The estimated errors for positions are much lower for each true target as
% compared to the three sensor simulation. Notice that the estimation
% results for position and velocity do degrade as compared to three sensors
% with high-accuracy measurements.
disp(cumulativeTruthMetrics(errorMetrics))
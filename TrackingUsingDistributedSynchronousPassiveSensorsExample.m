%% Tracking Using Distributed Synchronous Passive Sensors
% This example illustrates the tracking of objects using measurements from
% spatially-distributed and synchronous passive sensors. In the
% <docid:fusion_ug#mw_fc76d4f5-2549-41ae-a719-a88f09c787f6 Passive
% Ranging Using a Single Maneuvering Sensor>, you learned that passive
% measurements provide incomplete observability of a target's state and how
% a single sensor can be maneuvered to gain range information.
% Alternatively, multiple stationary sensors can also be used to gain
% observability. In this example, you will learn how to track multiple
% objects by fusing multiple detections from passive synchronous sensors.

% Copyright 2018-2021 The MathWorks, Inc.
%% Introduction
% In the synchronized multisensor-multitarget tracking problem, detections
% from multiple passive sensors are collected synchronously and are used to
% estimate the following:
%
% * Number of targets in the scenario
% * Position and velocity of those targets
%
% This example demonstrates the use of the _Static Fusion Before Tracking_ [1]
% architecture for tracking using passive measurements. The _Static
% Fusion_ part of the architecture aims to triangulate the most likely set
% of detections and output fused detections containing estimated positions
% of targets. As measurements need to be fused together by static
% fusion, the sensors must report measurements synchronously.
%
% <<../StaticFusionBeforeTracking.PNG>>
%
% With measurements containing only line-of-sight (LOS) information, at
% least 2 sensors are needed to find the position. However, with 2 sensors,
% the problem of ghosting (intersections at points with no targets) occurs
% when multiple targets lie in the same plane. With 2 targets and 2
% sensors, it is impossible to identify the correct pair from a single
% frame of measurements as demonstrated in the figure below:
%
% <<../GhostLOSIntersections.PNG>>
%
% Therefore, one must use 3 or more sensors to reduce the problem of
% ghosting. Due to the presence of measurement noise and false
% measurements, it is difficult to eliminate the problem of ghosting
% completely. Ghost triangulations returned by static association are
% likely to be discarded by the dynamic association blocks as the geometry
% of targets and sensors changes during the scenario.

%% Define Scenario
% The relative placement of sensors and targets in the scenario used here
% is taken from an example in [1]. The scenario consists of five
% equally-spaced targets observed by three to five passive sensors. The
% passive detections are modeled using |radarEmitter| and 
% |fusionRadarSensor| with |DetectionMode| set to |ESM|. The |HasNoise| 
% property of the sensors is set to |false| to generate noise-free 
% detections along with false alarms. Noise is added to measurements in 
% this example via a user-controlled variable. This is to simulate the 
% effect of sensor noise on static fusion. Each sensor has a field of view 
% of 180 degrees in azimuth and a |FalseAlarmRate| of 1e-3 per azimuth 
% resolution cell. This results in 2 to 3 false alarms per scan. The 
% scenario definition is wrapped inside the helper function 
% |helperGenerateFusionScenarioData|.

[detectionBuffer,truthLog,theaterDisplay] = helperGenerateStaticFusionScenarioData;
showScenario(theaterDisplay);
%%
showGrabs(theaterDisplay,[]);

%% Track with Three Sensors
% In this section, only measurements from the inner three sensors are
% considered and measurement noise covariance for each sensor is set to
% 0.01 degrees squared.
%
% The detections from each sensor are passed to a |staticDetectionFuser|.
% The |MeasurementFusionFcn| for passive triangulation is specified as
% |triangulateLOS|. The |MeasurementFusionFcn| allows specifying a function
% to fuse a given combination of detections (at most one detection from
% each sensor) and return the fused position and its error covariance. The
% parameters |FalseAlarmRate|, |Volume| and |DetectionProbability| are
% specified to reflect the parameters of sensors simulated in this
% scenario. These parameters are used to calculate the likelihood of
% feasible associations. The |UseParallel| property, when set to |true|,
% allows the fuser to evaluate the feasible associations using parallel
% processors.
%
% The tracking is performed by GNN data association by using a |trackerGNN|.
%
% The tracking performance is evaluated using |trackAssignmentMetrics| and
% |trackErrorMetrics|.
%
%% 
% *Setup*

% Number of sensors
numSensors = 3;

% Create a detection fuser using triangulateLOS function as the
% MeasurementFusionFcn and specify parameters of sensors.
fuser = staticDetectionFuser('MeasurementFusionFcn',@triangulateLOS,...
    'MaxNumSensors',numSensors,...
    'UseParallel',true,...
    'FalseAlarmRate',1e-3,...
    'Volume',3,...
    'DetectionProbability',0.99);

% Tracking using a GNN tracker
tracker = trackerGNN('AssignmentThreshold',45,...
    'ConfirmationThreshold',[3 5],'DeletionThreshold',[4 5]);

% Use assignment and error metrics to compute accuracy.
trackingMetrics = trackAssignmentMetrics('DistanceFunctionFormat','custom',...
    'AssignmentDistanceFcn',@trueAssignment,'DivergenceDistanceFcn',@trueAssignment);
errorMetrics = trackErrorMetrics;

%%
% *Run simulation with three sensors*

% Measurement noise
measNoise = 0.01;

time = 0; % simulation time
dT = 1; % 1 Hz update rate of scenario.

% Loop through detections and track targets
for iter = 1:numel(detectionBuffer)
    
    % Truth information
    sensorPlatPoses = truthLog{iter}(1:numSensors);
    targetPlatPoses = truthLog{iter}(6:end);
    groundTruth = [sensorPlatPoses;targetPlatPoses];
    
    % Generate noisy detections using recorded detections
    thisBuffer = detectionBuffer{iter};
    availableDetections = vertcat(thisBuffer{1:numSensors});
    noiseDetections = addNoise(availableDetections,measNoise);
    
    % Fuse noisy detections using fuser
    fusedDetections = mexFuser_mex(noiseDetections);
    
    % Run a tracker on fused detections
    confTracks = tracker(fusedDetections,time);
    
    % Update track and assignment metrics
    trackingMetrics(confTracks,targetPlatPoses);
    [trackIDs,truthIDs] = currentAssignment(trackingMetrics);
    errorMetrics(confTracks,trackIDs,targetPlatPoses,truthIDs);
    
    % Update theater display
    detsToPlot = [noiseDetections(:);fusedDetections(:)];
    theaterDisplay(confTracks,detsToPlot,groundTruth);
    
    % Increment simulation time
    time = time + dT;
end
axes(theaterDisplay.TheaterPlot.Parent);
%%
% ylim([0 1.5]);
%%
% Results from tracking using three sensors with 0.01 degrees-squared of
% noise covariance can be summarized using the assignment metrics. Note
% that all tracks were assigned to the correct truths and no false tracks
% were confirmed by the tracker. These results indicates good static
% association accuracy.
%%
assignmentTable = trackMetricsTable(trackingMetrics);
assignmentTable(:,{'TrackID','AssignedTruthID','TotalLength','FalseTrackStatus'})

%%
% The error in estimated position and velocity of the targets can be
% summarized using the error metrics. The errors in position and velocity
% are within 7 meters and 2 meters/sec respectively for all targets and the
% normalized errors are close to 1. The error metrics indicate good dynamic
% association and tracking performance.
disp(cumulativeTrackMetrics(errorMetrics));

%%
%
% *Effect of measurement accuracy*
%
% The fusion of passive detections to eliminate ghosting is highly
% dependent on the accuracy of passive measurements. As measurement noise
% increases, the distinction between ghost associations and true
% associations becomes less prominent, resulting in a significant drop in
% the accuracy of static association. With closely spaced targets,
% incorrect association of fused detections to tracks may also occur. In
% the next section, a helper function |helperRunStaticFusionSimulation| is
% used to re-run the scenario with a measurement noise covariance of 2
% degrees squared.

%%
%
% Run the scenario again with a high measurement noise
numSensors = 3;
measNoise = 2; %standard deviation of sqrt(2) degrees
[trackingMetrics,errorMetrics] = helperRunStaticFusionSimulation(detectionBuffer,truthLog,numSensors,measNoise,theaterDisplay,false);
axes(theaterDisplay.TheaterPlot.Parent);
%%
% ylim([0 1.5]);
%%
% Note that a few tracks were confirmed and then dropped in this
% simulation. Poor static association accuracy leads to ghost target
% triangulations more often, which results in tracker deleting these tracks
% due to multiple misses.
assignmentTable = trackMetricsTable(trackingMetrics);
assignmentTable(:,{'TrackID','AssignedTruthID','TotalLength','FalseTrackStatus'})

%%
% The estimated error for each truth is higher. Notice that the track
% jumps in the theater display above.
disp(cumulativeTruthMetrics(errorMetrics));
%% Summary
% This example showed how to track objects using a network of distributed
% passive sensors. You learned how to use |staticDetectionFuser| to
% statically associate and fuse detections from multiple sensors. The example
% demonstrated how this architecture is depends parameters like number of
% sensors in the network and the accuracy of sensor measurements. The
% example also showed how to accelerate performance by utilizing parallel
% computing and automatically generating C code from MATLAB code.

%% Supporting Functions
% *|trueAssignment|* Use ObjectAttributes of track to assign it to the
% right truth.
function distance = trueAssignment(track,truth)
tIDs = [track.ObjectAttributes.TargetIndex];
tIDs = tIDs(tIDs > 0);
if numel(tIDs) > 1 && all(tIDs == truth.PlatformID)
    distance = 0;
else
    distance = inf;
end
end
%%
% *|addNoise|* Add noise to detections
function dets = addNoise(dets,measNoise)
for i = 1:numel(dets)
    dets{i}.Measurement(1) = dets{i}.Measurement(1) + sqrt(measNoise)*randn;
    dets{i}.MeasurementNoise(1) = measNoise;
end
end

%% References
% [1] Bar-Shalom, Yaakov, Peter K. Willett, and Xin Tian. "Tracking and
% Data Fusion: A Handbook of Algorithms." (2011).

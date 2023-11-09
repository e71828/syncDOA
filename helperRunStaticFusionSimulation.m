function [trackingMetrics, errorMetrics] = helperRunStaticFusionSimulation(detectionBuffer,truthLog,numSensors,measNoise,theaterDisplay,isMex)
% This is a helper function to run the static fusion simulation and may be
% removed in a future release without notice.
% 
% Copyright 2018 The MathWorks, Inc.

if nargin == 3
    isMex = false;
end

theaterDisplay.TheaterPlot.XLimits = [-2 2];
theaterDisplay.TheaterPlot.YLimits = [-2 2];

% Clear track history
tp = theaterDisplay.TheaterPlot;
if numel(tp.Plotters) > 4
    tp.Plotters(5).clearData;
end
% Create a detection fuser using |triangulateLOS| function as the
% |MeasurementFusionFcn| and specify parameters of sensors.
if ~isMex
    fuser = staticDetectionFuser('MeasurementFusionFcn',@triangulateLOS,...
        'MaxNumSensors',numSensors,...
        'UseParallel',true,...
        'FalseAlarmRate',1e-3,...
        'Volume',0.0716,...
        'DetectionProbability',0.99);
else
    fuser = @mexFuser_mex;
end
% Tracking using a GNN tracker
tracker = trackerGNN('AssignmentThreshold',50,...
    'ConfirmationThreshold',[3 5],'DeletionThreshold',[4 5]);

% Use track assignment and error metrics to compute accuracy.
trackingMetrics = trackAssignmentMetrics('DistanceFunctionFormat','custom',...
    'AssignmentDistanceFcn',@trueAssignment,'DivergenceDistanceFcn',@trueAssignment);

errorMetrics = trackErrorMetrics;

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
    for index = 1:numel(thisBuffer)
        thisBuffer{index}= thisBuffer{index}(1:5);
    end
    availableDetections = vertcat(thisBuffer{1:numSensors});
    noiseDetections = addNoise(availableDetections,measNoise);
    
    % Fuse noisy detections using fuser
    fusedDetections = fuser(noiseDetections);
    
    % Run a tracker on fused detections
    confTracks = tracker(fusedDetections,time);
    if iter == 55 || iter == 80
        keyboard;
    end
    
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

end

%%
% *|trueAssignment|* 
% Use ObjectAttributes of track to assign it to the right truth.
function distance = trueAssignment(track,truth)
    tIDs = [track.ObjectAttributes.TargetIndex];
    tIDs = tIDs(tIDs > 0);
    if all(tIDs == truth.PlatformID)
        distance = 0;
    else
        distance = inf;
    end
end

function dets = addNoise(dets,measNoise)
for i = 1:numel(dets)
    dets{i}.Measurement(1) = dets{i}.Measurement(1) + sqrt(measNoise)*randn;
    dets{i}.MeasurementNoise(1) = measNoise;
end
end
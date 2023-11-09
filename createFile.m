[detectionBuffer,truthLog] = helperGenerateStaticFusionScenarioData;
close all
numSensors = 5;
measNoise = 0.01;
% Tracking using a GNN tracker
tracker = trackerGNN('AssignmentThreshold',50,...
    'ConfirmationThreshold',[3 5],'DeletionThreshold',[4 5]);

% Use track assignment and error metrics to compute accuracy.
trackingMetrics = trackAssignmentMetrics('DistanceFunctionFormat','custom',...
    'AssignmentDistanceFcn',@trueAssignment,'DivergenceDistanceFcn',@trueAssignment);


% fileID = fopen('simple_noiseDetections.in', 'w');
% fileID2 = fopen('simple_fusedDetections.in', 'w');

% fileID = fopen('hard_noiseDetections.in', 'w');
% fileID2 = fopen('hard_fusedDetections.in', 'w');

fileID = fopen('naive_noiseDetections.in', 'w');
fileID2 = fopen('naive_fusedDetections.in', 'w');

for iter = 1:numel(detectionBuffer)
    thisBuffer = detectionBuffer{iter};
    for index = 1:numel(thisBuffer)
        thisBuffer{index}= thisBuffer{index}(1:5);
    end
    availableDetections = vertcat(thisBuffer{1:numSensors});
    noiseDetections = addNoise(availableDetections,measNoise);
    fuser = @mexFuser_mex;
    fusedDetections =  fuser(noiseDetections);
        % Run a tracker on fused detections
    [confTracks,unconfirmed]= tracker(fusedDetections,iter-1);
    for i = 1:numel(noiseDetections)
        det = noiseDetections{i};
        fprintf(fileID, "%2d ", det.Time);
        % fprintf(fileID, "%2d ", det.ObjectAttributes{1}.TargetIndex);
        fprintf(fileID, "%2d ", mod(i, 5) + (mod(i, 5) == 0) * 5);
        fprintf(fileID, "%2d ", det.SensorIndex);
        fprintf(fileID, "%12.8f ", -det.Measurement(1));
        fprintf(fileID, "%12.9f ", det.MeasurementNoise(1));
        fprintf(fileID, "%9.3f %9.3f ",det.MeasurementParameters(2).OriginPosition(1:2));
        fprintf(fileID, "%9.3f %9.3f\n",det.MeasurementParameters(2).OriginVelocity(1:2));
    end

    if iter< 5
        for j = 1:numel(fusedDetections)
            det = fusedDetections{j};
            fprintf(fileID2, "%2d ", det.Time);
            fprintf(fileID2, "%9.3f %9.3f ", det.Measurement(1:2));
            fprintf(fileID2, "%12.9f %12.9f %12.9f\n", det.MeasurementNoise([1 2 5]));
        end
    else
        for j = 1:numel(confTracks)
            det = confTracks(j);
            fprintf(fileID2, "%2d ", det.UpdateTime);
            fprintf(fileID2, "%9.3f %9.3f ", det.State([1 3]));
            fprintf(fileID2, "%12.9f %12.9f %12.9f\n", det.StateCovariance([1 3 15]));
        end
    end
end
fclose(fileID);
fclose(fileID2);

thisBuffer = detectionBuffer{1};
availableDetections = vertcat(thisBuffer{1:numSensors});
noiseDetections = addNoise(availableDetections,measNoise);
for i = 1:5:numel(noiseDetections)
    det = noiseDetections{i};
    % fprintf( "%9.3f ", det.Measurement(1));
    fprintf( "%9.3f,%9.3f,\n",det.MeasurementParameters(2).OriginPosition(1:2));
end
for i = 1:numel(noiseDetections)
    det = noiseDetections{i};
    fprintf( "%5.1f, ", -det.Measurement(1));
    if mod(i,5)==0
        fprintf("\n");
    end
end
for i = 1:5:numel(noiseDetections)
    det = noiseDetections{i};
    fprintf( "%5.1f, ", -det.Measurement(1));
end

%% Supporting Functions
%%
% *|addNoise|* Add noise to detections
function dets = addNoise(dets,measNoise)
for i = 1:numel(dets)
    dets{i}.Measurement(1) = dets{i}.Measurement(1) + sqrt(measNoise)*randn;
    dets{i}.MeasurementNoise(1) = measNoise;
end
end

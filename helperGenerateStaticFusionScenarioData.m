function [detectionBuffer,truthLog,theaterDisplay] = helperGenerateStaticFusionScenarioData
% This is a helper function to generate ground truth data for the static 
% fusion simulation and may beremoved in a future release without notice.
% 
% Copyright 2018-2020 The MathWorks, Inc.

% Random seed for reproducible results
rng(2023);

[scene,theaterDisplay] = helperCreatePassiveDistributedScenario;

%% Generating Synthetic Data
% This section advances the scenario and generate detections from all
% sensors and records it in a buffer

% Detection buffer for recording
numIterations = scene.UpdateRate*scene.StopTime;
detectionBuffer = cell(numIterations,1);
truthLog = cell(numIterations,1);


% Record detections
iter = 0;
vv8 = norm(scene.Platforms{8}.Trajectory.Velocity);
vv10 = norm(scene.Platforms{10}.Trajectory.Velocity)*1.5;
vv6 = norm(scene.Platforms{6}.Trajectory.Velocity)*1.5;
vv7 = norm(scene.Platforms{7}.Trajectory.Velocity)*2;
vv9 = norm(scene.Platforms{9}.Trajectory.Velocity)*2;
while advance(scene)
    % scene 1
    % if iter <= 60
    %     scene.Platforms{6}.Trajectory.Velocity = vv6*[sind(iter*3) cosd(iter*3) 0];
    %     scene.Platforms{10}.Trajectory.Velocity = vv10*[-sind(iter*3) cosd(iter*3) 0];
    %     scene.Platforms{9}.Trajectory.Velocity = vv9*[cosd(45-iter/2) sind(45-iter/2) 0];
    %     scene.Platforms{7}.Trajectory.Velocity = vv7*[cosd(135+iter/2) sind(135+iter/2) 0];
    % end
    % if iter > 60
    %     scene.Platforms{6}.Trajectory.Velocity = vv6*[sind(iter*3) cosd(iter*3) 0];
    %     scene.Platforms{10}.Trajectory.Velocity = vv10*[-sind(iter*3) cosd(iter*3) 0];
    %     scene.Platforms{9}.Trajectory.Velocity = vv9*0.6*[cosd(45+iter-60) sind(45+iter-60) 0];
    %     scene.Platforms{7}.Trajectory.Velocity = vv7*0.6*[cosd(135-iter+60) sind(135-iter+60) 0];
    % end
    % 
    % if iter< 50
    %     scene.Platforms{8}.Trajectory.Velocity = vv8*[0 -1 0]*exp(-iter/60);
    % elseif iter < 70
    %     scene.Platforms{8}.Trajectory.Velocity = [3 0 0];
    % else
    %     scene.Platforms{8}.Trajectory.Velocity = vv8*[0 1 0]/exp(-iter/120);
    % end

    % scene2
    scene.Platforms{6}.Trajectory.Velocity = vv6/2*[cosd(60) sind(60) 0];
    scene.Platforms{10}.Trajectory.Velocity = vv10/2*[cosd(120) sind(120) 0];
    iter = iter + 1;
    dets = helperGenerateDetections(scene);
    detectionBuffer{iter} = dets;
    truthLog{iter} = platformPoses(scene);
end

end

%% Utility functions to create scenario.
function [ts,tp] = helperCreatePassiveDistributedScenario()

ts = trackingScenario;
ts.UpdateRate = 1;
ts.StopTime = 60;

esm = fusionRadarSensor(1, 'No scanning', ...
    'DetectionMode', 'ESM', ...
    'FieldOfView',[180 1],...
    'HasElevation',true,...
    'MountingAngles',[90 0 0],...
    'HasNoise',false,...
    'HasFalseAlarms',true,...
    'HasINS',true,...
    'FalseAlarmRate',1e-3,...
    'AzimuthResolution',3,...
    'UpdateRate',ts.UpdateRate,...
    'ElevationResolution',1);

r = 6e3;
numSensors = 5;
thetaSpan = pi/7*numSensors;
theta = linspace(-thetaSpan,thetaSpan,numSensors)/2 + pi/2;
xSen = r*cos(theta);
ySen = zeros(numSensors,1); %-r*sin(theta);
locationIDs = [4 3 2 5 1];

for i = 1:numSensors
    thisID = locationIDs(i);
    senPlat = platform(ts,'Trajectory',kinematicTrajectory('Position',...
        [xSen(thisID) ySen(thisID) 0],'Velocity',[0*rand 0*rand 0]));
    senPlat.Sensors = {clone(esm)};
    senPlat.Sensors{1}.SensorIndex = i;
end

% Create a large jammer for each target
jammer = radarEmitter(1, 'No scanning','FieldOfView',[360 180]);

numTargets = 5;
% Targets in a straight line
yTgt = 5e2*ones(1,numTargets)*5+1e3;
xTgt = linspace(-5000,5000,numTargets);

% Pick a velocity for each target in the plane.
velocity = [-5 5;
            -3 3;
            0 5;
            3 3;
            5 5]*10;

for i = 1:numTargets
    tgtPlat = platform(ts,'Trajectory',kinematicTrajectory('Position',...
        [xTgt(i) yTgt(i) 0],'Velocity',[velocity(i,:) 0]));
    tgtPlat.Emitters = {clone(jammer)};
end

tp = helperStaticFusionDisplay(ts,'XLim',[-8 8],'YLim',[-1 15]);

end

%% Generate detections
function [dets,configs] = helperGenerateDetections(ts,measNoise)
if nargin == 1
    measNoise = 0;
end
txSigs = [];
for platID = 1:numel(ts.Platforms)
    emitSigs = emit(ts.Platforms{platID},ts.SimulationTime);
    txSigs = [txSigs;emitSigs]; %#ok<AGROW>
end
% Generate detections from transmitted signals
dets = cell(0,1);
configs = [];
for platID = 1:numel(ts.Platforms)
    [thisDet,~,thisConfig] = detect(ts.Platforms{platID},txSigs,ts.SimulationTime);
    for i = 1:numel(thisDet)
        thisDet{i}.Measurement(1) = thisDet{i}.Measurement(1) + sqrt(measNoise)*randn;
        thisDet{i}.MeasurementNoise(1,1) = measNoise;
        thisDet{i}.MeasurementNoise(2,2) = 1e-4;
    end
    if ~isempty(thisDet)
        dets{end+1} = thisDet;  %#ok<AGROW>
        configs = [configs;thisConfig]; %#ok<AGROW>
    end
end
end

%%
% The association accuracy can be improved by increasing the number of
% sensors. However, the computational requirements increase exponentially
% with the addition of each sensor. The static fusion algorithm spends most
% of the time computing the feasibility of each triangulation. This
% part of the algorithm is parallelized when the |UseParallel| property of
% the |staticDetectionFuser| is set to |true|, which provides a linear
% speed-up proportional to the number of processors. To further accelerate
% execution, you can also generate C/C++ code which will also run in
% parallel execution on multiple processors. You can learn the basics of
% code generation using MATLAB(R) Coder(TM) at
% <docid:coder_doccenter#bs_d54l
% Get Started with MATLAB Coder>.

%% Accelerate MATLAB Code Through Parallelization and Code Generation
% To accelerate MATLAB Code for simulation, the algorithm must be
% restructured as a MATLAB function, which can be compiled into a MEX file
% or a shared library. For this purpose, the static fusion algorithm is
% restructured into a function. To preserve the state of the fuser between
% multiple calls, it is defined as a <docid:matlab_ref#f76-920859 persistent>
% variable.
%%
type('mexFuser');
load buffer.mat detectionBuffer
%%
% MATLAB(R) Coder(TM) requires specifying the properties of all the input
% arguments. An easy way to do this is by defining the input properties by
% example at the command line using the |-args| option. For more
% information, see <docid:coder_ug#bq9_obs-1 Define Input Properties by
% Example at the Command Line>. To allow variable number of detections, you
% will use the |coder.typeof| function to allocate data types and sizes for
% the inputs.


% Get a sample detection from the stored buffer
sampleDetection = detectionBuffer{1}{1}{1};

% Use the coder.typeof function to allow variable-size inputs for
% detections.
maxNumDets = 500;
inputDets = coder.typeof({sampleDetection},[maxNumDets,1],[1 0]);

if ~isfolder("codegen")
    h = msgbox({'Generating code for function. This may take a few minutes...';...
        'This message box will close when done.'},'Codegen Message');
    
    % Use the codegen command to generate code by specifying input arguments
    % via example by using the |-args| option.
    cfg = coder.config('dll');
    cfg.GenCodeOnly = true;
    cfg.TargetLang = 'C++';
    cfg.CppInterfaceStyle = 'Methods';
    cfg.CppInterfaceClassName = 'Fusion';
    cfg.GenerateExampleMain = 'GenerateCodeAndCompile'; 
    cfg.Toolchain = 'CMake'; 
    cfg.EnableOpenMP = false;
    hw_cfg = coder.HardwareImplementation;
    hw_cfg.ProdHWDeviceType = 'intel->x86-64 (Windows64)';
    cfg.HardwareImplementation = hw_cfg;
    codegen mexFuser -config cfg -args {inputDets} -report;
    codegen mexFuser -args {inputDets}
    close(h);
end

%%
% You can verify the speed-up achieved by code generation by comparing the
% time taken by them for fusing one frame of detections
testDetections = addNoise(vertcat(detectionBuffer{1}{1:5}),1);
tic;mexFuser(testDetections);t_ML = toc;
tic;mexFuser_mex(testDetections);t_Mex = toc;
disp(['MATLAB Code Execution time = ',num2str(t_ML)]);
disp(['MEX Code Execution time = ',num2str(t_Mex)]);

%%
% *|addNoise|* Add noise to detections
function dets = addNoise(dets,measNoise)
for i = 1:numel(dets)
    dets{i}.Measurement(1) = dets{i}.Measurement(1) + sqrt(measNoise)*randn;
    dets{i}.MeasurementNoise(1) = measNoise;
end
end
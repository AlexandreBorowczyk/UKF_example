% addpath('/home/aborowczyk/Documents/ukf_ws/src/ukf_example/matlab_gen/msggen')
% savepath
% /home/aborowczyk/.matlab/R2016b/javaclasspath.txt

% To use the custom messages, follow these steps:
%  
% 1. Edit javaclasspath.txt, add the following file locations as new lines, and save the file:
%  
% /home/aborowczyk/Documents/ukf_ws/src/ukf_example/matlab_gen/jar/ukf_example-0.0.0.jar
%  
% 2. Add the custom message folder to the MATLAB path by executing:
%  
% addpath('/home/aborowczyk/Documents/ukf_ws/src/ukf_example/matlab_gen/msggen')
% savepath
%  
% 3. Restart MATLAB and verify that you can use the custom messages. 
%    Type "rosmsg list" and ensure that the output contains the generated
%    custom message types.
clear all

global state command

try
    rosinit('aborowczyk-XPS')
catch
    rosshutdown
    rosinit('aborowczyk-XPS')
end

server = rossvcserver('ComputeCommandDip','ukf_example_srvs/ComputeCommand',@serviceCallback);
disp('SrvServer Running')

% rosshutdown
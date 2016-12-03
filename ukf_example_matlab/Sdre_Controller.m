addpath('/home/aborowczyk/Documents/ukf_ws/src/ukf_example/matlab_gen/msggen')
savepath

rosinit('aborowczyk-XPS')

try
server = rossvcserver('ComputeCommandDip','ukf_example_srvs/ComputeCommand',@serviceCallback)
catch exception
    rosshutdown
    error(exception)
end
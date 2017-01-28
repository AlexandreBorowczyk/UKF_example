function response = serviceCallback(server,reqmsg,defaultrespmsg)

format long

response = defaultrespmsg;
% Build the response message here

x = [reqmsg.X, reqmsg.Theta, reqmsg.Phi, reqmsg.DotX, reqmsg.DotTheta, reqmsg.DotPhi]';

u = 0;
try

u = ComputeCommand(x);
    
catch
    rosshutdown
end

response.Force = u;

end

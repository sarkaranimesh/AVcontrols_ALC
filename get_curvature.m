function md = get_curvature(Vx,Xref,Yref,time)
% get_curvature(Vx,yaw_rate_pose,Ts)
% radius from 

% Desired curvature
DX = gradient(Xref,0.1);
DY = gradient(Yref,0.1);
D2Y = gradient(DY,0.1);
curvature = DX.*D2Y./(DX.^2+DY.^2).^(3/2);


% md.time = time(1:length(curvature));
md.time = time;
md.signals.values = curvature;

end
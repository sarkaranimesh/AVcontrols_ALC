 %%
clc; close all; clear all;
%% velocity and time 
Vx = 10; %mps
Tf = 5; % time to complete lane change
Ts= 0.1;
alc_dir = -1; % R to L =+1  , L to R = -1
%% lane change path planning

path = alc_pathplanning(Vx,Tf,Ts,alc_dir);
xRef = path.xRef;
yRef= path.yRef;
yawRef = path.yawRef;
% yawRef(1,length(yawRef)+1) = yawRef(end);

T_lane = path.T_lane
tRef = 0:Ts:T_lane;

xRef_max = max(xRef);


%% Model parameters
m = 1140;
I_z = 1436.24; %kgm2
l_f = 1.165; 
l_r = 1.165;
c_f = 155494.663;
c_r = 1554494.663;

% assuming constant velocity
v_x = Vx; %mps
%% Dynamic bicycle model 
% continous A B C D matrixes
A = [0 1 Vx 0;
     0 -2*(c_f+c_r)/(m*v_x) 0 -2*(l_f*c_f + l_r*c_r)/(m*v_x) - v_x;
     0 0 0 1;
     0 2*(l_r*c_r - l_f*c_f)/(I_z*v_x)  0  -2*(l_r^2*c_r - l_f^2*c_f)/(I_z*v_x)];
 
B = [0;
    2*c_f/m;
    0;
    2*l_f*c_f/I_z];

C= [1 0 0 0;
       0 1 0 0];
D = [0;0];
%%
vehicle = ss(A,B,C,D)
load('mpc1.mat');

%% Overview of Simulink Model
% Open the Simulink model.
mdl = 'week9_alc';
open_system(mdl)

%% check for safe distance
safe_d = 2; % 2m safe distance between host and target vehicle
target_x = 500;
if (target_x - xRef_max) > 2
    % Run the model.
    out = sim(mdl);
    disp('Lane change performed')
else
    disp('Lane change cannot be performed')
end

bdclose(mdl)
%% plot results
lat_pose = out.lat_yaw_state(:,1);
yaw_pose = out.lat_yaw_state(:,2);
n = size(yaw_pose);

% match the reference input size to state output
tRefq = (linspace(0,T_lane,n(1)))';
xRefq = interp1(tRef,xRef,tRefq);
yRefq = interp1(tRef,yRef,tRefq);
yawRefq = interp1(tRef,yawRef,tRefq);

x_state = xRefq;
y_state = lat_pose;
yaw_state = yaw_pose;

all_data = [x_state y_state yaw_state xRefq yRefq];

figure(1)
plot(xRefq,yRefq,'Linewidth',2); hold on
plot(x_state,y_state, 'Linewidth',2); legend('reference','state')
xlabel('X (m)');ylabel('Y(m)'); grid on

        



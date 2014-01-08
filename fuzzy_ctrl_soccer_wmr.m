%% CI Class Homework WSS2013-14 
%% Fuzzy Inference Systems
%
% Trajectory tracking control for wheeled mobile robots in robot soccer
% the coordinates are given in a 300x300 [cm] field

% init
clear all; close all; clc;

% type of trajectory to track
% types of trajectories 
% {line, circle, sine, line_half_loop, complex1, complex2, stairs, center_switch}
tp = 'center_switch';
% field size size x size
field_size = 300;

%% Define the reference trajectory for the robot in the field

start_trajectory_generator(tp);

%% Simulate Fuzzy Control for Trajectory Tracking

%load the generated reference trajectory 
load(sprintf('reference_trajectory_data_%s.mat', tp));

% the control systems will use a TSK (Takagi-Sugeno-Kang FIS) such that the
% output is already given as a crisp format, directly applicable to the
% robot's actuators

% load the reference trajectory data 
xr          = [ref_traj.xr];
yr          = [ref_traj.yr];
thetar      = [ref_traj.thetar]; 
sim_time    = length(ref_traj.xr);

% error vectors intialization 
de      = zeros(1, sim_time);
thetae  = zeros(1, sim_time);

% fis params
command_size = 2; % sends 2 velocity values for the 2 wheels
num_rules = 4;
w = zeros(sim_time, num_rules);
w_fin = zeros(sim_time, 1);

% control signal sent to the robot actuators
%   vl,vr - velocities to left/right motors
%   u = [vl;vr] or
%   u(t,i) = [k_d(i)*de + k_t(i)*thetae; k_d(i)*de - k_t(i)*thetae];

% Gains of low level P-Controllers
k_d = [0.25, 0.25, 0.50, 0.50];
k_t = [0.12, 0.25, 0.12, 0.25];

% control signal
u   = zeros(command_size, sim_time, num_rules);
u_fin = zeros(command_size, sim_time, 1);

% robot pose during simulation
x       = zeros(sim_time, 1);
y       = zeros(sim_time, 1);
theta   = zeros(sim_time, 1);

% start position for the robot
x(1)     = xr(1);
y(1)     = yr(1);
theta(1) = thetar(1);

% robot params 
R = 0.04;   % wheel radius [m]
D = 0.16;   % robot size   [m]
vr = zeros(sim_time, 1);    % robot translational velocity [m/s]
wr = zeros(sim_time, 1);    % robot angular velocity       [rad/s]

% run simulation loop 
for t = 1:sim_time
    
    % compute the errors in distance and angle 
    % the errors are the inputs to the FLC 
    
    % compute distance error in the 2D field
    de(t) = sqrt((xr(t) - x(t))^2 + (yr(t) - y(t))^2);
    % compute the angle error
    thetae(t) = thetar(t) - theta(t);
   
    % fire rules in the inference engine    
    % if de is small and abs(thetae) is small then 
    % u(t,i) = [k_d(i)*de(t) + k_t(i)*thetae(t); k_d(i)*de(t) - k_t(i)*thetae(t)];
    w(t, 1) = min([compute_membership(de(t),'f11'), ...
                 compute_membership(abs(thetae(t)),'f21')]);
    u(:, t, 1) = [k_d(1)*de(t) + k_t(1)*thetae(t); k_d(1)*de(t) - k_t(1)*thetae(t)];
    
    % if de is small and abs(thetae) is large then 
    w(t, 2) = min([compute_membership(de(t),'f11'), ...
                 compute_membership(abs(thetae(t)),'f22')]);
    u(:, t, 2) = [k_d(2)*de(t) + k_t(2)*thetae(t); k_d(2)*de(t) - k_t(2)*thetae(t)];
    
    % if de is large and abs(thetae) is small then 
    w(t, 3) = min([compute_membership(de(t),'f12'), ...
                 compute_membership(abs(thetae(t)),'f21')]);
    u(:,t, 3) = [k_d(3)*de(t) + k_t(3)*thetae(t); k_d(3)*de(t) - k_t(3)*thetae(t)];
    
    % if de is large and abs(thetae) is large then 
    w(t, 4) = min([compute_membership(de(t),'f12'), ...
                 compute_membership(abs(thetae(t)),'f22')]);
    u(:, t, 4) = [k_d(4)*de(t) + k_t(4)*thetae(t); k_d(4)*de(t) - k_t(4)*thetae(t)];
    
    % compute output of the fuzzy controller that will be sent to the robot
    for idx = 1:num_rules
        u_fin(:, t) = u_fin(:, t) + w(t, idx)*u(:, t, idx);
        w_fin(t) = w_fin(t) + w(t, idx);
    end
    
    % weighted average
    u_fin(:, t) = u_fin(:, t)/w_fin(t);
    u_fin(:, t) = u_fin(:,t);
    % weighted sum - more effective 
    % u_fin(command_size, t) = u_fin(command_size, t);
    
    % send the command to the robot 
    vr(t) = (u_fin(1,t) + u_fin(2, t))/2;
    wr(t) = (u_fin(1,t) - u_fin(2, t))/D;
    x(t+1) = x(t) + vr(t)*cos(theta(t));
    y(t+1) = y(t) + vr(t)*sin(theta(t));
    theta(t+1) = theta(t) + wr(t);
end

%% Visualization 

% get field handler
field = generate_field(field_size);
hold all;

% inactive robots on the field
red_team_inactive1_x = 75;
red_team_inactive1_y = 225;
black_team_inactive1_x = 225;
black_team_inactive1_y = 225;
black_team_inactive2_x = 225;
black_team_inactive2_y = 75;
robot_size = D*100; % in cm 
% draw inactive robots
rectangle('Position', [red_team_inactive1_x,   red_team_inactive1_y, robot_size, robot_size],...
          'FaceColor','r',...
          'LineWidth',3); hold on;
rectangle('Position', [black_team_inactive1_x, black_team_inactive1_y, robot_size, robot_size],...
          'FaceColor','k',...
          'LineWidth',3); hold on;
rectangle('Position', [black_team_inactive2_x, black_team_inactive2_y, robot_size, robot_size],...
          'FaceColor','k',...
          'LineWidth',3); hold on;

% draw active robot in the intial position 
rectangle('Position', [xr(1)-robot_size, yr(1)-robot_size, robot_size, robot_size],...
          'FaceColor','r',...
          'LineStyle','--',...
          'LineWidth',3); hold on;
      
% draw active robot in final position
rectangle('Position', [x(end), y(end), robot_size, robot_size],...
          'FaceColor','r',...
          'LineWidth',3); hold on;

% plot reference trajectory 
% TODO add errors manifold overlayed on reference trajectory
plot(xr, yr, 'r');
hold on;
 
% plot active robot movement to final position
for t = 1:sim_time
    plot(x(t), y(t), '*b');
    hold on;
end
grid on;
legend('WMR Reference trajectory (red)', 'WMR Real trajectory (blue)');

% figure for quantitative analysis
figure;
for t = 1:sim_time
    plot(xr(t), yr(t), '.r'); hold on;
    plot(x(t), y(t), '*b'); hold on;
end
grid on;
set(gcf,'Color','w');
set(gca, 'Box', 'off'); 
text_offset = 5;
text(xr(1)-text_offset, yr(1)-text_offset, 'Start');
text(x(end)-text_offset, y(end)-text_offset, 'Stop');
legend('Reference trajectory', 'Robot trajectory');

%%
close all
clear all
addpath([pwd,'/DoNotChange']);
  
% controller
controlhandle = @controller;

% real-time 
real_time = true;

% max time
time_tol = 20;

% parameters for simulation
params = manipulator();
%THESE WILL BE YOUR TEST TORQUE LIMITS FOR GRADING. 
params.torque_limit = [1.5;10;.1;1.5];
%Feel free to change to see what different gains do, but remember to change
%them back for your final submission

% trajectory generator
% trajhandle = @(t,l1,l2,l0)trajectory_gripper(t,goalStates,l1,l2,l0);

% RIGHT CLICK FOR MORE INFORMATION ABOUT HELIX_TRAJ
% function [ desired_state ] = helix_traj( t , x_c,y_c, r, omega, slope, tmax , l1, l2, l0 )
trajhandle = @(t,l1,l2,l0)helix_traj( t , l1+l2 , 0 , (l1+l2/4)*.7 , .1 , -.05 , 16 , params.l1, params.l2, params.l0 );

% Added another traj function that goes to a single point, interesting to
% look at what your controller is doing given a different goal. You will
% not be grade on how well you do with this trajectory

%  trajhandle = @(t,l1,l2,l0)go_to_traj( t,[xTarget;yTarget;zTarget;yaw] ,l1,l2,l0)
%  trajhandle = @(t,l1,l2,l0)go_to_traj( t,[0;1;-.5;pi/2] ,l1,l2,l0);


%% Initialize figure
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
linkcolors = lines(3); 

set(gcf, 'Renderer', 'OpenGL');

%% Initial Conditions
fprintf('Setting initial conditions...\n')
max_iter  = 5000;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors

% Set start state and stop state
F0 = trajhandle(0,params.l1,params.l2,params.l0);
Ff = trajhandle(time_tol,params.l1,params.l2,params.l0);
start_gripper = F0.pos';
stop_gripper  = Ff.pos';
x0 = zeros(8,1);
x0(1:4,1)     = start_gripper';
x             = x0;        % state

% Joint torques
torques_hist = zeros(4, 0); 
%% Run Simulation
fprintf('Simulation Running...')

for iter = 1:max_iter
    
    timeint = time:tstep:time+cstep;
    
    tic;
    if iter == 1
        MP = manipulatorPlot(x0, params.joint1, params.joint2, params.joint3, params.l1, params.l2,...
            params.l0, params.gripper, params.finger, linkcolors, max_iter, h_3d);
        MP.updateManipulatorPlot(x, x0,[0,0,0,0]', time);
        h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
    end
    [tsave, xsave] = ode45(@(t,s) manipulatorODE(t, s, controlhandle, trajhandle, params, cstep, 2), timeint, x);
    x = xsave(end, :)';
    % update manipulator plot
    [sdot, desired_state, torque] = manipulatorODE();
    torques_hist = [torques_hist, torque];   
    MP.updateManipulatorPlot(x, [desired_state.pos; desired_state.vel], [0,0,0,0]', time+cstep);
    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep)); 
    time = time+cstep;
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*50)
        err = 'Ode45 Unstable';
        break;
    end
    
    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end
    
    if time > time_tol
        break;
    end
end


%% Post Processing

MP.truncateHist();
h_pos = figure('Name', ['Manipulator ' ' : position']);
plot_state(h_pos, MP.state_hist(1:4,:), MP.time_hist, 'pos', 'vic');
plot_state(h_pos, MP.state_des_hist(1:4,:), MP.time_hist, 'pos', 'des');

h_vel = figure('Name', ['Manipulator ' ' : velocity']);
plot_state(h_vel, MP.state_hist(5:8,:), MP.time_hist, 'vel', 'vic');
plot_state(h_vel, MP.state_des_hist(5:8,:), MP.time_hist, 'vel', 'des');

h_gripper = figure('Name', ['Gripper ' ' : state']);
plot_state(h_gripper, MP.gripper_state_hist(:,:), MP.time_hist, 'gripper', 'vic');
plot_state(h_gripper, MP.des_gripper_state_hist(:,:), MP.time_hist, 'gripper', 'des');

h_torque = figure('Name', ['Torques ' ' : torque']);
plot_state(h_torque, torques_hist(:,:), MP.time_hist(2:end), 'torques', 'vic');

if(~isempty(err))
    error(err);
end

fprintf('finished.\n')


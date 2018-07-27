close all
clear variables
clc

%% Initialization

% The following parameters are instantiated using the quad_params file
% Change the system parameters by editing the quad_params file

parameters = quad_params();  

% Initial state and simulation time

initial_attitude = expm(225*pi/180*hat([0 0 1].'));
initial_angular_velocity = [0 0 0].';
tspan =(0:.005:10);
initial_estimate = [0.5 0.5 0.5].';

%% Simulation

x0 = [initial_attitude,initial_angular_velocity,initial_estimate];

% Passing in dynamics function,disturbance function(noise) and trajectory
% handle to the dynamics ode
[tvalues,xvalues] = ode45(@(t,x) dynamics_ode(t,x,@controller,@trajectory,@noise,parameters), tspan, x0);
xvalues = xvalues.';


%% Post processing
[R_desvalues,omega_desvalues] = arrayfun(@trajectory,tvalues,'UniformOutput',0);
[~,delta_values] = arrayfun(@noise,tvalues,'UniformOutput',0);
[W,~] = noise(1);

% Preallocating matrices
B = zeros(length(parameters.theta),1);
e_RB = zeros(3,length(parameters.theta));
angles = zeros(length(tvalues),length(parameters.theta));
psi = zeros(length(tvalues),1);
V = zeros(length(tvalues),1);
delta_cap_values = zeros(3,length(tvalues));
delta_values_plot = zeros(3,length(tvalues));
R_desvalues_plot = zeros(9,length(tvalues));
omega_desvalues_plot = zeros(3,length(tvalues));

for i = 1:length(tvalues)
    R = reshape(xvalues(1:9,i),3,3);
    R_desvalues_plot(1:9,i) = R_desvalues{i}(:);
    omega_desvalues_plot(:,i) = omega_desvalues{i};
    A = (1/2)*trace(parameters.G*(eye(3) - R_desvalues{i}.'*R));
    e_RA = (1/2)*inv_hat(parameters.G*R_desvalues{i}.'*R - R.'*R_desvalues{i}*parameters.G);
    
    for j = 1:length(parameters.theta)    
        
        
        B(j) = 1 - (1/parameters.alpha)*log((cos(parameters.theta(j)) - (parameters.r).'*R.'*(parameters.v(:,j)))/(1 + cos(parameters.theta(j))));     
        angles(i,j) = acos((parameters.r).'*R.'*parameters.v(:,j));
        e_RB(:,j) = hat(R.'*(parameters.v(:,j)))*parameters.r/(parameters.alpha*(parameters.r.'*R.'*(parameters.v(:,j)) - cos(parameters.theta(j))));
    end
    
    psi(i) = A*(1 + sum(B) - length(parameters.theta));
    
    e_omega = xvalues(10:12,i);
    e_R = e_RA * sum(B) + e_RB * (A * ones(length(parameters.theta),1));                                  % Rotation error vector                                                                    % Angular Velocity error vector

    %True Parameter Values
    delta_values_plot(:,i) = delta_values{i};
    
    % The lyapunvov function
    V(i) = (1/2)*e_omega.'*parameters.inertia*e_omega + parameters.kR*psi(i) +...
        parameters.c*(parameters.inertia*e_omega).'*e_R + (1/(2*parameters.kdelta))*(delta_values{i}-xvalues(13:15,i)).'*(delta_values{i}-xvalues(13:15,i));

end
     
 
%% Animation
plot_motion(xvalues.',tvalues,@trajectory,parameters)

%ode45(@(t,y) dynamics_ode(t,y,@controller,@trajectory,@noise,parameters), interval, y0);
%% Plotting

fig1 = figure('units','normalized','outerposition',[0 0 1 1]);

subplot(2,2,1)
plot(tvalues,(180/pi)*angles);
title('Angle to constraint for Debris Vectors vs time')

subplot(2,2,2)
plot(tvalues,psi);
legend('Psi')
title('Error Function(psi) vs time')

subplot(2,2,3)
plot(tvalues,V);
legend('V')
title('Lyapunov Function vs time')

subplot(2,2,4)
plot(tvalues,[diff(V);(V(end)-V(end-1))]);
legend('Vdot')
title('Lyapunov Function Derivative vs time')



fig1 = figure('units','normalized','outerposition',[0 0 1 1]);
subplot(3,1,1)
plot(tvalues,delta_values_plot(1,:),tvalues,xvalues(13,:))
legend('True Value','Estimated Value')
title('True Disturbance vs Estimate (1st component) with time')


subplot(3,1,2)
plot(tvalues,delta_values_plot(2,:),tvalues,xvalues(14,:))
legend('True Value','Estimated Value')
title('True Disturbance vs Estimate (2nd component) with time')

subplot(3,1,3)
plot(tvalues,delta_values_plot(3,:),tvalues,xvalues(15,:))
legend('True Value','Estimated Value')
title('True Disturbance vs Estimate (3rd component) with time')

fig2 = figure('units','normalized','outerposition',[0 0 1 1]);

subplot(3,4,1)
plot(tvalues,xvalues(1,:),tvalues,R_desvalues_plot(1,:))
legend('R11','R11 des')
title('R11 vs time')

subplot(3,4,2)
plot(tvalues,xvalues(2,:),tvalues,R_desvalues_plot(2,:))
legend('R12','R12 des')
title('R12 vs time')

subplot(3,4,3)
plot(tvalues,xvalues(3,:),tvalues,R_desvalues_plot(3,:))
legend('R13','R13 des')
title('R13 vs time')

subplot(3,4,4)
plot(tvalues,xvalues(10,:),tvalues,omega_desvalues_plot(1,:))
legend('w1','w1 des')
title('w1 vs time')

subplot(3,4,5)
plot(tvalues,xvalues(4,:),tvalues,R_desvalues_plot(4,:))
legend('R21','R21 des')
title('R21 vs time')

subplot(3,4,6)
plot(tvalues,xvalues(5,:),tvalues,R_desvalues_plot(5,:))
legend('R22','R22 des')
title('R22 vs time')

subplot(3,4,7)
plot(tvalues,xvalues(6,:),tvalues,R_desvalues_plot(6,:))
legend('R23','R23 des')
title('R23 vs time')

subplot(3,4,8)
plot(tvalues,xvalues(11,:),tvalues,omega_desvalues_plot(2,:))
legend('w2')
title('w2 vs time')

subplot(3,4,9)
plot(tvalues,xvalues(7,:),tvalues,R_desvalues_plot(7,:))
legend('R31','R31 des')
title('R31 vs time')

subplot(3,4,10)
plot(tvalues,xvalues(8,:),tvalues,R_desvalues_plot(8,:))
legend('R32','R32 des')
title('R32 vs time')

subplot(3,4,11)
plot(tvalues,xvalues(9,:),tvalues,R_desvalues_plot(9,:))
legend('R33','R33 des')
title('R33 vs time')

subplot(3,4,12)
plot(tvalues,xvalues(12,:),tvalues,omega_desvalues_plot(3,:))
legend('w3')
title('w3 vs time')




%% Defining system variables

m = 5; % Load [kg]
M = 250; % Trolley Mass [kg]
l = 3; % Rope Length [m]
g = 9.81; % Acceleration due to gravity [m/s^2]
V_max = 20/60; % Max Trolley Speed [m/s]
P_max = 1500; % Max Actuator Power [W]
F_max = P_max / V_max % Max Force on Trolley [N]
stop_time = 20; % Simulation Time [s]

% Defining Linear System State Space
A = [0 1       0        0;
     0 0    -m*g/M      0;
     0 0       0        1;
     0 0 -(m+M)*g/(M*l) 0;];
B = [0; 1/M; 0; 1/(M*l)];
%% Simulation with no sway control 
% Setting Gains

% PID Gains for Position Control
K_p = 150;
K_d = 500;
K_i = 0.02;
K = [0 0 0 0]; % No LQR
% Simulating

sim_no_control = sim("Crane_Model_Simscape.slx"); % Running simulation without control
%sim_no_control = sim("Crane_Model_Simulink.slx"); % Running simulation without control
% Results

figure 
hold on
plot(sim_no_control.trolley_ref_Pos)
plot(sim_no_control.trolley_Pos)
title("Reference & Actual Trolley Position (No Sway Control)")
xlabel("Time (s)")
ylabel("Position (m)")
legend("Reference", "Actual", "Location", "southeast");
ylim([0 5])
hold off
grid on

figure 
hold on
plot(sim_no_control.trolley_Force)
title("Actuation Force (No Sway Control)")
xlabel("Time (s)")
ylabel("Force (N)")
ylim([-40 40])
hold off
grid on

figure 
hold on
plot(sim_no_control.rope_Angle)
title("Rope Sway Angle (No Sway Control)")
xlabel("Time (s)")
ylabel("Angle (degrees)")
hold off
grid on
%% Simulation with sway control through LQR
% Setting Gains

% Defining weighting matrices
Q = [1 0 0 0;
     0 1 0 0;
     0 0 10000000 0;
     0 0 0 10000000;]; 
R = 1;

C_mat = ctrb(A, B); % Controllability Matrix
rank_C = rank(C_mat); % Rank Determination

% Determine controllability & K gain
if rank_C == size(A, 1)
    disp('The system is controllable.');
    K = lqr(A,B,Q,R);
else
    disp('The system is not controllable.');
end
% Simulating

sim_control = sim("Crane_Model_Simscape.slx"); % Running simulation with control
%sim_control = sim("Crane_Model_Simulink.slx"); % Running simulation with control
% Results

figure 
hold on
plot(sim_control.trolley_ref_Pos)
plot(sim_control.trolley_Pos)
title("Reference & Actual Trolley Position (Sway Control)")
xlabel("Time (s)")
ylabel("Position (m)")
legend("Reference", "Actual", "Location", "southeast");
ylim([0 5])
hold off
grid on

figure 
hold on
plot(sim_control.trolley_Force)
title("Actuation Force (Sway Control)")
xlabel("Time (s)")
ylabel("Force (N)")
%ylim([0 200])
hold off
grid on

figure 
hold on
plot(sim_control.rope_Angle)
title("Rope Sway Angle (Sway Control)")
xlabel("Time (s)")
ylabel("Angle (degrees)")
hold off
grid on
%% Comparing Results

figure 
hold on
plot(sim_no_control.rope_Angle)
plot(sim_control.rope_Angle)
title("Rope Sway Angle")
xlabel("Time (s)")
ylabel("Angle (degrees)")
legend("No Control", "Control", "Location", "southeast");
hold off
grid on

figure 
hold on
plot(sim_no_control.trolley_ref_Pos)
plot(sim_no_control.trolley_Pos)
plot(sim_control.trolley_Pos)
title("Reference & Actual Trolley Position")
xlabel("Time (s)")
ylabel("Position (m)")
legend("Reference", "No Control", "Control", "Location", "southeast");
ylim([0 5])
hold off
grid on
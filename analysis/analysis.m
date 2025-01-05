% Ball on Track Analysis
% Jacob Lessing
% 2 January 2025

%% System Parameters
g = 9.8;                        % gravitational acceleration (m/s^2)

m = 0.01838;                    % mass of ball (kg)
Ib = 1.654 * 10^-6;             % moment of inertia of ball (kg*m^2)
Ip = 1.8 * 10^-4;               % moment of inertia of track (kg*m^2)
rb = 0.015;                     % radius of the ball (m)

%% Model
s = tf('s');
P = - m*g / ((m + Ib/rb^2) * s^2);            % X(s)/Theta(s), plant TF

%% Analysis
figure(1);
pzplot(P)
% P has two poles at s = 0, so is marginally stable

%% Control (PID)

% pidTuner(P);
Kp = -0.01;
Ki = 0;
Kd = -1;
C = pid(Kp, Ki, Kd);                % controller
G = feedback(C*P, 1);               % X(s)/R(s), closed loop TF

t = 0:0.01:10;                      % 1 sec
step_amplitude = 0.05;              % 1 cm
step(step_amplitude*G, t)

%% Testing

% Testing how well output follow control signal
t = 0:0.01:5;
r = 0.05*sin(10*t);                 % reference signal (5cm amplitude, 10Hz)
x = lsim(G, r, t);                  % output

plot(t, x, 'r-')
hold on;
plot(t, r, 'b-')
hold off;

% Enhancing the plot
title('System Response to Control Signal');
xlabel('Time (s)');
ylabel('Amplitude');
legend({'Output (y)', 'Reference Signal (r)'});
grid on; % Add grid lines

% The PID controlled system follows the control signal well

%% Mechanical Design Requirements

% What range of angles does the track need to be able to swing through?

H = C / (1 + C*P);                            % Theta(s) / R(s), TF

t = 0:0.01:5;
r = 0.05*sin(10*t);                 % reference signal (5cm amplitude, 10Hz)
u = lsim(H, r, t);                  % theta

plot(t, u, 'r-')
hold on;
plot(t, r, 'b-')
hold off;

% Enhancing the plot
title('System Response to Control Signal');
xlabel('Time (s)');
ylabel('Theta');
legend({'Theta (u)', 'Reference Signal (r)'});
grid on; % Add grid lines

%% Scratchpad

% pzmap(G)
pidTuner(P)
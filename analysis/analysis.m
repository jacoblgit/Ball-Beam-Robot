% Ball on Track Analysis
% Jacob Lessing
% 2 January 2025
%% New System Model
m = 2.8 * 10^-3;                % mass of ball (kg)
c = 2/5;                        % solid steel ball
Ip = 1.8 * 10^-4;               % moment of inertia of track (kg*m^2)
g = 9.8;                        % gravitational acceleration (m/s^2)
alpha = 1;                      % time constant (sec)

kp = (1+c)/(g*alpha^2);
ki = 0;
kd = 2*(1+c)/(g*alpha);
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

C = pid(Kp, Ki, Kd);                % controller
G = feedback(C*P, 1);               % X(s)/R(s), closed loop TF

t = 0:0.01:10;                      % 1 sec
step_amplitude = 0.10;              % 1 cm
step(step_amplitude*G, t)

clearvars
clc
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TEST 99 data 
% time, elapsed, command, rpm, adc, voltage, volt (DF)
%   1      2        3      4    5      6         7

%% get left motor data
L_99 = xlsread('2022-04-19_L_motor_test_99.csv'); 
L_cmd     = L_99(:,3);
L_t_end   = L_99(end,2);
L_elap    = L_99(:,2);
L_voltage = L_99(:,6);
L_rpm     = L_99(:,4);
L_omega   = L_rpm*0.10472;

% get right motor data
R_99 = xlsread('2022-04-19_R_motor_test_99.csv');
R_cmd     = R_99(:,3);
R_t_end   = R_99(end,2);
R_elap    = R_99(:,2);
R_voltage = R_99(:,6);
R_rpm     = R_99(:,4);
R_omega   = R_rpm*0.10472;

%% plot left motor data
figure
subplot(1,2,1)
hold on
plot(L_elap,L_omega);
plot(L_elap,L_voltage);
title('Left motor');
legend('omega [rad/s]','voltage [V]','Location','NorthWest');

% plot right motor
subplot(1,2,2)
hold on
plot(R_elap,R_omega);
plot(R_elap,R_voltage);
title('Right motor');
legend('omega [rad/s]','voltage [V]','Location','NorthWest');

%% plot cmd, voltages and omega
figure
subplot(1,3,1)
hold on
plot(L_elap,L_cmd);
plot(R_elap,R_cmd);
title('motor command');
xlabel('time elapsed [s]')
ylabel('motor command')
legend('left','right','Location','NorthWest');
subplot(1,3,2)
hold on
plot(L_elap,L_voltage);
plot(R_elap,R_voltage);
title('voltage');
xlabel('time elapsed [s]')
ylabel('voltage [V]')
legend('left','right','Location','NorthWest');
subplot(1,3,3)
hold on
plot(L_elap,L_omega);
plot(R_elap,R_omega);
title('omega');
xlabel('time elapsed [s]')
ylabel('omega [rad/s]')
legend('left','right','Location','NorthWest');

%% system id, voltage/omega
% results from system id:
% tf from voltage to rad/s
tf_L = tf(1.795e07, [1, 2.943e07])
tf_R = tf(21.25, [1, 33.45])

% step responses (step=1)
figure
subplot(1,2,1)
hold on
step(tf_L)
title('step response, left')
subplot(1,2,2)
step(tf_R)
title('step response, right')

%% volt/omega model comparison - left
figure
subplot(1,2,1)
plot(L_elap,L_omega)
ylim([0 3])
ylabel('omega [rad/s]')
xlabel('time [s]')
title('experimental step response - left motor')
subplot(1,2,2)
ylim([0 3])
hold on
opt = stepDataOptions('StepAmplitude',1);
[L_s1, L_t] = step(tf_L,opt);
plot(L_t, L_s1)
opt = stepDataOptions('StepAmplitude',2);
[L_s2, L_t] = step(tf_L,opt);
plot(L_t, L_s2)
opt = stepDataOptions('StepAmplitude',4);
[L_s4, L_t] = step(tf_L,opt);
plot(L_t, L_s4)
ylabel('omega [rad/s]')
xlabel('time [s]')
title('model step response - left motor')


% model comparison - right
figure
subplot(1,2,1)
plot(R_elap,R_omega)
ylim([0 3])
ylabel('omega [rad/s]')
xlabel('time [s]')
title('experimental step response - right motor')
subplot(1,2,2)
ylim([0 3])
hold on
opt = stepDataOptions('StepAmplitude',1);
[R_s1, R_t] = step(tf_R,opt);
plot(R_t, R_s1)
opt = stepDataOptions('StepAmplitude',2);
[R_s2, R_t] = step(tf_R,opt);
plot(R_t, R_s2)
opt = stepDataOptions('StepAmplitude',4);
[R_s4, R_t] = step(tf_R,opt);
plot(R_t, R_s4)
ylabel('omega [rad/s]')
xlabel('time [s]')
title('model step response - right motor')

%% system id - cmd/omega (1p/0z)
tf_L_cmd = tf(0.166, [1 13.35]);
tf_R_cmd = tf(0.1777, [1 14.39]);

figure
title('hi')
subplot(1,2,1)
opt = stepDataOptions('StepAmplitude',50);
step(tf_L_cmd,opt)
title('step L, amp=50 (cmd/omega)')
subplot(1,2,2)
opt = stepDataOptions('StepAmplitude',50);
step(tf_R_cmd,opt)
title('step R, amp=50 (cmd/omega)')

%% model comparison (cmd/omega) (1p/0z)
figure
% LEFT MOTOR 
subplot(1,2,1)
hold on
plot(L_elap,L_omega)            % plot experimental
                                % generate and plot model
opt = stepDataOptions('StepAmplitude',50);
[L_s1, L_t] = step(tf_L_cmd,25,opt);
plot(L_t+5, L_s1)
opt = stepDataOptions('StepAmplitude',100);
[L_s2, L_t] = step(tf_L_cmd,25,opt);
plot(L_t+10, L_s2)
opt = stepDataOptions('StepAmplitude',200);
[L_s4, L_t] = step(tf_L_cmd,25,opt);
plot(L_t+15, L_s4)
                                % labels
ylabel('omega [rad/s]')
xlabel('time [s]')
ylim([0 3])
xlim([0 25])
title('cmd/omega model comparison, left motor')
legend('experimental','model, amp=50','model, amp=100','model, amp=200','Location','NorthWest')

% RIGHT MOTOR
subplot(1,2,2)
hold on
plot(R_elap,R_omega)            % plot experimental
                                % generate and plot model
opt = stepDataOptions('StepAmplitude',50);
[R_s1, R_t] = step(tf_R_cmd,25,opt);
plot(R_t+5, R_s1)
opt = stepDataOptions('StepAmplitude',100);
[R_s2, R_t] = step(tf_R_cmd,25,opt);
plot(R_t+10, R_s2)
opt = stepDataOptions('StepAmplitude',200);
[R_s4, R_t] = step(tf_R_cmd,25,opt);
plot(R_t+15, R_s4)
                                % labels
ylabel('omega [rad/s]')
xlabel('time [s]')
ylim([0 3])
xlim([0 25])
title('cmd/omega model comparison, right motor')
legend('experimental','model, amp=50','model, amp=100','model, amp=200','Location','NorthWest')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% TEST 03 %%
%% get motor data
clc
close all
clearvars
% Time elapsed [s], Command Value, omega [rad/s], ADC, Motor Voltage (LPF) [V], Motor Voltage (Filtered LPF) [V]
%       1                 2             3          4              5                      6
L_03 = xlsread('2022-05-04_L_motor_test_03.csv'); 
L_cmd     = L_03(:,2);
L_t_start = L_03(1,1);
L_t_end   = L_03(end,1);
L_elap    = L_03(:,1);
L_ts      = L_elap(2) - L_elap(1);
L_voltage = L_03(:,5);
L_omega   = L_03(:,3);

% get right motor data
R_03 = xlsread('2022-05-04_R_motor_test_03.csv');
R_cmd     = R_03(:,2);
R_t_start = R_03(1,1);
R_t_end   = R_03(end,1);
R_elap    = R_03(:,1);
R_ts      = R_elap(2) - R_elap(1);
R_voltage = R_03(:,5);
R_omega   = R_03(:,3);

%% plot motor data (omega and voltage)
figure
subplot(1,2,1)
hold on
plot(L_elap,L_omega);
plot(L_elap,L_voltage);
title('Left motor');
legend('omega [rad/s]','voltage [V]','Location','NorthWest');

% plot right motor
subplot(1,2,2)
hold on
plot(R_elap,R_omega);
plot(R_elap,R_voltage);
title('Right motor');
legend('omega [rad/s]','voltage [V]','Location','NorthWest');

%% plot cmd, voltages and omega
figure
subplot(1,3,1)
hold on
plot(L_elap,L_cmd);
plot(R_elap,R_cmd);
title('motor command');
xlabel('time elapsed [s]')
ylabel('motor command')
legend('left','right','Location','NorthWest');
subplot(1,3,2)
hold on
plot(L_elap,L_voltage);
plot(R_elap,R_voltage);
title('voltage');
xlabel('time elapsed [s]')
ylabel('voltage [V]')
legend('left','right','Location','NorthWest');
subplot(1,3,3)
hold on
plot(L_elap,L_omega);
plot(R_elap,R_omega);
title('omega');
xlabel('time elapsed [s]')
ylabel('omega [rad/s]')
legend('left','right','Location','NorthWest');

%% system id - cmd/omega (1p/0z)
tf_L_cmd = tf(2.411, [1 19.81]);
tf_R_cmd = tf(2.431, [1 20.1]);

figure
subplot(1,2,1)
opt = stepDataOptions('StepAmplitude',50);
step(tf_L_cmd,opt)
title('step L, amp=50 (cmd/omega)')
subplot(1,2,2)
opt = stepDataOptions('StepAmplitude',50);
step(tf_R_cmd,opt)
title('step R, amp=50 (cmd/omega)')

%% model comparison (cmd/omega) (1p/0z)
figure
% LEFT MOTOR 
subplot(1,2,1)
hold on
plot(L_elap,L_omega)            % plot experimental
                                % generate and plot model
opt = stepDataOptions('StepAmplitude',50);
[L_s1, L_t] = step(tf_L_cmd,25,opt);
plot(L_t+3, L_s1)
opt = stepDataOptions('StepAmplitude',100);
[L_s2, L_t] = step(tf_L_cmd,25,opt);
plot(L_t+6, L_s2)
opt = stepDataOptions('StepAmplitude',200);
[L_s4, L_t] = step(tf_L_cmd,25,opt);
plot(L_t+9, L_s4)
                                % labels
ylabel('omega [rad/s]')
xlabel('time [s]')
ylim([0 30])
xlim([0 12])
title('cmd/omega model comparison, left motor')
legend('experimental','model, amp=50','model, amp=100','model, amp=200','Location','NorthWest')

% RIGHT MOTOR
subplot(1,2,2)
hold on
plot(R_elap,R_omega)            % plot experimental
                                % generate and plot model
opt = stepDataOptions('StepAmplitude',50);
[R_s1, R_t] = step(tf_R_cmd,25,opt);
plot(R_t+3, R_s1)
opt = stepDataOptions('StepAmplitude',100);
[R_s2, R_t] = step(tf_R_cmd,25,opt);
plot(R_t+6, R_s2)
opt = stepDataOptions('StepAmplitude',200);
[R_s4, R_t] = step(tf_R_cmd,25,opt);
plot(R_t+9, R_s4)
                                % labels
ylabel('omega [rad/s]')
xlabel('time [s]')
ylim([0 30])
xlim([0 12])
title('cmd/omega model comparison, right motor')
legend('experimental','model, amp=50','model, amp=100','model, amp=200','Location','NorthWest')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% 
systemIdentification




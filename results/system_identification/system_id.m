%% housekeeping

clearvars
clc
close all

%% read motor data

data = xlsread('24-09-20_sys_id_data.csv');
% headers: elapsed time [s], cmd [A255], omega L [rad/s], omega R [rad/s]
%                  1             2             3                4
t_vec   = data(:,1);
cmd     = data(:,2);
L_omega = data(:,3);
R_omega = data(:,4);
t_start = t_vec(1);
ts      = t_vec(2) - t_vec(1);

%% plot motor data (omega)
figure
subplot(1,2,1)
hold on
plot(t_vec,L_omega);
title('Motor speed');
legend('\omega [rad/s]','Location','NorthWest');

% plot right motor
subplot(1,2,2)
hold on
plot(t_vec,R_omega);
title('Right motor');
legend('\omega [rad/s]','Location','NorthWest');

% plot together
figure
hold on
plot(t_vec,L_omega);
plot(t_vec,R_omega);
title('Motor speed');
legend('\omega_L [rad/s]','\omega_R [rad/s]','Location','NorthWest');

%% system id results (from system identification GUI)
tf_L = tf(2.896, [1, 24.35]);
tf_R = tf(3.143, [1, 24.94]);

%% plot step response
figure
subplot(1,2,1)
opt = stepDataOptions('StepAmplitude',50);
step(tf_L,opt)
title('step response L, amp=50')
ylim([0,7])
xlim([0, 0.4])
ylabel('\omega [rad/s]')

subplot(1,2,2)
opt = stepDataOptions('StepAmplitude',50);
step(tf_R,opt)
title('step response R, amp=50')
ylim([0,7])
xlim([0, 0.4])
ylabel('\omega [rad/s]')

figure
hold on
opt = stepDataOptions('StepAmplitude',50);
step(tf_L,opt)
opt = stepDataOptions('StepAmplitude',50);
step(tf_R,opt)
legend('left','right','Location','east')
ylabel('\omega [rad/s]')
title('step response of motors, command value 50')

%% model comparison

figure

command_duration = 4;
time_step_1 = 4;
time_step_2 = 8;
time_step_3 = 12;

% LEFT MOTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(1,2,1)
hold on
plot(t_vec,L_omega) % plot experimental          

% plot first modeled step response, 50
opt = stepDataOptions('StepAmplitude',50);
[L_s1, L_t] = step(tf_L,25,opt);
plot(L_t + 1*command_duration, L_s1)

% plot second modeled step response, 100
opt = stepDataOptions('StepAmplitude',100);
[L_s2, L_t] = step(tf_L,25,opt);
plot(L_t + 2*command_duration, L_s2)

% plot third modeled step response, 200
opt = stepDataOptions('StepAmplitude',200);
[L_s4, L_t] = step(tf_L,25,opt);
plot(L_t + 3*command_duration, L_s4)
                               
ylabel('\omega [rad/s]')
xlabel('time [s]')
ylim([0 30])
xlim([0 4*command_duration])
title('step response experimental vs model, left motor')
legend('experimental','model, amp=50','model, amp=100',...
    'model, amp=200','Location','NorthWest')

% RIGHT MOTOR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(1,2,2)
hold on
plot(t_vec,R_omega)            
                                
opt = stepDataOptions('StepAmplitude',50);
[R_s1, R_t] = step(tf_R,25,opt);
plot(R_t + 1*command_duration, R_s1)

opt = stepDataOptions('StepAmplitude',100);
[R_s2, R_t] = step(tf_R,25,opt);
plot(R_t + 2*command_duration, R_s2)

opt = stepDataOptions('StepAmplitude',200);
[R_s4, R_t] = step(tf_R,25,opt);
plot(R_t + 3*command_duration, R_s4)
                                % labels
ylabel('\omega [rad/s]')
xlabel('time [s]')
ylim([0 30])
xlim([0 4*command_duration])
title('step response experimental vs model, right motor')
legend('experimental','model, amp=50','model, amp=100',...
    'model, amp=200','Location','NorthWest')

x0= 100;
y0= 250;
width = 750;
height = 315;
set(gcf, 'units', 'points','position',[x0,y0,width,height])


















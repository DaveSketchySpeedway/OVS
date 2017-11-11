clear
close all
clc

TIME_STEP = 0.05;
VEHICLE_FILE = '..\config\vehicle_pg_61187.yml';
% MODEL_FILE = '..\config\vehicle_sluggish_car.yml';

%% instantiate
model = Model(TIME_STEP);
model.load_vehicle(VEHICLE_FILE);
model.check_gain();
% car.print_continuous_transfer_functions();
% car.print_discrete_transfer_functions();
model.plot_step();
% car.plot_bode();

%% speed sensitivity
u = [1, 10, 20, 30, 40, 50, 60];
loop_start_time = tic;
for j = 1:length(u)
    section_start_time = tic;
    model.init();
    for i = 1:100
        t(i,1) = i*TIME_STEP;
        model.set_input(0,1);
        model.set_linvel([u(j),0,0]);
        model.calc();
        angvel(i,:) = model.get_angvel();
        linacc(i,:) = model.get_linacc();
        sideslip(i,:) = model.get_sideslip();
    end
    
    wz(:,j) = angvel(:,3);
    ay(:,j) = linacc(:,2);
    beta_refsteer(:,j) = sideslip(:,1);
    beta_latacc(:,j) = sideslip(:,2);
    beta_linvel(:,j) = sideslip(:,3);
    clear angvel linacc sideslip
    toc(section_start_time);
end
toc(loop_start_time);
    
%%
figure
subplot(2,2,1)
plot(t, wz.*0.1)
title('Yaw Rate')
xlabel('Time[s]')
ylabel('Yaw Rate[rad/s]') 
legend('1','10','20','30','40','50','60')
hold on
grid on

subplot(2,2,2)
plot(t, ay.*0.1)
title('Lateral Acceleration')
ylabel('Acceleration[m/s2]')
xlabel('Time[s]')
legend('1','10','20','30','40','50','60')
hold on
grid on

subplot(2,2,3)
plot(t, beta_refsteer.*0.1)
title('Sideslip (from vector)')
ylabel('Slip[rad]')
xlabel('Time[s]')
legend('1','10','20','30','40','50','60')
hold on
grid on

subplot(2,2,4)
plot(t, beta_latacc.*0.1)
title('Sideslip (from eqn)')
ylabel('Slip[rad]')
xlabel('Time[s]')
legend('1','10','20','30','40','50','60')
hold on
grid on


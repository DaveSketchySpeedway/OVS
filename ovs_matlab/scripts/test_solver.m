clear
close all
clc

TIME_STEP = 0.05;
VEHICLE_FILE = '..\config\vehicle_sluggish_car.yml';

model = Model(TIME_STEP);
model.load_vehicle(VEHICLE_FILE);
solver = Solver(TIME_STEP);

%% io test

% c = cos(1);
% s = sin(1);
% r1 = [c, -s, 0; s, c, 0; 0, 0, 1];
% r2 = [1, 0, 0; 0, c, -s; 0, s, c];
% r3 = r1*r2;
% q3 = rotmat2quaternion(r3);
% engine.init_position([0,0,0]);
% engine.init_linvel([100/3.6,0,0]);
% engine.init_orientation(q3);
% 
% position = engine.get_position()
% linvel = engine.get_linvel()
% linacc = engine.get_linacc()
% orientation = engine.get_orientation()
% angvel = engine.get_angvel()
% 
% body_linvel = engine.transform2body(linvel)
% local_linvel = engine.transform2local(body_linvel)

%% init
model.init()
solver.init_position([0,0,0]);
solver.init_linvel([50/3.6,0,0]);
solver.init_orientation([1,0,0,0]);
h0 = [5, 0, 0];
figure
plot3([0 h0(1)], [0 h0(2)], [0 h0(3)], 'r');
axis([0,100, -50,50 -20,20]);
xlabel('x');
ylabel('y');
zlabel('z');
grid on
hold on


%% run
i = 1;
t(1) = 0;
tic
while (t(i) < 5)
    % input
    i = i + 1;
    t(i) = i*TIME_STEP;
    longacc = 0;
    refsteer = 0.2;
%     refsteer = 0.2 * sin(2*pi()*0.8*t(i));
    model.set_input(longacc, refsteer);
    
    % update model
    linvel(i,:) = solver.get_linvel();
    body_linvel(i,:) = solver.transform2body(linvel(i,:));
    model.set_linvel(body_linvel(i,:));
    model.calc();
    
    % update solver
    body_linacc(i,:) = model.get_linacc();
    body_angvel(i,:) = model.get_angvel();
    body_sideslip(i,:) = model.get_sideslip();
    solver.set_body_linacc(body_linacc(i,:));
    solver.set_body_angvel(body_angvel(i,:));
    solver.update();
    cur_pos = solver.get_position();
    cur_head = solver.rotate_vector(h0);
    
    %% show
    plot3([cur_pos(1) cur_pos(1)+cur_head(1)],...
        [cur_pos(2) cur_pos(2)+cur_head(2)],...
        [cur_pos(3), cur_pos(3)+cur_head(3)],'b');
    disp([body_linacc(i,:) body_angvel(i,:) norm(linvel(i,:))]);   
    drawnow limitrate
end
toc

%%
figure
subplot(2,2,1)
plot(t, body_angvel(:,3))
title('yaw')
grid on

subplot(2,2,2)
plot(t, body_linacc(:,2))
title('ay')
grid on

subplot(2,2,3)
plot(t, body_sideslip)
title('beta')
legend('refsteer','latacc','vector')
grid on

subplot(2,2,4)
plot(t, body_linvel(:,1:2))
title('u,v')
grid on



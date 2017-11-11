clear
close all
clc

TIME_STEP = 0.05;
VEHICLE_FILE = '..\config\vehicle_sluggish_car.yml';
DX = 20;
DY = 20;
DZ = 20;
AZ = -60;
EL = 45;

solver = Solver(TIME_STEP);
model = Model(TIME_STEP);
model.load_vehicle(VEHICLE_FILE);

%%
solver.set_model(model);
solver.init_position([0,0,0]);
solver.init_linvel([50/3.6,0,0]);
solver.init_orientation([1,0,0,0]);

figure
a = axes();
l = animatedline(a, 0, 0, 0, 'Color', 'k');
p = patch(a,...
    'Faces', model.skin_faces,...
    'Vertices', solver.render(),...
    'FaceVertexCData', model.skin_colors,...
    'FaceColor','flat');
xlabel(a,'x [m]')
ylabel(a,'y [m]')
zlabel(a,'z [m]')
axis(a, [-DX,DX, -DY,DY, -DZ,DZ]);
grid on

%%
i = 1;
t(1) = 0;
tic
while (t(i) < 5)
    i = i + 1;
    t(i) = i*TIME_STEP;
    
    longacc = 2;
    refsteer = 0.2;
%     refsteer = 0.2 * sin(2*pi()*0.8*t(i));
    model.set_input(longacc, refsteer);
    
    solver.step()
    
    set(p,'Vertices', solver.render());
    pos = solver.get_position();
    addpoints(l,pos(1), pos(2), pos(3));
    axis([pos(1)-DX, pos(1)+DX, pos(2)-DY, pos(2)+DY, pos(3)-DZ, pos(3)+DZ]);
    view(a,AZ,EL);
    drawnow limitrate
end
toc


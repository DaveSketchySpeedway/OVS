close all
clear
clc

PATH_FILE = '..\config\path_mushroom.yml';
trajgen = TrajectoryGenerator();

%% txt to yaml (just a redundant tool)
raw = load('..\config\path_mushroom.txt');
checkpoints_x = raw(:,1);
checkpoints_y = raw(:,2);
checkpoints_z = raw(:,3);
% figure 
% plot3(checkpoints_x(2:end-1), checkpoints_y(2:end-1), checkpoints_z(2:end-1), 'k+')
% hold on
% plot3(checkpoints_x(1), checkpoints_y(1), checkpoints_z(1), 'ro');
% plot3(checkpoints_x(end), checkpoints_y(end), checkpoints_z(end), 'ro');
% grid on
clear data

for i = 1:length(checkpoints_x)
    c.x = checkpoints_x(i);
    c.y = checkpoints_y(i);
    c.z = checkpoints_z(i);
    checkpoints(i) = c;
end
fout.checkpoints = checkpoints;

YAML.write(PATH_FILE, fout);

%%
trajgen.load_path(PATH_FILE);
trajgen.scale_all_checkpoints(3,3,3);

%%
trajgen.save_path(PATH_FILE);
trajgen.generate();

%%
ax = axes;
pc = plot3(ax,...
    trajgen.get_checkpoints_x(),...
    trajgen.get_checkpoints_y(),...
    trajgen.get_checkpoints_z(), 'r+');
hold on
grid(ax,'on')
xlabel('x')
ylabel('y')
zlabel('z')
pf = plot3(ax,...
    trajgen.get_fringepoints_x(),...
    trajgen.get_fringepoints_y(),...
    trajgen.get_fringepoints_z(), 'ro');

ps = plot3(ax,...
    trajgen.get_splinepoints_x(),...
    trajgen.get_splinepoints_y(),...
    trajgen.get_splinepoints_z(), 'r');




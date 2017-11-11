clear
close all
clc


CUR_POS = [100,10,20];
PATH_FILE = '..\config\path_mushroom.yml';
trajgen = TrajectoryGenerator();
trajgen.load_path(PATH_FILE);
trajgen.generate();

TIME_STEP = 0.1;
rob = Robot(TIME_STEP);
rob.set_trajectory(trajgen);

%%
ax = axes;
plot3(ax,CUR_POS(1),CUR_POS(2), CUR_POS(3),'+');
t = line(ax,...
    trajgen.get_splinepoints_x(),...
    trajgen.get_splinepoints_y(),...
    trajgen.get_splinepoints_z(),...
    'Color', 'r');
grid on
hold on



%%
rob.find_reference(CUR_POS);
poserr_vector = rob.get_poserr_vector();
ahead_vector = rob.get_ahead_vector();
e = line(ax,...
    [CUR_POS(1) CUR_POS(1)+poserr_vector(1)],...
    [CUR_POS(2) CUR_POS(2)+poserr_vector(2)],...
    [CUR_POS(3) CUR_POS(3)+poserr_vector(3)],...
    'Color', 'g');
h = line(ax,...
    [CUR_POS(1) CUR_POS(1)+ahead_vector(1)],...
    [CUR_POS(2) CUR_POS(2)+ahead_vector(2)],...
    [CUR_POS(3) CUR_POS(3)+ahead_vector(3)],...
    'Color', 'g');



close all
clear
clc

data = load('..\config\path_mushroom.txt');
checkpoints_x = data(:,1);
checkpoints_y = data(:,2);
checkpoints_z = data(:,3);
figure 
plot3(checkpoints_x(2:end-1), checkpoints_y(2:end-1), checkpoints_z(2:end-1), 'k+')
hold on
plot3(checkpoints_x(1), checkpoints_y(1), checkpoints_z(1), 'ro');
plot3(checkpoints_x(end), checkpoints_y(end), checkpoints_z(end), 'ro');
grid on
clear data

%%
resolution = 100;
tension = 0;
    
spline_x = [];
spline_y = [];
spline_z = [];
for i = 2:length(checkpoints_x)-2
    x0 = checkpoints_x(i-1);
    y0 = checkpoints_y(i-1);
    z0 = checkpoints_z(i-1);
    x1 = checkpoints_x(i);
    y1 = checkpoints_y(i);
    z1 = checkpoints_z(i);
    x2 = checkpoints_x(i+1);
    y2 = checkpoints_y(i+1);
    z2 = checkpoints_z(i+1);
    x3 = checkpoints_x(i+2);
    y3 = checkpoints_y(i+2);
    z3 = checkpoints_z(i+2);
    t = linspace(0,1,resolution);
    mx1 = (1-tension)*(x2-x0)/2;
    my1 = (1-tension)*(y2-y0)/2;
    mz1 = (1-tension)*(z2-z0)/2;
    mx2 = (1-tension)*(x3-x1)/2;
    my2 = (1-tension)*(y3-y1)/2;
    mz2 = (1-tension)*(z3-z1)/2;
%     my1 = 0;
%     mz1 = 0;
%     mx1 = 0;
%     mx2 = 0;
%     my2 = 0;
%     mz2 = 0;

    for j = 1:length(t)-1
            x = x1*(2*t(j)^3-3*t(j)^2+1) +...
                x2*(-2*t(j)^3+3*t(j)^2) + ...
                mx1*(t(j)^3-2*t(j)^2+t(j)) + ...
                mx2*(t(j)^3-t(j)^2);
            y = y1*(2*t(j)^3-3*t(j)^2+1) + ...
                y2*(-2*t(j)^3+3*t(j)^2) + ...
                my1*(t(j)^3-2*t(j)^2+t(j)) + ...
                my2*(t(j)^3-t(j)^2);
            z = z1*(2*t(j)^3-3*t(j)^2+1) + ...
                z2*(-2*t(j)^3+3*t(j)^2) + ...
                mz1*(t(j)^3-2*t(j)^2+t(j)) + ...
                mz2*(t(j)^3-t(j)^2);
        spline_x = [spline_x, x];
        spline_y = [spline_y, y];
        spline_z = [spline_z, z];
%         if (j==1)
%             disp([x,y,z]);
%         end
%         if (j==length(t)-1)
%             disp([x,y,z]);
%         end
    end 
    clear j
end
clear i
plot3(spline_x, spline_y, spline_z, 'r')


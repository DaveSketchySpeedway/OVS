%% init
close all
clear
clc

% global frame
position = [0 0 0];
orientation = [1 0 0 0];

% body frame
angvel_body = [0.2 0.2 0];
linvel_body = [0.5 0 0];
timestep = 0.1;   

% visualization
i0 = [0 1 0 0];
j0 = [0 0 1 0];


figure
p0 = position;
plot3([p0(1) p0(1)+i0(2)],[p0(1) p0(2)+i0(3)], [p0(3), p0(3)+i0(4)],'r');
hold on
plot3([p0(1) p0(1)+j0(2)],[p0(1) p0(2)+j0(3)], [p0(3), p0(3)+j0(4)],'r');
axis([-10,10,-10,10,-10,10]);
xlabel('x')
ylabel('y')
zlabel('z')
hold on
grid on


%%
while true
    vecrotmat = quaternion2rotmat(orientation);
    angvel = [vecrotmat * angvel_body']';
    linvel = [vecrotmat * linvel_body']';
    
    % update orientation
    beta = norm(angvel) * timestep;
    unit = angvel ./ norm(angvel);
    rotation = [cos(beta/2), unit.*sin(beta/2)];
    orientation = quaternion_mul(rotation, orientation);
    
    % update position
    position = position + linvel.*timestep;
    
    % update visualization
%     i = quaternion_mul(orientation,...
%                     quaternion_mul(i0 , quaternion_inv(orientation)));
%     j = quaternion_multi(orientation,...
%                     quaternion_mul(j0 , quaternion_inv(orientation)));
    i = zeros(1,4);
    j = zeros(1,4);
    i(2:end) = [vecrotmat * i0(2:end)']';
    j(2:end) = [vecrotmat * j0(2:end)']';

    %checks
    det(vecrotmat)
    norm(angvel)
    norm(unit)
    norm(rotation)
    norm(orientation)
    norm(i(2:end))
    norm(j(2:end))
    
    % show
    plot3([position(1) position(1)+i(2)],...
        [position(2) position(2)+i(3)],...
        [position(3), position(3)+i(4)],'g');
    plot3([position(1) position(1)+j(2)],...
        [position(2) position(2)+j(3)],...
        [position(3), position(3)+j(4)],'b');
    pause(timestep)
end




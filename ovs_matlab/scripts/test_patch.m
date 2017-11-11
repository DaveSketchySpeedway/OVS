close all
clear
clc

%% construct shell

% up front
uf_vertices = [-0 -1 1; 2 -1 1; 2 1 1; -0 1 1];
uf_color = [0 1 0];

% up back
ub_vertices = [-2 -1 1; 0 -1 1; 0 1 1; -2 1 1];
ub_color = [1 0 0];

% front
f_vertices = [2 -1 0; 2 1 0; 2 1 1; 2 -1 1];
f_color = [0 1 0];

% back
b_vertices = [-2 -1 0; -2 1 0; -2 1 1; -2 -1 1];
b_color = [0 0 1];

% right
r_vertices = [-2 -1 0; 2 -1 0; 2 -1 1; -2 -1 1];
r_color = [0 0 1];

% left
l_vertices = [-2 1 0; 2 1 0; 2 1 1; -2 1 1];
l_color = [0 0 1];

% down
d_vertices = [-2 -1 0; 2 -1 0; 2 1 0; -2 1 0];
d_color = [0 0 0];

% combine
vertices = [uf_vertices;...
    ub_vertices;...
    f_vertices;...
    b_vertices;...
    r_vertices;...
    l_vertices;...
    d_vertices];
colors = [uf_color;...
    ub_color;...
    f_color;...
    b_color;...
    r_color;...
    l_color;...
    d_color];

m = size(colors,1); % num faces
n = 4; % num vertices per face
faces = zeros(m,n);
k = 1;
for i = 1:m
    for j = 1:n
        faces(i,j) = k;
        k = k+1;
    end
end

%% init
figure
p = patch('Faces', faces,...
    'Vertices', vertices,...
    'FaceVertexCData', colors,...
    'FaceColor','flat');
axis([-10,10, -10,10, -10,10]);
grid on
%% single
c = cos(0.001);
s = sin(0.001);
r = [c -s 0; s c 0; 0 0 1];
for i = 1:size(vertices,1);
    vertices(i,:) = [r * vertices(i,:)']';
end
set(p,'Vertices', vertices);
drawnow

%% batch
while true
    tic
    for i = 1:size(vertices,1);
    vertices(i,:) = [r * vertices(i,:)']';
    end
    set(p,'Vertices', vertices);
    drawnow limitrate
    toc
end




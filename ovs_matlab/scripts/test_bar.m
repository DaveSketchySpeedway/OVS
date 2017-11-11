clear
close all
clc

%%
% y = [2 2 3; 2 5 6; 2 8 9];
% c = categorical({'body_acc','body_vel','body_slip'});
% b = bar(c,y);
% get(b(1))

%%
y = [2 2 3 2 5 6 2 8 9];
b = bar(y);
get(b)
ylim([-5,5])
ylabel('asdf')
xlabel('fdsa')
set(gca,'xtick',[])


%%
tic
for i = 1:1000
    
    set(b,'YData',rand(9,1));
    drawnow limitrate
end
toc
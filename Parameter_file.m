% Parameter for 3-DoF Robot manipulation and locomotion
% load all parameters to base workspace before starting simulation
clc
close all
clear

% test trajectory
T = 200;
samplingtime = 0.001;
n = T/samplingtime;
i = 1;
count = 1;
T_index = 0:0.001:T;
position_ws = zeros(3,n+1);     % 2 rows for X and Y

while i <= n+1
    position_ws(2,i)= 0.2*cos(0.1*T_index(count))+0.68; % for X
    position_ws(3,i)= 0.2*sin(0.1*T_index(count)); % for Y
    count = count+1;
    i = i+1;
end
position_ws(1,:) = T_index;

%parameters
lowerBound      = [ -0.2; -0.2 ];

upperBound      = [ 0.25 ; 0.25 ];

l_const         = [ 0.8, 0.8; 
                    1.5, 1.5 ];

g_bar_const     = [ 100, 0.00;
                    0.00, 100 ];

g_bar_const_pinv = pinv(g_bar_const); % pseudo inverse




save('position_ws.mat',"position_ws");
% save('lowerbound','lowerBound');
% save('upperbound','upperBound');
% save('g_bar_const_pinv','g_bar_const_pinv')
% save('l_const','l_const')

% analize the output of the C implementation of the Fuzzy Controller for
% Wheeled Mobile Robot Trajectory Tracking
close all; clear all; clc;
in = load('input/testInput43B.txt');
out = load('output/testOutput43B.txt');
figure;
set(gcf, 'color', 'w');
plot(in(:,1), in(:,2), '.r'); hold on;
plot(out(:,1), out(:,2), '.b'); grid on;
legend('Reference trajectory','Robot trajectory');

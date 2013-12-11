% create the squared soccer field using a given size 
function fig = generate_field(fsize)
close all; 
%% draw center circle
t = linspace(0,2*pi,100000);
cirX=fsize/2; cirY=fsize/2;
r=50;
x = r*cos(t)+cirX;
y = r*sin(t)+cirY;
plot(x,y, 'Color','white', 'LineWidth', 4); hold on;
%% left post
t2 = linspace(3*pi, 3*pi/2,100000);
cirX2=0; cirY2=150;
x2 = r*cos(t2)+cirX2;
y2 = r*sin(t2)+cirY2;
plot(x2, y2, 'Color', 'white', 'LineWidth', 4); hold on;
%% right post
t3 = linspace(4*pi, 3*pi/2,100000);
cirX3=300; cirY3=150;
x3 = r*cos(t3)+cirX3;
y3 = r*sin(t3)+cirY3;
plot(x3, y3, 'Color', 'white', 'LineWidth', 4); hold on;
%% final config
xlim([0,fsize]);
ylim([0,fsize]);
set(gca,'Color',[0.2 0.7 0]);
set(gcf,'Color','w');
set(gca, 'Box', 'off'); 
grid on;
fig = gcf;
end
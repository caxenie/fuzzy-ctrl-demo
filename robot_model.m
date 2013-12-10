function robot_icon=robot_model(date)
L = 0.61;
Lm = 1;
Lc = 0.2;
robot_icon=[];

for i=1:length(date(:,1))
    
    x0 = date(i,1);
    y0 = date(i,2);
    theta0 = date(i,3);
    
    xc = x0-Lc*cos(theta0);
    yc = y0-Lc*sin(theta0);
    
    x1 = xc+L/2*sin(theta0);
    y1 = yc-L/2*cos(theta0);
    
    x4 = xc-L/2*sin(theta0);
    y4 = yc+L/2*cos(theta0);
    
    x5 = xc+Lm*cos(theta0);
    y5 = yc+Lm*sin(theta0);
    
    x2 = x5+L/2*sin(theta0);
    y2 = y5-L/2*cos(theta0);
    
    x3 = x5-L/2*sin(theta0);
    y3 = y5+L/2*cos(theta0);
    
    A=[x1 x2 x3 x4 x1 x5 x4;y1 y2 y3 y4 y1 y5 y4]';
    robot_icon=[robot_icon; A];
end
% start the trajectory generator to generate reference trajectories for
% fuzzy control of WMR in robot soccer
function start_trajectory_generator(ttype)

global kkkk xxx yyy teta P v_final;

switch ttype
    % loop circle _|^|O_
    case 'complex1';
        xxx     = [15, 20, 40, 60, 80, 100, 120, 100, 80, 100, 140];
        yyy     = [15, 15, 35, 55, 35, 15, 35, 55, 35, 15, 15];
        teta    = [0, 0, pi/2, 0, -pi/2, 0, pi/2, pi, -pi/2, 0, 0];
        
        % complex 2
    case 'complex2';
        xxx     = [15,20,40,60,80,100,120,100,80,80,80,100, 120, 140, 160, 140, 120, 140, 160, 180];
        yyy     = [14,14,20,40,50,80,60,40,60,80,100,120, 100, 80, 60, 40, 20,14,14, 14];
        teta    = [0,0,pi/2,0, pi/2, 0, -pi/2,pi,pi/2,pi/2, pi/2, 0, -pi/2, 0, -pi/2, pi, -pi/2, 0, 0, 0 ];
        
        % % oblique sine
    case 'sine';
        xxx      = [50,   60,   80,  100,   120,  140,   160];
        yyy      = [55,   60,   80,  100,   120,  140,   160];
        teta     = [0,  pi/2, 0, pi/2, 0,  pi/2,  0];
        
        % % circle
    case 'circle';
        xxx =  [28, 30,   40, 60,       80,   100,     80,     60];
        yyy =  [75, 75,   75, 95,       115,   95,     75,     95];
        teta = [0, 0,   0, pi/2,    0,  -pi/2,  pi,   pi/2];
        
        % % simple line
    case 'line';
        xxx     = [10,30,50,70,90,110,130,150,170,190,200,210,220,230,240];
        yyy     = [25,25,25,25,25,25,25,25,25,25,25,25,25,25,25];
        teta    = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
        
        % line half loop
    case 'line_half_loop';
        xxx   = [50, 100, 125, 100];
        yyy   = [50, 50, 75, 100];
        teta  = [0, 0, pi/2, pi];
end
P = [xxx;yyy];

kkkk=zeros(2,length(P));
v_final=zeros(1,length(P));

trajectory_generator(v_final, ttype);
end
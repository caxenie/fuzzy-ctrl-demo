clear all; clear; clc; close all;

global kkkk xxx yyy teta P v_final;

% loop circle _|^|O_
% ttype   = 'complex1';
% xxx     = [0, 20, 40, 60, 80, 100, 120, 100, 80, 100, 104];
% yyy     = [0, 0, 20, 40, 20, 0, 20, 40, 20, 0, 0];
% teta    = [0, 0, pi/2, 0, -pi/2, 0, pi/2, pi, -pi/2, 0, 0];

% complex 2
% ttype   = 'complex2';
% xxx     = [0,20,40,60,80,100,120,100,80,80,80,100, 120, 140, 160, 140, 120, 140, 160, 180];
% yyy     = [0,0,20,40,50,80,60,40,60,80,100,120, 100, 80, 60, 40, 20, 0, 0, 0];
% teta    = [0,0,pi/2,0, pi/2, 0, -pi/2,pi,pi/2,pi/2, pi/2, 0, -pi/2, 0, -pi/2, pi, -pi/2, 0, 0, 0 ];

% % oblique sine
ttype = 'sine';
xxx      = [10,   20,   40,  60,   80,  100,   120];
yyy      = [15,   20,   40,  60,   80,  100,   120];
teta     = [0,  pi/2, 0, pi/2, 0,  pi/2,  0];

% % circle 
% ttype = 'circle';
% xxx =  [0, 10,   20, 40,       60,   80,     60,     40];
% yyy =  [0, 0,   0, 20,       40,   20,     0,     20];
% teta = [0, 0,   0, pi/2,    0,  -pi/2,  pi,   pi/2];

% % simple line 
% ttype   = 'line';
% xxx     = [10,30,50,70,90,110,130,150,170,190,210,230,250,270,290];
% yyy     = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
% teta    = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];

% line half loop
% ttype = 'line_half_loop';
% xxx   = [0, 20, 25, 20];
% yyy   = [0, 0, 5, 10];
% teta  = [0, 0, pi/2, pi];

P = [xxx;yyy];

kkkk=zeros(2,length(P));
v_final=zeros(1,length(P));

trajectory_generator(v_final, ttype);

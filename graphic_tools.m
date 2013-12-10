function g=graphic_tools(vi,ai,vf,af,v2,v3,a2,tt,L)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   initial conditions  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
a0=ai;
v0=vi;
a5=af;
v5=vf;
%%%%%%%%%%%%%%%%%%%%%%%%%%%


% tt=[0.8034 0.8 0.7966 0.8 0.8];

t1=tt(1);
t2=tt(2);
t3=tt(3);
t4=tt(4);
t5=tt(5);

tt=([t1 t2 t3 t4 t5]);

a(1,1)=vi;
a(1,2)=(2*v2*t1+2*vi*t2+(ai-a2)*t1*t2)/(2*(t1+t2));
a(1,3)=v2;
a(1,4)=v3;
a(1,5)=(2*(vf*t4+v3*t5)*t3+(2*v3-2*v2-a2*t3-af*t3)*t4*t5)/(2*t3*(t4+t5));

a(2,1)=ai/2;
a(2,2)=(-ai*t1-a2*t2-2*vi+2*v2)/(2*(t1+t2));
a(2,3)=a2/2;
a(2,4)=(-a2*t3-2*v2+2*v3)/(2*t3);
a(2,5)=(2*(v2-v3)*t4-(2*v3-2*vf-a2*t4+af*t5)*t3)/(2*t3*(t4+t5));

a(3,1)=(-2*ai*t1-(ai+a2)*t2-2*vi+2*v2)/(6*t1*(t1+t2));
a(3,2)=((ai+a2)*t1+2*a2*t2+2*vi-2*v2)/(6*t2*(t1+t2));
a(3,3)=(-2*a2*t3+2*v3-2*v2)/(6*t3*t3);
a(3,4)=(2*(v2-v3)*(t5+2*t4)+(2*a2*t4-2*(v3-vf)+(a2-af)*t5)*t3)/(6*t3*t4*(t4+t5));
a(3,5)=((2*v3-2*vf-a2*t4+af*t4+2*af*t5)*t3-2*(v2-v3)*t4)/(6*t3*t5*(t4+t5));

%   a=[0 0.5264 1.2198 1.3412 0.8969;0 0.6552 0.2116 -0.0593 -0.4961; 0.2719 -0.1848 -0.1134 -0.1820 0.2067];
%%%%%%%%%%
%%  t1  %%
%%%%%%%%%%


m=1;
for l=0:0.01:t1
    aa1(1,m)=l;                                 % the time vector
    aa1(2,m)=a(1,1)+2*a(2,1)*l+3*a(3,1)*l^2;    % the velocity vector
    aa1(3,m)=2*a(2,1)+6*a(3,1)*l;               % the acceleration vector
    aa1(4,m)=a(1,1)*l+a(2,1)*l^2+a(3,1)*l^3;    % the length segments vector
    aa1(5,m)=abs(6*a(3,1));                     % the jerk vector
    m=m+1;
end

%%%%%%%%%%
%%  t2  %%
%%%%%%%%%%

m=1;
for l=0.01:0.01:t2
    aa2(1,m)=aa1(1,length(aa1))+l;
    aa2(2,m)=a(1,2)+2*a(2,2)*l+3*a(3,2)*l^2;
    aa2(3,m)=2*a(2,2)+6*a(3,2)*l;
    aa2(4,m)=(aa1(4,length(aa1))+a(1,2)*l+a(2,2)*l^2+a(3,2)*l^3);
    % aa2(4,m)=a(1,2)*l+a(2,2)*l^2+a(3,2)*l^3;
    aa2(5,m)=abs(6*a(3,2));
    m=m+1;
end

%%%%%%%%%%
%%  t3  %%
%%%%%%%%%%


m=1;
for l=0.01:0.01:t3
    aa3(1,m)=aa2(1,length(aa2))+l;
    aa3(2,m)=a(1,3)+2*a(2,3)*l+3*a(3,3)*l^2;
    aa3(3,m)=2*a(2,3)+6*a(3,3)*l;
    aa3(4,m)=(aa2(4,length(aa2))+a(1,3)*l+a(2,3)*l^2+a(3,3)*l^3);
    % aa3(4,m)=a(1,3)*l+a(2,3)*l^2+a(3,3)*l^3;
    aa3(5,m)=abs(6*a(3,3));
    m=m+1;
end

%%%%%%%%%%
%%  t4  %%
%%%%%%%%%%


m=1;
for l=0.01:0.01:t4
    aa4(1,m)=aa3(1,length(aa3))+l;
    aa4(2,m)=a(1,4)+2*a(2,4)*l+3*a(3,4)*l^2;
    aa4(3,m)=2*a(2,4)+6*a(3,4)*l;
    aa4(4,m)=(aa3(4,length(aa3))+a(1,4)*l+a(2,4)*l^2+a(3,4)*l^3);
    % aa4(4,m)=a(1,4)*l+a(2,4)*l^2+a(3,4)*l^3;
    aa4(5,m)=abs(6*a(3,4));
    m=m+1;
end

%%%%%%%%%%
%%  t5  %%
%%%%%%%%%%


m=1;
for l=0.01:0.01:t5
    aa5(1,m)=aa4(1,length(aa4))+l;
    aa5(2,m)=a(1,5)+2*a(2,5)*l+3*a(3,5)*l^2;
    aa5(3,m)=2*a(2,5)+6*a(3,5)*l;
    aa5(4,m)=aa4(4,length(aa4))+a(1,5)*l+a(2,5)*l^2+a(3,5)*l^3;
    % aa5(4,m)=a(1,5)*l+a(2,5)*l^2+a(3,5)*l^3;
    aa5(5,m)=abs(6*a(3,5));
    m=m+1;
end

aa5(4,length(aa5))=round2(aa5(4,length(aa5)));

aa=[aa1 aa2 aa3 aa4 aa5];
S=aa(4,length(aa));
g=aa;

yh = 0:0.01:(t1+t2+t3+t4+t5);

% figure(1)
% plot(yh,aa(2,:),'r',3,aa1(2,length(aa1)),'bo',6,aa2(2,length(aa2)),'bo',9,aa3(2,length(aa3)),'bo',12,aa4(2,length(aa4)),'bo',15,aa5(2,length(aa5)),'bo',0,0.45,'bo')
% hold on
% line([3 3],[0 aa1(2,length(aa1))])
% line([6 6],[0 aa2(2,length(aa2))])
% line([9 9],[0 aa3(2,length(aa3))])
% line([12 12],[0 aa4(2,length(aa4))])
% line([15 15],[0 aa5(2,length(aa5))])
% hold off
% axis([0 17 0 1.5])
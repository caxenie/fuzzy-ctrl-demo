function v=parametrize_final_velocity(xi,yi,xf,yf,L,t_fin,vi,vf,ai,af,tetaA,l,KK,x_init,y_init)


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   initial conditions  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
a0=ai;
v0=vi;
a5=af;
v5=vf;
%%%%%%%%%%%%%%%%%%%%%%%%%%%


t_gama=round2(t_fin/5);

if ai < 0 & vi > 0
    t_beta=2*vi/ai;
elseif af > 0 & vf > 0
    t_beta=2*vf/af;
else
    t_beta=t_gama;
end

%   t1=min(t_gama,t_beta);
t1=t_gama;
t2=t1;
t4=t2;
t5=t4;
t3=t_fin-t1-t2-t4-t5;
t=round2([t1 t2 t3 t4 t5]);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FIND a starting solution  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v=parametrize_velocity(L,vi,vf,ai,af,t);
v2=v(1);
v3=v(2);
a2=v(3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

aa=graphic_tools(vi,ai,vf,af,v2,v3,a2,t,L);
t_fin=t(1)+t(2)+t(3)+t(4)+t(5);

%_______________________________________________

l_init=interp1(0:0.01:1,l,0:0.01/t_fin:1,'spline');
l_init(length(l_init))=L;
KK_init=interp1(0:0.01:1,KK,0:0.01/t_fin:1,'spline');

tt2=interp1(aa(4,:),aa(1,:),l_init,'spline');
speed=interp1(aa(1,:),aa(2,:),tt2,'spline');
omega3=KK_init.*speed;
omega4=interp1(tt2,omega3,0:0.01:t_fin,'spline');
%omega4(length(omega4))=0;

%_______________________________________________

v=[aa;omega4;KK_init];


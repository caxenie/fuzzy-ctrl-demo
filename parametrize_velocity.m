function v=parametrize_velocity(sf,vi,vf,ai,af,t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FIND a starting solution  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

v0=vi;
v5=vf;
a0=ai;
a5=af;

v2=0;
v3=v2;
a2=0;

t1=t(1);
t2=t(2);
t3=t(3);
t4=t(4);
t5=t(5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 S=0;
   
    while S ~= sf
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% THE SIMPLIFIED CURVE PARAMETRIZATION %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    clear aa1 aa2 aa3 aa4 aa5 aa

        a(1,1)=v0;
        a(1,2)=(2*(v2*t1+v0*t2)+a0*t1*t2)/(2*(t1+t2));
        a(1,3)=v2;
        a(1,4)=v2;
        a(1,5)=(2*(v2*t5+v5*t4)-a5*t4*t5)/(2*(t4+t5));

        a(2,1)=a0/2;
        a(2,2)=(2*(v2-v0)-a0*t1)/(2*(t1+t2));
        a(2,3)=0;
        a(2,4)=0;
        a(2,5)=(2*(v5-v2)-a5*t5)/(2*(t4+t5));
    
        a(3,1)=(2*(v2-v0-a0*t1)-a0*t2)/(6*t1*(t1+t2));
        a(3,2)=-(2*(v2-v0)-a0*t1)/(6*t2*(t1+t2));
        a(3,3)=0;
        a(3,4)=(-a5*t5-2*v2+2*v5)/(6*t4*(t4+t5));
        a(3,5)=(2*(v2-v5+a5*t5)+a5*t4)/(6*t5*(t4+t5));

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        %%%%%%%%%%
        %%  t1  %%
        %%%%%%%%%%

        for m=1:5
            va(m)=a(1,m)+2*a(2,m)*t(m)+3*a(3,m)*(t(m))^2;          % the velocity vector
            aa(m)=2*a(2,m)+6*a(3,m)*t(m);                          % the acceleration vector
            sa(m)=a(1,m)*t(m)+a(2,m)*(t(m))^2+a(3,m)*(t(m))^3;     % the length segments vector
            ja(m)=abs(6*a(3,m));                                   % the jerk vector
        end

        v1=va(1);
        v2=va(2);
        v3=va(3);
        v4=va(4);
        v5=va(5);
        a2=aa(2);

        s1=sa(1);
        s2=sa(2);
        s3=sa(3);
        s4=sa(4);
        s5=sa(5);
  
        S=round2(s1+s2+s3+s4+s5);

        if abs(sf-S) > 1
            if S < sf
                v2=v2+0.05;
            elseif S ~= sf
                v2=v2-0.005;
            end
        else

            if S < sf
                v2=v2+0.0001;
            elseif S ~= sf
                v2=v2-0.00001;
            end
        end    
    end
    
S_vitee=S;
v=[va(2) va(3) aa(2)];
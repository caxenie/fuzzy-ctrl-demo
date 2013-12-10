function trajectory_generator(v_fin, ttype)

global kkkk xxx yyy teta P v_final;

k=zeros(1,length(xxx));
v_in=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L=0;
ttt=[];
LL=[];
all=[];
vv=[];
aWW=[];
aLT=[];
aLT_rms=[];
KKK=[];
point=[];
XY=[];
Kl=[];
t_fin=0;
Theta(1)=0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for g=1:length(P)-1
    
    tetaA=teta(g);
    tetaB=teta(g+1);
    
    xA=P(1,g);
    yA=P(2,g);
    
    xB=P(1,g+1);
    yB=P(2,g+1);
    
    kA=k(g);
    kB=k(g+1);
    
    kA_d=0;
    kB_d=0;
    
    g1=norm(P(:,g)-P(:,g+1));
    g2=g1;
    g3=0;
    g4=-g3;
    
    x0 = xA;
    x1 = g1*cos(tetaA);
    x2 = 1/2*(g3*cos(tetaA)-(g1)^2*kA*sin(tetaA));
    x3 = 10*(xB-xA)-(6*g1 + 3/2*g3)*cos(tetaA) - (4*g2 - (1/2)*g4 )*cos(tetaB) + (3/2)*(g1)^2*kA*sin(tetaA) - (1/2)*(g2)^2*kB*sin(tetaB);
    x4 = -15*(xB - xA) + (8*g1 + (3/2)*g3)*cos(tetaA) + (7*g2 - g4)*cos(tetaB) - (3/2)*(g1)^2*kA*sin(tetaA) + (g2)^2*kB*sin(tetaB);
    x5 = 6*(xB - xA) - (3*g1 + (1/2)*g3)*cos(tetaA) - (3*g2 - (1/2)*g4)*cos(tetaB) + (1/2)*(g1)^2*kA*sin(tetaA)-(1/2)*(g2)^2*kB*sin(tetaB);
    
    y0 = yA;
    y1 = g1*sin(tetaA);
    y2 = 1/2*(g3*sin(tetaA)+(g1)^2*kA*cos(tetaA));
    y3 = 10*(yB-yA)-(6*g1 + (3/2)*g3)*sin(tetaA) - (4*g2 - (1/2)*g4 )*sin(tetaB) - (3/2)*(g1)^2*kA*cos(tetaA) + (1/2)*(g2)^2*kB*cos(tetaB);
    y4 = -15*(yB - yA) + (8*g1 + (3/2)*g3)*sin(tetaA) + (7*g2 - g4)*sin(tetaB) + (3/2)*(g1)^2*kA*cos(tetaA) - (g2)^2*kB*cos(tetaB);
    y5 = 6*(yB - yA) - (3*g1 + (1/2)*g3)*sin(tetaA) - (3*g2 - (1/2)*g4)*sin(tetaB) - (1/2)*(g1)^2*kA*cos(tetaA)+(1/2)*(g2)^2*kB*cos(tetaB);
    
    X=[x0 x1 x2 x3 x4 x5];
    Y=[y0 y1 y2 y3 y4 y5];
    
    clear l
    n=1;
    l(1)=0;
    
    for u=0:0.01:1
        alfa(n)=x0+x1*u+x2*u^2+x3*u^3+x4*u^4+x5*u^5;
        beta(n)=y0+y1*u+y2*u^2+y3*u^3+y4*u^4+y5*u^5;
        
        if n > 1
            l(n) = l(n-1) + sqrt((alfa(n) - alfa(n-1))^2 + (beta(n) - beta(n-1))^2);
        end
        
        x_d(n) = x1 + 2*x2*u + 3*x3*u^2 + 4*x4*u^3 + 5*x5*u^4;
        y_d(n) = y1 + 2*y2*u + 3*y3*u^2 + 4*y4*u^3 + 5*y5*u^4;
        
        x_d_d(n) = 2*x2 + 6*x3*u + 12*x4*u^2 + 20*x5*u^3;
        y_d_d(n) = 2*y2 + 6*y3*u + 12*y4*u^2 + 20*y5*u^3;
        
        KK(n)=(x_d(n)*y_d_d(n) - x_d_d(n)*y_d(n))/((sqrt((x_d(n))^2 + (y_d(n))^2))^3);
        
        if u<1
            alfa_1(n)=alfa(n);
            beta_1(n)=beta(n);
        end
        
        n = n + 1;
    end
    
    LLL=round2(l(length(l)));
    l(length(l))=LLL;
    LL=[LL l(length(l))];
    L=L+LLL;
    
    XXYY=[X;Y];
    XY=[XY XXYY];
    
    KKK=[KKK [alfa_1;beta_1]];
    Kl=[Kl [alfa;beta;KK;l]];
    
    tfin=round2(sqrt(2*LLL/0.11)); % 0.21   0.18 %LINIE 0.08, LAST 0.21 aL <= 0.24 m/sec^2 ---- 0.16  ---- 0.02
    
    if g==kkkk(1,g)
        tfin=kkkk(2,g)+kkkk(2,g)*10/100 % 10%
        
    end
    
    if v_fin(g)==0
        v_fin(g) = LLL/(tfin);
    end
    
    if g==length(P)-1
        v_fin(g)=0;
    end
    
    
    aW = 4;
    
    while aW > 0.20 %0.31 0.25
        
        V=parametrize_final_velocity(xA,yA,xB,yB,LLL,tfin,v_in,v_fin(g),0,0,tetaA,l,KK,alfa,beta);
        
        kx=1.4;
        ky=1.4;
        
        aT=V(3,:);
        aL=V(6,:).*(V(2,:));
        
        aL_rms = sqrt((sum(aL.^2))/length(aL));
        aT_rms = sqrt((sum(aT.^2))/length(aT));
        
        aW=sqrt(kx^2*aT_rms^2 + ky^2*aL_rms^2);
        
        if aW > 0.20 %0.31  0.25
            %aWW=aW
            tfin=tfin+tfin*2.5/100;
            
            
            
            if min(V(2,:))<-0.005
                if g==1
                    v_fin(g)=v_fin(g)-0.05
                    trajectory_generator(v_fin);
                    return;
                elseif v_fin(g-1)>0.05 & v_fin(g-1) > v_fin(g)
                    v_fin(g-1)=v_fin(g-1)-0.4;
                    v_fin(g)=0
                    % v_in=v_fin(g-1)
                    trajectory_generator(v_fin);
                else v_fin(g)<0.05
                    v_fin(g)=v_fin(g)-0.05;
                    %v_in=v_fin(g-1)
                    trajectory_generator(v_fin);
                    return;
                end
            end
        end
        
    end %(WHILE)
    
    t_fin=t_fin+tfin;
    ttt=[ttt tfin];
    
    aLT1=[aL; aT];
    aLT=[aLT aLT1];
    
    aLT1_rms=[aL_rms; aT_rms];
    aLT_rms=[aLT_rms aLT1_rms];
    aWW=[aWW aW];
    
    vv=[vv V];
    v_in=v_fin(g);
    point = [point [v_fin(g); t_fin]];
    
end % (FOR)

aT_total=vv(3,:);
aL_total=vv(6,:).*vv(2,:);

aL_rms_total = sqrt((sum(aL_total.^2))/length(aL_total));
aT_rms_total = sqrt((sum(aT_total.^2))/length(aT_total));
aW_total=sqrt(kx^2*aT_rms_total^2 + ky^2*aL_rms_total^2);

x_rrr(1)=P(1,1);
y_rrr(1)=P(2,1);
teta_rrr(1)=teta(1);

pass=0.01;

mm=1;

for m=0:pass:t_fin
    teta_rrr(mm+1)=pass*(vv(6,mm))+teta_rrr(mm);
    
    if (teta_rrr(mm+1)<-pi)
        teta_rrr(mm+1)=teta_rrr(mm+1)+2*pi;
    elseif (teta_rrr(mm+1)>pi)
        teta_rrr(mm+1)=teta_rrr(mm+1)-2*pi;
    end
    
    x_rrr(mm+1)=pass*vv(2,mm)*cos(teta_rrr(mm))+x_rrr(mm);
    y_rrr(mm+1)=pass*vv(2,mm)*sin(teta_rrr(mm))+y_rrr(mm);
    mm=mm+1;
end

XYTheta = [x_rrr; y_rrr; teta_rrr];
time_total=0:0.01:length(vv)*0.01-0.01;

nr_points=round(t_fin/10);
x_inter=interp1(0:0.01:t_fin+0.01,x_rrr,[5 10 15 20],'spline');
y_inter=interp1(0:0.01:t_fin+0.01,y_rrr,[5 10 15 20],'spline');


AA=robot_model(KKK);

figure(1)
plot(Kl(1,:),Kl(2,:),'g',x_rrr,y_rrr,'r',P(1,:),P(2,:),'ob','LineWidth',2)
% plot(Kl(2,:),Kl(1,:),'g',y_rrr,x_rrr,'r',P(2,:),P(1,:),'ob','LineWidth',2)
title('Path','fontsize',12);
xlabel('x [cm]','fontsize',12);
ylabel('y [cm]','fontsize',12);
grid;

% figure(2);
% %  subplot(2,1,1)
% plot(time_total,vv(2,:),'r', time_total,vv(6,:),'--b'); %,point(2,:),point(1,:),'b*')
% xlabel('time [s]','fontsize',12);
% ylabel('Velocity','fontsize',12);
% legend('Linear velocity [m/s]','Angular velocity [rad/s]');
%      subplot(2,1,2);
%         plot(time_total,aLT(2,:),'r', time_total,aLT(1,:),'--b')
%         xlabel('time [s]','fontsize',12)
%         ylabel('Acceleration [m/s^2]','fontsize',12)
%         legend('Longitudinal accel.','Lateral accel.')
%
% figure(3)
%     bar([aLT_rms(1,:);aLT_rms(2,:);aWW]')
%     legend('Lat. rms accel.','Long. rms accel.','Overal rms accel.')

timee=0:0.01:length(vv(1,:))*0.01-0.01;
velocity=[vv];
%   speed = velocity(2,1:5:length(velocity))';
%   acceler = velocity(3,1:5:length(velocity))';
%   angular_velo = velocity(6,1:5:length(velocity))';

speed = velocity(2,1:10:length(velocity))';
acceler = velocity(3,1:10:length(velocity))';
angular_velo = velocity(6,1:10:length(velocity))';

acc_angular = (angular_velo.*speed);

K2=XYTheta(:,1:50:length(XYTheta))';  %50
KKK=K2;

% save everything in a struct
ref_traj.xr     = KKK(:,1);
ref_traj.yr     = KKK(:,2);
ref_traj.thetar = KKK(:,3);

% throw the data into a mat file as well 
save(sprintf('reference_trajectory_data_%s.mat', ttype),'ref_traj');

% dlmwrite('xytheta_desired.txt',KKK,'coffset',1);
fid = fopen(sprintf('reference_profile_%s.h', ttype),'wt');

fprintf(fid,'double len=%d; \n',length(KKK));
fprintf(fid,'static double reference_trajectory[][3]={ \n');

for h=1:length(KKK)
    fprintf(fid,'%2.8f, %2.8f, %2.8f\n', KKK(h, 1), KKK(h, 2), KKK(h,3));
end

fprintf(fid,'};\n');

fclose(fid);

%
% date = [speed, acceler, angular_velo, acc_angular];
% date2 = [speed, angular_velo];
%
% dlmwrite('velo_desired.txt',date,'coffset',1);
% save lin_velo_wheelchair speed -ASCII ;
% save angular_velo_wheelchair angular_velo -ASCII ;
% save velo date -v4,
% save acceler acceler -v4;
%
% fid = fopen('G:\Velocity\reference_profile.h','wt');
%
% fprintf(fid,'double len=%d; \n',length(speed));
% fprintf(fid,'static double reference_profile[][4]={ \n');
%
% for h=1:length(speed)-1
%     fprintf(fid,'%2.8f, %2.8f, %2.8f, %2.8f, \n',speed(h), acceler(h), angular_velo(h), acc_angular(h));
% end
%
% fprintf(fid,'%2.8f, %2.8f, %2.8f, %2.8f}; \n',speed(h+1), acceler(h+1), angular_velo(h+1), acc_angular(h+1));
%
% fclose(fid);

return;


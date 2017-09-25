clc
clear all
close all



L(1) = Link('revolute', 'a', 0, 'd', 10, 'alpha', 90*pi/180);
L(2) = Link('revolute', 'a', 10, 'd', 0, 'alpha', 0*pi/180);
L(3) = Link('revolute', 'a', 5, 'd', 5, 'alpha', 90*pi/180);
L(4) = Link('revolute', 'a', 0, 'd', 15, 'alpha', -90*pi/180);
L(5) = Link('revolute', 'a', 0, 'd', 0, 'alpha', 90*pi/180);
L(6) = Link('revolute', 'a', 0, 'd', 20, 'alpha', 0*pi/180);

sixlink = SerialLink(L);
matq(:,1)=[90*pi/180; 0*pi/180; 180*pi/180; 180*pi/180; 0*pi/180; 0*pi/180]+(1+2*rand(6,1))*pi/180;

for itntraj=2:10
   matq(:,itntraj)=matq(:,itntraj-1)+(1+2*rand(6,1))*pi/180;
end
fkobj = FKinRotMat(sixlink);
fkobj.compute(matq);
ikobj=IK_NR_NG(sixlink,[],[],100);
for itntraj=2:10
    q_com=ikobj.compute(fkobj.v_r_total(:,itntraj),fkobj.mat_rot_total(:,:,itntraj),matq(:,itntraj-1));
    q_com
    matq(:,itntraj)
end

% 
% matq(:,1)=[90*pi/180; 0*pi/180; 180*pi/180; 150*pi/180; 100*pi/180; 0*pi/180];
% matq(:,2)=[90*pi/180; 0*pi/180; 180*pi/180; 160*pi/180; 100*pi/180; 0*pi/180];
% matq(:,3)=[90*pi/180; 0*pi/180; 180*pi/180; 170*pi/180; 100*pi/180; 0*pi/180];
% matq(:,4)=[90*pi/180; 0*pi/180; 180*pi/180; 180*pi/180; 100*pi/180; 0*pi/180];
% matq(:,5)=[90*pi/180; 0*pi/180; 180*pi/180; 200*pi/180; 100*pi/180; 0*pi/180];
% figure
% for it =1:5
%     sixlink.RobotPlot(matq(:,it),2);
%     hold off
% end
% 
% matq1=[30*pi/180; 1.5; 240*pi/180; 80*pi/180; 0*pi/180; 0*pi/180];
% sixlinjacobe1=Jacobian(sixlink);
% sixlinjacobe1.compute(matq);
% sixlinjacobe1.Je_total
% sixlinjacobe1.J0_total
% 
% 
% 
% 
% matq2=[30*pi/180, 1.5, 240*pi/180, 80*pi/180, 2*pi/180, 10*pi/180];
% sixlinkinput2=ActuatorKin(matq2);
% sixlinjacobe2=Jacobian(sixlink,sixlinkinput2);
% sixlinjacobe2.compute;
% sixlinjacobe2.Je_total
% sixlinjacobe2.J0_total
% 
% 

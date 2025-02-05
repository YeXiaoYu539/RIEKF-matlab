% clc
% clear
close all
% drawfigure_newRIEKF;
% 
%load('Data\0823data01_220s.mat');
num = size(All.t,1);
%%

color4 = [252 160 93]/255;
color4 = [205 20 20]/255;
color5 = [190 190 190]/255;
color3 = [84 134 135]/255;
color1 = [71 51 53]/255;
color2 = [189 30 30]/255;
color6 = [252 160 93]/255;

r2d = 180/pi;

%%
figure(1)
subplot1 = subplot(3,1,1);
plot(All.t(1:num),All.phiL(1:num),'LineWidth',2,'Color',color1);

hold on;
plot(out.tout,out.NavData.CF_NavData.RotationData.phi_rad.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.RotationData.phi_rad.Data(:,1),'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.RotationData.phi_rad.Data(:,1),'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.RotationData.phi_rad.Data(:,1),'LineWidth',2,'Color',color4);
ylabel('$$\phi\,$$[deg]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ref','NCF','ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',5,'Location','north','FontSize',10);
legend('boxoff');
axis([0 250 -50 70]);
set(gca,'FontSize',12);

subplot1 = subplot(3,1,2);
plot(All.t(1:num),All.thetaL(1:num),'LineWidth',2,'Color',color1);
hold on;
plot(out.tout,out.NavData.CF_NavData.RotationData.theta_rad.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.RotationData.theta_rad.Data(:,1),'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.RotationData.theta_rad.Data(:,1),'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.RotationData.theta_rad.Data(:,1),'LineWidth',2,'Color',color4);
ylabel('$$\theta\,$$[deg]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on','FontSize',12);
legend({'ref','NCF','ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',5,'Location','north','FontSize',10);
legend('boxoff');
axis([0 250 -5 25]);
set(gca,'FontSize',12);

subplot1 = subplot(3,1,3);
plot(All.t(1:num),All.psiL(1:num),'LineWidth',2,'Color',color1);
hold on;
plot(out.tout,out.NavData.CF_NavData.RotationData.psi_rad.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.RotationData.psi_rad.Data(:,1),'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.RotationData.psi_rad.Data(:,1),'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.RotationData.psi_rad.Data(:,1),'LineWidth',2,'Color',color4);
ylabel('$$\psi\,$$[deg]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ref','NCF','ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',5,'Location','north','FontSize',10);
legend('boxoff');
axis([0 250 -200 280]);
set(gca,'FontSize',12);

%%
figure(2)
subplot1 = subplot(3,1,1);
%plot(All.t,All.vn,'LineWidth',2,'Color',color1);
plot(out.tout,out.NavData.SensorData.vn_gps.Data(:,1),'LineWidth',2,'Color',color1)
hold on;
plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color4);

ylabel('$$vel_n\,$$[m/s]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'RTK','NCF','ES-EKF','ES-LIEKF','ES-RIEKF',},'NumColumns',5,'Location','north','FontSize',10);
legend('boxoff');
axis([0 250 -30 40])
set(gca,'FontSize',12);

subplot2 = subplot(3,1,2);
plot(out.tout,out.NavData.SensorData.ve_gps.Data(:,1),'LineWidth',2,'Color',color1)
hold on;
plot(out.tout,out.NavData.CF_NavData.TranlationData.ve_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.TranlationData.ve_m_s.Data(:,1),'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.TranlationData.ve_m_s.Data(:,1),'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.TranlationData.ve_m_s.Data(:,1),'LineWidth',2,'Color',color4);
ylabel('$$vel_e\,$$[m/s]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot2,'on');
grid(subplot2,'on');
hold(subplot2,'off');
set(subplot2,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'RTK','NCF','ES-EKF','ES-LIEKF','ES-RIEKF',},'NumColumns',5,'Location','north','FontSize',10);
legend('boxoff');
axis([0 250 -25 40])
set(gca,'FontSize',12);

subplot3 = subplot(3,1,3);
plot(out.tout,-out.NavData.SensorData.vd_gps.Data(:,1),'LineWidth',2,'Color',color1)
hold on;
plot(out.tout,-out.NavData.CF_NavData.TranlationData.vd_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout,-out.NavData.ER_NavData.TranlationData.vd_m_s.Data(:,1),'LineWidth',2,'Color',color3);
plot(out.tout,-out.NavData.RI_NavData.TranlationData.vd_m_s.Data(:,1),'LineWidth',2,'Color',color5);
plot(out.tout,-out.NavData.RI_NavData.TranlationData.vd_m_s.Data(:,1),'LineWidth',2,'Color',color4);
ylabel('$$vel_h\,$$[m/s]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot3,'on');
grid(subplot3,'on');
hold(subplot3,'off');
set(subplot3,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'RTK','NCF','ES-EKF','ES-LIEKF','ES-RIEKF',},'NumColumns',5,'Location','north','FontSize',10);
legend('boxoff');
axis([0 250 -3 5])
set(gca,'FontSize',12);

%%
LineWidth = 1.5;
figure(3)
subplot1 = subplot(3,1,1);
plot(out.tout,out.NavData.SensorData.pos_N_gps.Data(:,1),'LineWidth',LineWidth,'Color',color1)
hold on;
plot(out.tout,out.NavData.CF_NavData.TranlationData.pn_m.Data(:,1),'LineWidth',LineWidth,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.TranlationData.pn_m.Data(:,1),'LineWidth',LineWidth,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.TranlationData.pn_m.Data(:,1),'LineWidth',LineWidth,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.TranlationData.pn_m.Data(:,1),'LineWidth',LineWidth,'Color',color4);
ylabel('$$pos_n\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'RTK','NCF','ES-EKF','ES-LIEKF','ES-RIEKF',},'NumColumns',5,'Location','north','FontSize',10);
legend('boxoff');
axis([0 250 -250 700])

set(gca,'FontSize',12);

subplot1 = subplot(3,1,2);
plot(out.tout,out.NavData.SensorData.pos_E_gps.Data(:,1),'LineWidth',LineWidth,'Color',color1)
hold on;
plot(out.tout,out.NavData.CF_NavData.TranlationData.pe_m.Data(:,1),'LineWidth',LineWidth,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.TranlationData.pe_m.Data(:,1),'LineWidth',LineWidth,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.TranlationData.pe_m.Data(:,1),'LineWidth',LineWidth,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.TranlationData.pe_m.Data(:,1),'LineWidth',LineWidth,'Color',color4);
ylabel('$$pos_n\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'RTK','NCF','ES-EKF','ES-LIEKF','ES-RIEKF',},'NumColumns',5,'Location','north','FontSize',10);
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -700 350])

subplot1 = subplot(3,1,3);
plot(out.tout,-out.NavData.SensorData.pos_D_gps.Data(:,1),'LineWidth',LineWidth,'Color',color1)
hold on;
plot(out.tout,out.NavData.CF_NavData.TranlationData.H_m.Data(:,1),'LineWidth',LineWidth,'Color',color6);
plot(out.tout,-out.NavData.ER_NavData.TranlationData.pd_m.Data(:,1),'LineWidth',LineWidth,'Color',color3);
plot(out.tout,-out.NavData.LI_NavData.TranlationData.pd_m.Data(:,1),'LineWidth',LineWidth,'Color',color5);
plot(out.tout,-out.NavData.RI_NavData.TranlationData.pd_m.Data(:,1),'LineWidth',LineWidth,'Color',color4);
ylabel('$$pos_h\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'RTK','NCF','ES-EKF','ES-LIEKF','ES-RIEKF',},'NumColumns',5,'Location','north','FontSize',10);
legend('boxoff');
axis([0 250 0 50])
set(gca,'FontSize',12);

%% AOA/SA
figure(4)
subplot1 = subplot(2,1,1)
plot(out.tout,out.NavData.RI_NavData.AOA_SA_Wind.AOA.Data(:,1),'LineWidth',2,'Color',color1);
hold on;
plot(out.tout,out.NavData.OtherData.AOA_direct_Deg.Data(:,1),'LineWidth',2,'Color',color3);
ylabel('$$\alpha\,$$[deg]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'RIEKF','Direct'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);

subplot1 = subplot(2,1,2)
plot(out.tout,out.NavData.RI_NavData.AOA_SA_Wind.SA.Data(:,1),'LineWidth',2,'Color',color1);
hold on;
plot(out.tout,convertdpsi(-out.NavData.OtherData.SA_direct_Deg.Data(:,1)),'LineWidth',2,'Color',color3);
ylabel('$$\beta\,$$[deg]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-RIEKF','Direct'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);

%% vel err
GNSSflag = out.NavData.RI_NavData.GNSS_enable_flag.GNSS_enable_flag.Data(:,1);
enable_index = find(GNSSflag == 1);
enable_index = enable_index(1:3:end)
figure(5)
interval = 20;
subplot1 = subplot(3,1,1);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
plot(out.tout(enable_index),abs(out.NavData.CF_NavData.TranlationData.vn_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vn_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),abs(out.NavData.ER_NavData.TranlationData.vn_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vn_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), abs(out.NavData.LI_NavData.TranlationData.vn_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vn_gps.Data(enable_index,1)),'LineWidth',2,'Color',color5);
plot(out.tout(enable_index), abs(out.NavData.RI_NavData.TranlationData.vn_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vn_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta vel_N\,[m/s]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'NCF','ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');

subplot1 = subplot(3,1,2);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
plot(out.tout(enable_index),abs(out.NavData.CF_NavData.TranlationData.ve_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.ve_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),abs(out.NavData.ER_NavData.TranlationData.ve_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.ve_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), abs(out.NavData.LI_NavData.TranlationData.ve_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.ve_gps.Data(enable_index,1)),'LineWidth',2,'Color',color5);
plot(out.tout(enable_index), abs(out.NavData.RI_NavData.TranlationData.ve_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.ve_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta vel_E\,[m/s]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'NCF','ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');

subplot1 = subplot(3,1,3);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
plot(out.tout(enable_index),abs(out.NavData.CF_NavData.TranlationData.vd_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vd_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),abs(out.NavData.ER_NavData.TranlationData.vd_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vd_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), abs(out.NavData.LI_NavData.TranlationData.vd_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vd_gps.Data(enable_index,1)),'LineWidth',2,'Color',color5);
plot(out.tout(enable_index), abs(out.NavData.RI_NavData.TranlationData.vd_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vd_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta vel_D\,[m/s]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'NCF','ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);

%% pos error
% gpsdely = 10;
% RIpos_N_gps_test(:,1) = [zeros(gpsdely,1);out.NavData.RI_NavData.TranlationData.pn_m.Data(1:end-gpsdely,1);];
pos_N_gps =  out.NavData.SensorData.pos_N_gps.Data(:,1) - 0.4;
figure('Name','pos error')
subplot1 = subplot(3,1,1);
interval = 20;
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
% plot(out.tout(enable_index),(out.NavData.CF_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
%     pos_N_gps(enable_index,1)),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),(out.NavData.ER_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    pos_N_gps(enable_index,1)) ,'LineWidth',2,'Color',color3);

plot(out.tout(enable_index), (out.NavData.LI_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    pos_N_gps(enable_index,1)),'LineWidth',2,'Color',color5);

plot(out.tout(enable_index), (out.NavData.RI_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    pos_N_gps(enable_index,1)),'LineWidth',2,'Color',color4);

ylabel('$$\delta pos_n\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -2 4]);

subplot1 = subplot(3,1,2);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),(out.NavData.ER_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), (out.NavData.LI_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1)),'LineWidth',2,'Color',color5);

plot(out.tout(enable_index), (out.NavData.RI_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta pos_e\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -3 4]);

subplot1 = subplot(3,1,3);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),(out.NavData.ER_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);

plot(out.tout(enable_index), (out.NavData.LI_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1)),'LineWidth',2,'Color',color5);

plot(out.tout(enable_index), (out.NavData.RI_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta pos_d\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -1.5 2]);

%% bias 
figure(Name='gyro bias')
subplot1 = subplot(3,1,1);
hold on;
plot(out.tout,out.NavData.CF_NavData.bias_g.Data(:,1)*r2d,'LineWidth',2,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.INSbiasData.INSstate_dg.Data(:,1)*r2d,'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.INSbiasData.INSstate_dg.Data(:,1)*r2d,'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.INSbiasData.INSstate_dg.Data(:,1)*r2d,'LineWidth',2,'Color',color4);
ylabel('$$bg_x\,$$[deg/s]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'NCF','ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
axis([0 250 -0.1 0.35])
set(gca,'FontSize',12);

subplot1 = subplot(3,1,2);
hold on;
plot(out.tout,out.NavData.CF_NavData.bias_g.Data(:,2)*r2d,'LineWidth',2,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.INSbiasData.INSstate_dg.Data(:,2)*r2d,'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.INSbiasData.INSstate_dg.Data(:,2)*r2d,'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.INSbiasData.INSstate_dg.Data(:,2)*r2d,'LineWidth',2,'Color',color4);
ylabel('$$bg_y\,$$[deg/s]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'NCF','ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
axis([0 250 -0.25 0.25])
set(gca,'FontSize',12);

subplot1 = subplot(3,1,3);
hold on;
plot(out.tout,out.NavData.CF_NavData.bias_g.Data(:,3)*r2d,'LineWidth',2,'Color',color6);
plot(out.tout,out.NavData.ER_NavData.INSbiasData.INSstate_dg.Data(:,3)*r2d,'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.INSbiasData.INSstate_dg.Data(:,3)*r2d,'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.INSbiasData.INSstate_dg.Data(:,3)*r2d,'LineWidth',2,'Color',color4);
ylabel('$$bg_z\,[deg/s]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'NCF','ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
axis([0 250 -0.25 0.4])

set(gca,'FontSize',12);

%% bias 
figure(Name='accl bias')
subplot1 = subplot(3,1,1);
hold on;
plot(out.tout,out.NavData.ER_NavData.INSbiasData.INSstate_da.Data(:,1),'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.INSbiasData.INSstate_da.Data(:,1),'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.INSbiasData.INSstate_da.Data(:,1),'LineWidth',2,'Color',color4);
ylabel('$$ba_x\,$$[m/s$$^2$$]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');   
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -0.02 0.15])

subplot1 = subplot(3,1,2);
hold on;
plot(out.tout,out.NavData.ER_NavData.INSbiasData.INSstate_da.Data(:,2),'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.INSbiasData.INSstate_da.Data(:,2),'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.INSbiasData.INSstate_da.Data(:,2),'LineWidth',2,'Color',color4);
ylabel('$$ba_y\,$$[m/s$$^2$$]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -0.25 0.15])

subplot1 = subplot(3,1,3);
hold on;
plot(out.tout,out.NavData.ER_NavData.INSbiasData.INSstate_da.Data(:,3),'LineWidth',2,'Color',color3);
plot(out.tout,out.NavData.LI_NavData.INSbiasData.INSstate_da.Data(:,3),'LineWidth',2,'Color',color5);
plot(out.tout,out.NavData.RI_NavData.INSbiasData.INSstate_da.Data(:,3),'LineWidth',2,'Color',color4);
ylabel('$$ba_z\,$$[m/s$$^2$$]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -0.01 0.02])

%% traj     2d
figure(Name='traj')
subplot1 = subplot(1,1,1);
hold on;
plot(out.NavData.SensorData.pos_E_gps.Data(:,1), ...
    out.NavData.SensorData.pos_N_gps.Data(:,1),'LineWidth',2,'Color',color1)
plot(out.NavData.ER_NavData.TranlationData.pe_m.Data(:,1), ...
    out.NavData.ER_NavData.TranlationData.pn_m.Data(:,1),'LineWidth',2,'Color',color3);
plot(out.NavData.RI_NavData.TranlationData.pe_m.Data(:,1), ...
    out.NavData.RI_NavData.TranlationData.pn_m.Data(:,1),'LineWidth',2,'Color',color4);
ylabel('$$pos_N\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('$$pos_E\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'RTK','ES-EKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
%% pos error
% gpsdely = 10;
% RIpos_N_gps_test(:,1) = [zeros(gpsdely,1);out.NavData.RI_NavData.TranlationData.pn_m.Data(1:end-gpsdely,1);];
figure(Name='pos error')
subplot1 = subplot(3,1,1);
interval = 20;
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout,(out.NavData.ER_NavData.TranlationData.pn_m.Data(:,1) - ...
    out.NavData.SensorData.pos_N_gps.Data(:,1)) ,'LineWidth',2,'Color',color3);

plot(out.tout, (out.NavData.LI_NavData.TranlationData.pn_m.Data(:,1) - ...
    out.NavData.SensorData.pos_N_gps.Data(:,1)),'LineWidth',2,'Color',color5);

plot(out.tout, (out.NavData.RI_NavData.TranlationData.pn_m.Data(:,1) - ...
    out.NavData.SensorData.pos_N_gps.Data(:,1)),'LineWidth',2,'Color',color4);

ylabel('$$\delta pos_n\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');

subplot1 = subplot(3,1,2);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout,(out.NavData.ER_NavData.TranlationData.pe_m.Data(:,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(:,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout, (out.NavData.LI_NavData.TranlationData.pe_m.Data(:,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(:,1)),'LineWidth',2,'Color',color5);

plot(out.tout, (out.NavData.RI_NavData.TranlationData.pe_m.Data(:,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(:,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta pos_e\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');

subplot1 = subplot(3,1,3);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout,(out.NavData.ER_NavData.TranlationData.pd_m.Data(:,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(:,1)) ,'LineWidth',2,'Color',color3);

plot(out.tout, (out.NavData.LI_NavData.TranlationData.pd_m.Data(:,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(:,1)),'LineWidth',2,'Color',color5);

plot(out.tout, (out.NavData.RI_NavData.TranlationData.pd_m.Data(:,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(:,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta pos_d\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);


%% traj     3d
figure(Name='traj3D')
subplot1 = subplot(1,1,1);
hold on;
plot3(out.NavData.SensorData.pos_E_gps.Data(:,1), ...
    out.NavData.SensorData.pos_N_gps.Data(:,1), ...
    -out.NavData.SensorData.pos_D_gps.Data(:,1),'LineWidth',2,'Color',color1)
plot3(out.NavData.ER_NavData.TranlationData.pe_m.Data(:,1), ...
    out.NavData.ER_NavData.TranlationData.pn_m.Data(:,1), ...
    -out.NavData.ER_NavData.TranlationData.pd_m.Data(:,1),'LineWidth',2,'Color',color3);
plot3(out.NavData.RI_NavData.TranlationData.pe_m.Data(:,1), ...
    out.NavData.RI_NavData.TranlationData.pn_m.Data(:,1), ...
   - out.NavData.RI_NavData.TranlationData.pd_m.Data(:,1),'LineWidth',2,'Color',color4);
ylabel('$$pos_N\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('$$pos_E\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
zlabel('$$pos_H\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');

box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'RTK','ES-EKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);


%%


% Lon_ESEKF = zeros(size(out.NavData.ER_NavData.TranlationData.pn_m.Data,1),1);
% Lat_ESEKF = Lon_ESEKF;
% Alt_ESEKF = Lon_ESEKF;
% 
% Lon_ESRIEKF = Lon_ESEKF;
% Lat_ESRIEKF = Lon_ESRIEKF;
% Alt_ESRIEKF = Lon_ESRIEKF;
% 
% Lon_ESLIEKF = Lon_ESEKF;
% Lat_ESLIEKF = Lon_ESEKF;
% Alt_ESLIEKF = Lon_ESEKF;
% 
% Lon_NCF = Lon_ESEKF;
% Lat_NCF = Lon_ESEKF;
% Alt_NCF = Lon_ESEKF;
% 
% pos_ESEKF = [out.NavData.ER_NavData.TranlationData.pn_m.Data(:,1),...
%             out.NavData.ER_NavData.TranlationData.pe_m.Data(:,1),...
%             out.NavData.ER_NavData.TranlationData.pd_m.Data(:,1)];
% pos_RIEKF = [out.NavData.RI_NavData.TranlationData.pn_m.Data(:,1),...
%             out.NavData.RI_NavData.TranlationData.pe_m.Data(:,1),...
%             out.NavData.RI_NavData.TranlationData.pd_m.Data(:,1)];
% pos_LIEKF = [out.NavData.LI_NavData.TranlationData.pn_m.Data(:,1),...
%             out.NavData.LI_NavData.TranlationData.pe_m.Data(:,1),...
%             out.NavData.LI_NavData.TranlationData.pd_m.Data(:,1)];
% LLA0 = [All.lon(1)/57.3+0.01,All.lat(1)/57.3+0.01,All.mhdot(1)];
% for i = 1:size(out.NavData.ER_NavData.TranlationData.pn_m.Data(:,1),1)
%     [ Lon_ESEKF(i),Lat_ESEKF(i),Alt_ESEKF(i) ] = TransformBDXTLLA(pos_ESEKF(i,:),LLA0(1,:));
%     [ Lon_ESRIEKF(i),Lat_ESRIEKF(i),Alt_ESRIEKF(i) ] = TransformBDXTLLA(pos_RIEKF(i,:),LLA0(1,:));
%     [ Lon_ESLIEKF(i),Lat_ESLIEKF(i),Alt_ESLIEKF(i) ] = TransformBDXTLLA(pos_LIEKF(i,:),LLA0(1,:));
%     %[ Lon_NCF(i),Lat_NCF(i),Alt_NCF(i) ] = TransformBDXTLLA(pos_RIEKF(i,:),LLA0(1,:));
% end
% 
% % wm = webmap('World Imagery');
% wm = webmap('World Shaded Relief');
% 
% s = geoshape(All.lat, All.lon); % lat, lon
% s1 = geoshape(Lat_ESEKF*57.3, Lon_ESEKF*57.3);
% s2 = geoshape(Lat_ESRIEKF*57.3, Lon_ESRIEKF*57.3);% lat, lon
% s3 = geoshape(Lat_ESLIEKF*57.3, Lon_ESLIEKF*57.3);% lat, lon
% %s4 = geoshape(Lat_NCF(131/0.005:220/0.005,:)*57.3, Lon_NCF(131/0.005:220/0.005,:)*57.3);% lat, lon
% 
% 
% wmline(s2,'Color', [0.1177,0.5647,1], 'Width', 3);
% wmline(s3,'Color', 'cyan', 'Width', 3);
% % wmline(s4,'Color', 'white', 'Width', 3);
% wmline(s,'Color', 'yellow','FeatureName','complefilter', 'Width', 3);
% wmline(s1,'Color', 'red', 'Width', 3);


%% wind
figure(Name='wind')
subplot1 = subplot(4,1,1);
hold on;
plot(out.tout,out.NavData.RI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,1),'LineWidth',2,'Color',color4);
plot(out.tout, out.NavData.LI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,1),'LineWidth',2,'Color',color3);
ylabel('$$v^{wind}_N\,$$[m/s]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-RIEKF','ES-LIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([20 200 -2.5 2]);

subplot1 = subplot(4,1,2);
hold on;
plot(out.tout,out.NavData.RI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,2),'LineWidth',2,'Color',color4);
plot(out.tout, out.NavData.LI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,2),'LineWidth',2,'Color',color3);
ylabel('$$v^{wind}_E\,$$[m/s]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-RIEKF','ES-LIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([20 200 -2 2]);

subplot1 = subplot(4,1,3);
hold on;
plot(out.tout,out.NavData.RI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,3),'LineWidth',2,'Color',color4);
plot(out.tout, out.NavData.LI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,3),'LineWidth',2,'Color',color3);
ylabel('$$v^{wind}_D\,$$[m/s]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-RIEKF','ES-LIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([20 200 -0.4 0.3]);

subplot1 = subplot(4,1,4);
vwind_RI = sqrt((out.NavData.RI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,1)).^2+...
             (out.NavData.RI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,2)).^2+...
             (out.NavData.RI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,3)).^2);
vwind_LI = sqrt((out.NavData.LI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,1)).^2+...
             (out.NavData.LI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,2)).^2+...
             (out.NavData.LI_NavData.AOA_SA_Wind.vel_wind_NED.Data(:,3)).^2);
hold on;
plot(out.tout,vwind_RI,'LineWidth',2,'Color',color4);
plot(out.tout,vwind_LI,'LineWidth',2,'Color',color3);

ylabel('$$|v^{wind}| \,$$[m/s]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-RIEKF','ES-LIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([20 200 0 3]);
%%
figure('Name','flight phase')
subplot1 = subplot(1,1,1)
%ph0 = find(All.fmphase== 0); %起飞前准备
ph1 = find(All.fmphase == 1); %起飞阶段
% ph2 = find(All.fmphase == 2); %爬升阶段
ph3 = find(All.fmphase == 3); %巡航阶段
ph4 = find(All.fmphase== 4); %进场阶段
ph5 = find(All.fmphase == 5); %着陆阶段
ph6 = find(All.fmphase == 6); %地面滑行阶段
hold on
plot3(All.Yeast(ph1),All.Xnorth(ph1),All.mh(ph1),'LineWidth',2,'color',color6);
% plot3(All.Yeast(ph2),All.Xnorth(ph2),All.mh(ph2),'LineWidth',2,'color',color6);
plot3(All.Yeast(ph3),All.Xnorth(ph3),All.mh(ph3),'LineWidth',2,'color',color3);
plot3(All.Yeast(ph4),All.Xnorth(ph4),All.mh(ph4),'LineWidth',2,'color',color3);

plot3(All.Yeast(ph5),All.Xnorth(ph5),All.mh(ph5),'LineWidth',2,'color',color4);
plot3(All.Yeast(ph6),All.Xnorth(ph6),All.mh(ph6),'LineWidth',2,'color',color5);

ylabel('$$pos_N\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('$$pos_E\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
zlabel('$$pos_H\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');

box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
% legend({'RTK','ES-EKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% err calulation

start_time = 1;
end_time = 220;
ts = 0.01;
step = 1; 
% fd =  fopen('.\Results\2outdoor_rect_fast_3547.2g_113.21s.txt','w');
fd =  fopen("误差统计\156_wholeflightwithGNSS.txt",'w');

ref_att = [All.phiL(1:22001),All.thetaL(1:22001),All.psiL(1:22001)];
CF_att = [out.NavData.CF_NavData.RotationData.phi_rad.Data(:,1),...
        out.NavData.CF_NavData.RotationData.theta_rad.Data(:,1),...
        out.NavData.CF_NavData.RotationData.psi_rad.Data(:,1)];
ESEKF_att = [out.NavData.ER_NavData.RotationData.phi_rad.Data(:,1),...
        out.NavData.ER_NavData.RotationData.theta_rad.Data(:,1),...
        out.NavData.ER_NavData.RotationData.psi_rad.Data(:,1)];
ESLIEKF_att = [out.NavData.LI_NavData.RotationData.phi_rad.Data(:,1),...
        out.NavData.LI_NavData.RotationData.theta_rad.Data(:,1),...
        out.NavData.LI_NavData.RotationData.psi_rad.Data(:,1)];
ESRIEKF_att = [out.NavData.RI_NavData.RotationData.phi_rad.Data(:,1),...
        out.NavData.RI_NavData.RotationData.theta_rad.Data(:,1),...
        out.NavData.RI_NavData.RotationData.psi_rad.Data(:,1)];
[CF_att_std,CF_att_rmse] = err_cal(CF_att(start_time/ts:step:end_time/ts,:),ref_att(start_time/ts:step:end_time/ts,:),'att');
[ESEKF_att_std,ESEKF_att_rmse] = err_cal(ESEKF_att(start_time/ts:step:end_time/ts,:),ref_att(start_time/ts:step:end_time/ts,:),'att');
[ESLIEKF_att_std,ESLIEKF_att_rmse] = err_cal(ESLIEKF_att(start_time/ts:step:end_time/ts,:),ref_att(start_time/ts:step:end_time/ts,:),'att');
[ESRIEKF_att_std,ESRIEKF_att_rmse] = err_cal(ESRIEKF_att(start_time/ts:step:end_time/ts,:),ref_att(start_time/ts:step:end_time/ts,:),'att');


% fprintf(fd,'CF_att_std = %f %f %f,-----CF_att_rmse = %f %f %f -----CF_mean_rmse = %f\n', ...
%     CF_att_std,CF_att_rmse,mean(CF_att_rmse));
% fprintf(fd,'ESEKF_att_std = %f %f %f,-----ESEKF_att_rmse = %f %f %f -----ESEKF_att_mean_rmse = %f\n', ...
%     ESEKF_att_std,ESEKF_att_rmse,mean(ESEKF_att_rmse));
% fprintf(fd,'ESLIEKF_att_std = %f %f %f,----ESLIEKF_att_rmse = %f %f %f -----ESLIEKF_att_mean_rmse = %f\n', ...
%     ESLIEKF_att_std,ESLIEKF_att_rmse,mean(ESLIEKF_att_rmse));
% fprintf(fd,'ESRIEKF_att_std = %f %f %f,----ESRIEKF_att_rmse = %f %f %f -----ESRIEKF_att_mean_rmse = %f\n', ...
%     ESRIEKF_att_std,ESRIEKF_att_rmse,mean(ESRIEKF_att_rmse));
% fclose('all');
%% pos error calculation
fd =  fopen("误差统计\156_poserror_wholeflightwithGNSS.txt",'w');
pos_N_gps =  out.NavData.SensorData.pos_N_gps.Data(:,1) - 0.4;
pos_E_gps =  out.NavData.SensorData.pos_E_gps.Data(:,1);
pos_D_gps =  out.NavData.SensorData.pos_D_gps.Data(:,1);

poserr_ESEKF_x = abs(out.NavData.ER_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    pos_N_gps(enable_index,1));
poserr_ESLIEKF_x = abs(out.NavData.LI_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    pos_N_gps(enable_index,1));
poserr_ESRIEKF_x = abs(out.NavData.RI_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    pos_N_gps(enable_index,1));

poserr_ESEKF_y = abs(out.NavData.ER_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    pos_E_gps(enable_index,1));
poserr_ESLIEKF_y = abs(out.NavData.LI_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    pos_E_gps(enable_index,1));
poserr_ESRIEKF_y = abs(out.NavData.RI_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    pos_E_gps(enable_index,1));

poserr_ESEKF_z = abs(out.NavData.ER_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    pos_D_gps(enable_index,1));
poserr_ESLIEKF_z = abs(out.NavData.LI_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    pos_D_gps(enable_index,1));
poserr_ESRIEKF_z = abs(out.NavData.RI_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    pos_D_gps(enable_index,1));

poserr_ESEKF = [poserr_ESEKF_x,poserr_ESEKF_y,poserr_ESEKF_z];
poserr_ESEKF_std = mean(poserr_ESEKF);
poserr_ESEKF_rmse = sqrt(sum(poserr_ESEKF.^2)/size(poserr_ESEKF,1));

poserr_ESLIEKF = [poserr_ESLIEKF_x,poserr_ESLIEKF_y,poserr_ESLIEKF_z];
poserr_ESLIEKF_std = mean(poserr_ESLIEKF);
poserr_ESLIEKF_rmse = sqrt(sum(poserr_ESLIEKF.^2)/size(poserr_ESEKF,1));

poserr_ESRIEKF = [poserr_ESRIEKF_x,poserr_ESRIEKF_y,poserr_ESRIEKF_z];
poserr_ESRIEKF_std = mean(poserr_ESRIEKF);
poserr_ESRIEKF_rmse = sqrt(sum(poserr_ESRIEKF.^2)/size(poserr_ESEKF,1));


fprintf(fd,'ESEKF_pos_std = %f %f %f,-----ESEKF_pos_mean_rmse = %f\n ESEKF_pos_rmse = %f %f %f -----ESEKF_pos_mean_rmse = %f\n', ...
    poserr_ESEKF_std,mean(poserr_ESEKF_std),poserr_ESEKF_rmse,mean(poserr_ESEKF_rmse));
fprintf(fd,'ESLIEKF_pos_std = %f %f %f,-----ESLIEKF_pos_mean_rmse = %f\n ESLIEKF_pos_rmse = %f %f %f -----ESLIEKF_pos_mean_rmse = %f\n', ...
    poserr_ESLIEKF_std,mean(poserr_ESLIEKF_std),poserr_ESLIEKF_rmse,mean(poserr_ESLIEKF_rmse));
fprintf(fd,'ESRIEKF_pos_std = %f %f %f,-----ESRIEKF_pos_mean_rmse = %f\n ESRIEKF_pos_rmse = %f %f %f -----ESRIEKF_pos_mean_rmse = %f\n', ...
    poserr_ESRIEKF_std,mean(poserr_ESRIEKF_std),poserr_ESRIEKF_rmse,mean(poserr_ESRIEKF_rmse));
fclose('all');


%% pos error
% gpsdely = 10;
% RIpos_N_gps_test(:,1) = [zeros(gpsdely,1);out.NavData.RI_NavData.TranlationData.pn_m.Data(1:end-gpsdely,1);];
pos_N_gps =  out.NavData.SensorData.pos_N_gps.Data(:,1) - 0.4;
figure('Name','pos error')
subplot1 = subplot(3,1,1);
interval = 20;
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
% plot(out.tout(enable_index),(out.NavData.CF_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
%     pos_N_gps(enable_index,1)),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),(out.NavData.ER_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    pos_N_gps(enable_index,1)) ,'LineWidth',2,'Color',color3);

plot(out.tout(enable_index), (out.NavData.LI_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    pos_N_gps(enable_index,1)),'LineWidth',2,'Color',color5);

plot(out.tout(enable_index), (out.NavData.RI_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    pos_N_gps(enable_index,1)),'LineWidth',2,'Color',color4);

ylabel('$$\delta pos_n\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -2 4]);

subplot1 = subplot(3,1,2);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),(out.NavData.ER_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), (out.NavData.LI_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1)),'LineWidth',2,'Color',color5);

plot(out.tout(enable_index), (out.NavData.RI_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta pos_e\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -3 4]);

subplot1 = subplot(3,1,3);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),(out.NavData.ER_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);

plot(out.tout(enable_index), (out.NavData.LI_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1)),'LineWidth',2,'Color',color5);

plot(out.tout(enable_index), (out.NavData.RI_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta pos_d\,$$[m]','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-LIEKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
set(gca,'FontSize',12);
axis([0 250 -1.5 2]);
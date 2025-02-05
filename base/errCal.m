%% vel / pos error calculation
%% vel err
GNSSflag = out.NavData.RI_NavData.GNSS_enable_flag.GNSS_enable_flag.Data(:,1);
enable_index = find(GNSSflag == 1);
figure(5)
interval = 20;
subplot1 = subplot(3,1,1);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),abs(out.NavData.ER_NavData.TranlationData.vn_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vn_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), abs(out.NavData.RI_NavData.TranlationData.vn_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vn_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta vel_N\,[m/s]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');

subplot1 = subplot(3,1,2);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),abs(out.NavData.ER_NavData.TranlationData.ve_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.ve_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), abs(out.NavData.RI_NavData.TranlationData.ve_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.ve_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta vel_E\,[m/s]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');

subplot1 = subplot(3,1,3);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),abs(out.NavData.ER_NavData.TranlationData.vd_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vd_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), abs(out.NavData.RI_NavData.TranlationData.vd_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vd_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta vel_D\,[m/s]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');

%% pos error
figure(6)
subplot1 = subplot(3,1,1);
interval = 20;
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),abs(out.NavData.ER_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_N_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), abs(out.NavData.RI_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_N_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta pos_n\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');

subplot1 = subplot(3,1,2);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),abs(out.NavData.ER_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), abs(out.NavData.RI_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta pos_e\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');

subplot1 = subplot(3,1,3);
% plot(All.t,All.vn,'LineWidth',2,'Color',color1);
hold on;
%plot(out.tout,out.NavData.CF_NavData.TranlationData.vn_m_s.Data(:,1),'LineWidth',2,'Color',color6);
plot(out.tout(enable_index),abs(out.NavData.ER_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1)) ,'LineWidth',2,'Color',color3);
plot(out.tout(enable_index), abs(out.NavData.RI_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1)),'LineWidth',2,'Color',color4);
ylabel('$$\delta pos_d\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
legend({'ES-EKF','ES-RIEKF'},'NumColumns',4,'Location','north');
legend('boxoff');
%%
fd =  fopen('.\ErrResult\01.txt','w');

dvx_ESEKF = abs(out.NavData.ER_NavData.TranlationData.vn_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vn_gps.Data(enable_index,1));
dvy_ESEKF = abs(out.NavData.ER_NavData.TranlationData.ve_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.ve_gps.Data(enable_index,1));
dvz_ESEKF = abs(out.NavData.ER_NavData.TranlationData.vd_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vd_gps.Data(enable_index,1));

dvx_RIEKF = abs(out.NavData.RI_NavData.TranlationData.vn_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vn_gps.Data(enable_index,1));
dvy_RIEKF = abs(out.NavData.RI_NavData.TranlationData.ve_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.ve_gps.Data(enable_index,1));
dvz_RIEKF = abs(out.NavData.RI_NavData.TranlationData.vd_m_s.Data(enable_index,1) - ...
    out.NavData.SensorData.vd_gps.Data(enable_index,1));

dpx_ESEKF = abs(out.NavData.ER_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_N_gps.Data(enable_index,1));
dpy_ESEKF = abs(out.NavData.ER_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1));
dpz_ESEKF = abs(out.NavData.ER_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1));

dpx_RIEKF = abs(out.NavData.RI_NavData.TranlationData.pn_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_N_gps.Data(enable_index,1));
dpy_RIEKF = abs(out.NavData.RI_NavData.TranlationData.pe_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_E_gps.Data(enable_index,1));
dpz_RIEKF = abs(out.NavData.RI_NavData.TranlationData.pd_m.Data(enable_index,1) - ...
    out.NavData.SensorData.pos_D_gps.Data(enable_index,1));

dv_ESEKF = [dvx_ESEKF,dvy_ESEKF,dvz_ESEKF];
dv_RIEKF = [dvx_RIEKF,dvy_RIEKF,dvz_RIEKF];
[ESEKF_vel_std,ESEKF_vel_rmse] = err_cal(dv_ESEKF);
[RIEKF_vel_std,RIEKF_vel_rmse] = err_cal(dv_RIEKF);

dp_ESEKF = [dpx_ESEKF,dpy_ESEKF,dpz_ESEKF];
dp_RIEKF = [dpx_RIEKF,dpy_RIEKF,dpz_RIEKF];
[ESEKF_pos_std,ESEKF_pos_rmse] = err_cal(dp_ESEKF);
[RIEKF_pos_std,RIEKF_pos_rmse] = err_cal(dp_RIEKF);

fprintf(fd,'vel_ESEKF_std = %f %f %f,-----vel_ESEKF_rmse = %f %f %f \n',ESEKF_vel_std,ESEKF_vel_rmse);
fprintf(fd,'vel_RIEKF_std = %f %f %f,-----vel_RIEKF_rmse = %f %f %f \n',RIEKF_vel_std,RIEKF_vel_rmse);

fprintf(fd,'pos_ESEKF_std = %f %f %f,-----pos_ESEKF_rmse = %f %f %f \n',ESEKF_pos_std,ESEKF_pos_rmse);
fprintf(fd,'pos_RIEKF_std = %f %f %f,-----pos_RIEKF_rmse = %f %f %f \n',RIEKF_pos_std,RIEKF_pos_rmse);

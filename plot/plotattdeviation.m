% Alldata = Alldata_vn_deviation;
figure("Name",'att deviation')

subplot1 = subplot(2,5,1);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.ER_NavData.RotationData.phi_rad.Data(:,1));
end
plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.RotationData.phi_rad.Data(:,1), ...
    'LineStyle','--','LineWidth',2,'Color','r')
axis([15,30,-50,50])
ylabel('$$\phi\,[deg]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;

subplot1 = subplot(2,5,6);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.RI_NavData.RotationData.phi_rad.Data(:,1));
end
% plot(All.t,All.phiL,'LineWidth',2,'Color','r')

plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.RotationData.phi_rad.Data(:,1), ...
    'LineWidth',2,'LineStyle','--','Color','r')
axis([15,30,-50,50])
ylabel('$$\phi\,[deg]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;

%%
subplot1 = subplot(2,5,2);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.ER_NavData.RotationData.theta_rad.Data(:,1));
end
plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.RotationData.theta_rad.Data(:,1), ...
    'LineStyle','--','LineWidth',2,'Color','r')
axis([15,30,-50,70])
ylabel('$$\theta\,[deg]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;

subplot1 = subplot(2,5,7);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.RI_NavData.RotationData.theta_rad.Data(:,1));
end
% plot(All.t,All.phiL,'LineWidth',2,'Color','r')

plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.RotationData.theta_rad.Data(:,1), ...
    'LineWidth',2,'LineStyle','--','Color','r')
axis([15,30,-50,70])
ylabel('$$\theta\,[deg]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;
%%
% subplot1 = subplot(2,3,3);
% for ii = 1:60
%     hold on
%     plot(Alldata{ii}.tout,Alldata{ii}.NavData.ER_NavData.RotationData.psi_rad.Data(:,1));
% end
% plot(Alldata{30}.tout, ...
%     Alldata{30}.NavData.RI_NavData.RotationData.psi_rad.Data(:,1), ...
%     'LineStyle','--','LineWidth',2,'Color','r')
% % axis([15,30,-50,70])
% ylabel('$$\theta\,[deg]$$','FontSize',12,'FontName','Times','Interpreter','latex');
% xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
% box(subplot1,'on');
% grid(subplot1,'on');
% hold(subplot1,'off');
% set(subplot1,'FontName','Times','GridLineStyle',...
%     '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
% grid on;
% 
% subplot1 = subplot(2,3,6);
% for ii = 1:60
%     hold on
%     plot(Alldata{ii}.tout,Alldata{ii}.NavData.RI_NavData.RotationData.psi_rad.Data(:,1));
% end
% % plot(All.t,All.phiL,'LineWidth',2,'Color','r')
% 
% plot(Alldata{30}.tout, ...
%     Alldata{30}.NavData.RI_NavData.RotationData.psi_rad.Data(:,1), ...
%     'LineWidth',2,'LineStyle','--','Color','r')
% % axis([15,30,-50,70])
% ylabel('$$\theta\,[deg]$$','FontSize',12,'FontName','Times','Interpreter','latex');
% xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
% box(subplot1,'on');
% grid(subplot1,'on');
% hold(subplot1,'off');
% set(subplot1,'FontName','Times','GridLineStyle',...
%     '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
% grid on;
%% pos x
subplot1 = subplot(2,5,3);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.ER_NavData.TranlationData.pn_m.Data(:,1));
end
plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.TranlationData.pn_m.Data(:,1), ...
    'LineStyle','--','LineWidth',2,'Color','r')
axis([15,25,-40,10]);
ylabel('$$posx\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;

subplot1 = subplot(2,5,8);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.RI_NavData.TranlationData.pn_m.Data(:,1));
end
% plot(All.t,All.phiL,'LineWidth',2,'Color','r')

plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.TranlationData.pn_m.Data(:,1), ...
    'LineWidth',2,'LineStyle','--','Color','r')
axis([15,25,-40,2]);
ylabel('$$posx\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;

%% pos y
subplot1 = subplot(2,5,4);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.ER_NavData.TranlationData.pe_m.Data(:,1));
end
plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.TranlationData.pe_m.Data(:,1), ...
    'LineStyle','--','LineWidth',2,'Color','r')
axis([15,25,-8,8]);
ylabel('$$posy\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;

subplot1 = subplot(2,5,9);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.RI_NavData.TranlationData.pe_m.Data(:,1));
end
% plot(All.t,All.phiL,'LineWidth',2,'Color','r')

plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.TranlationData.pe_m.Data(:,1), ...
    'LineWidth',2,'LineStyle','--','Color','r')
axis([15,25,-8,8]);
ylabel('$$posy\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;

%% pos z
subplot1 = subplot(2,5,5);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.ER_NavData.TranlationData.pd_m.Data(:,1));
end
plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.TranlationData.pd_m.Data(:,1), ...
    'LineStyle','--','LineWidth',2,'Color','r')
axis([15,25,-4,4]);
ylabel('$$posz\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;

subplot1 = subplot(2,5,10);
for ii = 1:60
    hold on
    plot(Alldata{ii}.tout,Alldata{ii}.NavData.RI_NavData.TranlationData.pd_m.Data(:,1));
end
% plot(All.t,All.phiL,'LineWidth',2,'Color','r')

plot(Alldata{30}.tout, ...
    Alldata{30}.NavData.RI_NavData.TranlationData.pd_m.Data(:,1), ...
    'LineWidth',2,'LineStyle','--','Color','r')
axis([15,25,-4,4]);
ylabel('$$posz\,[m]$$','FontSize',12,'FontName','Times','Interpreter','latex');
xlabel('time [s]','FontSize',12,'FontName','Times','Interpreter','latex');
box(subplot1,'on');
grid(subplot1,'on');
hold(subplot1,'off');
set(subplot1,'FontName','Times','GridLineStyle',...
    '--','LineWidth',0.8,'XMinorTick','on','YMinorTick','on','ZMinorTick','on');
grid on;